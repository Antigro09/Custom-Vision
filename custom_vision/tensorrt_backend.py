"""Small TensorRT 10 runtime using JetPack's CUDA runtime, without PyTorch."""

from __future__ import annotations

import ctypes
import ctypes.util
from pathlib import Path
import threading
import time

import numpy as np


class _CudaRuntime:
    """The CUDA calls used here have stable C signatures across JetPack 6."""

    def __init__(self) -> None:
        candidates = [
            ctypes.util.find_library("cudart"),
            "libcudart.so.12",
            "/usr/local/cuda/lib64/libcudart.so",
            "/usr/local/cuda/targets/aarch64-linux/lib/libcudart.so",
        ]
        self.lib = None
        for candidate in candidates:
            if candidate:
                try:
                    self.lib = ctypes.CDLL(candidate)
                    break
                except OSError:
                    continue
        if self.lib is None:
            raise RuntimeError("CUDA runtime libcudart was not found; install the JetPack CUDA runtime.")
        signatures = {
            "cudaSetDevice": [ctypes.c_int],
            "cudaMalloc": [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t],
            "cudaFree": [ctypes.c_void_p],
            "cudaMemcpyAsync": [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int, ctypes.c_void_p],
            "cudaHostAlloc": [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t, ctypes.c_uint],
            "cudaFreeHost": [ctypes.c_void_p],
            "cudaStreamCreate": [ctypes.POINTER(ctypes.c_void_p)],
            "cudaStreamSynchronize": [ctypes.c_void_p],
            "cudaStreamDestroy": [ctypes.c_void_p],
        }
        for name, argtypes in signatures.items():
            function = getattr(self.lib, name)
            function.argtypes = argtypes
            function.restype = ctypes.c_int
        self.lib.cudaGetErrorString.argtypes = [ctypes.c_int]
        self.lib.cudaGetErrorString.restype = ctypes.c_char_p
        self.check(self.lib.cudaSetDevice(0), "select CUDA device 0")

    def check(self, result: int, operation: str) -> None:
        if result:
            detail = self.lib.cudaGetErrorString(result)
            message = detail.decode("utf-8", errors="replace") if detail else str(result)
            raise RuntimeError(f"CUDA could not {operation}: {message}")

    def allocate(self, size: int) -> ctypes.c_void_p:
        pointer = ctypes.c_void_p()
        self.check(self.lib.cudaMalloc(ctypes.byref(pointer), size), "allocate device memory")
        return pointer


class TensorRTEngine:
    """Static batch-one YOLO execution with persistent device and pinned host buffers.

    Each instance owns one context and CUDA stream. infer() serializes use and
    returns independent arrays, or decodes under the lock to avoid output copies.
    Segmentation returns (predictions, prototypes), regardless of tensor names.
    Input is normalized RGB NCHW. No PyTorch or Ultralytics runtime is imported.
    """

    def __init__(self, model_path: str | Path, num_classes: int, *,
                 output_format: str = "yolov8_raw", task: str = "detect") -> None:
        self._lock = threading.Lock()
        self._cuda = None
        self._stream = ctypes.c_void_p()
        self._device = {}
        self._host_pointers = {}
        self._host = {}
        self._closed = True
        self.last_timings = {}
        path = Path(model_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"TensorRT engine does not exist: {path}")
        if num_classes < 1:
            raise ValueError("At least one object label is required.")
        if output_format not in ("yolov8_raw", "yolo26_end2end") or task not in ("detect", "segment"):
            raise ValueError("Use output_format yolov8_raw|yolo26_end2end and task detect|segment.")
        self.output_format, self.task = output_format, task
        try:
            import tensorrt as trt
        except ImportError as exc:
            raise RuntimeError(
                "TensorRT Python bindings are unavailable. Use the JetPack system Python "
                "and a virtual environment created with --system-site-packages."
            ) from exc
        if int(trt.__version__.split(".")[0]) < 10:
            raise RuntimeError("This backend requires TensorRT 10 or later (JetPack 6).")
        self._logger = trt.Logger(trt.Logger.WARNING)
        trt.init_libnvinfer_plugins(self._logger, "")
        self._cuda = _CudaRuntime()
        self._runtime = trt.Runtime(self._logger)
        try:
            self._engine = self._runtime.deserialize_cuda_engine(path.read_bytes())
            if self._engine is None:
                raise RuntimeError(
                    f"Cannot deserialize {path}. Build a raw TensorRT plan on this Jetson using trtexec; "
                    "engines from other TensorRT versions/devices or with an Ultralytics metadata header are unsupported."
                )
            names = [self._engine.get_tensor_name(i) for i in range(self._engine.num_io_tensors)]
            inputs = [name for name in names if self._engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT]
            outputs = [name for name in names if self._engine.get_tensor_mode(name) == trt.TensorIOMode.OUTPUT]
            if len(inputs) != 1 or len(outputs) != (2 if task == "segment" else 1):
                raise ValueError("Expected one RGB input and one detection output, plus one prototype output for segment.")
            shapes = {name: tuple(self._engine.get_tensor_shape(name)) for name in names}
            if any(dimension <= 0 for shape in shapes.values() for dimension in shape):
                raise ValueError("Dynamic TensorRT shapes are unsupported; export with dynamic=False, batch=1.")
            if any(np.prod(shape) > 16_777_216 for shape in shapes.values()):
                raise ValueError("Engine I/O exceeds 16M values per tensor; use a bounded input and prototype resolution.")
            self.input_name = inputs[0]
            self.input_shape = shapes[self.input_name]
            if len(self.input_shape) != 4 or self.input_shape[:2] != (1, 3):
                raise ValueError(f"Expected detector input [1,3,H,W], got {self.input_shape}.")
            predictions = [name for name in outputs if len(shapes[name]) == 3]
            prototypes = [name for name in outputs if len(shapes[name]) == 4]
            if len(predictions) != 1 or len(prototypes) != (1 if task == "segment" else 0):
                raise ValueError("Expected rank-3 predictions and rank-4 prototypes only for segmentation.")
            self.output_name = predictions[0]
            self.output_names = predictions + prototypes
            self.output_shape = shapes[self.output_name]
            mask_channels = 0
            if prototypes:
                shape = shapes[prototypes[0]]
                if shape[0] != 1 or not 1 <= shape[1] <= 128 or np.prod(shape) > 128 * 320 * 320:
                    raise ValueError("Mask prototypes must have bounded shape [1,nm,H,W], nm <=128.")
                mask_channels = shape[1]
            expected_channels = (4 + num_classes if output_format == "yolov8_raw" else 6) + mask_channels
            channel_index = 1 if output_format == "yolov8_raw" else 2
            if self.output_shape[0] != 1 or self.output_shape[channel_index] != expected_channels:
                raise ValueError(
                    f"Expected {output_format} output with {expected_channels} channels for {num_classes} labels; "
                    f"got {self.output_shape}. Check output_format, task and training class order."
                )
            for name in names:
                if self._engine.get_tensor_location(name) != trt.TensorLocation.DEVICE:
                    raise ValueError(f"Tensor {name} must reside on the device.")
                if self._engine.get_tensor_format(name) != trt.TensorFormat.LINEAR:
                    raise ValueError(f"Tensor {name} must use the LINEAR memory format.")
                if self._engine.get_tensor_dtype(name) not in (trt.float32, trt.float16):
                    raise ValueError(f"Tensor {name} must use FP32 or FP16 I/O.")
            self.input_dtype = np.dtype(trt.nptype(self._engine.get_tensor_dtype(self.input_name)))
            self._context = self._engine.create_execution_context()
            if self._context is None:
                raise RuntimeError("TensorRT could not create an execution context; check available GPU memory.")
            for name in names:
                dtype = np.dtype(trt.nptype(self._engine.get_tensor_dtype(name)))
                nbytes = int(np.prod(shapes[name])) * dtype.itemsize
                self._device[name] = self._cuda.allocate(nbytes)
                pointer = ctypes.c_void_p()
                self._cuda.check(self._cuda.lib.cudaHostAlloc(ctypes.byref(pointer), nbytes, 0), "allocate pinned host memory")
                self._host_pointers[name] = pointer
                memory = (ctypes.c_uint8 * nbytes).from_address(pointer.value)
                self._host[name] = np.frombuffer(memory, dtype=dtype).reshape(shapes[name])
                if not self._context.set_tensor_address(name, self._device[name].value):
                    raise RuntimeError(f"TensorRT rejected the buffer address for {name}.")
            self._cuda.check(self._cuda.lib.cudaStreamCreate(ctypes.byref(self._stream)), "create a CUDA stream")
            self._closed = False
        except Exception:
            self._release()
            raise

    def infer(self, tensor: np.ndarray, decoder=None):
        """Infer and optionally decode protected reusable outputs without copies.

        A decoder must return independent data and not retain array views. Normal
        callers receive independent NumPy arrays, safe across later/concurrent calls.
        Timings use host wall time and include copies/synchronization, not CUDA events.
        """
        with self._lock:
            if self._closed:
                raise RuntimeError("TensorRT engine is closed.")
            if tuple(tensor.shape) != self.input_shape:
                raise ValueError(f"Expected TensorRT input {self.input_shape}, got {tuple(tensor.shape)}.")
            started = time.perf_counter()
            host = self._host[self.input_name]
            np.copyto(host, tensor, casting="same_kind")
            cuda = self._cuda
            cuda.check(cuda.lib.cudaSetDevice(0), "select CUDA device 0")
            cuda.check(cuda.lib.cudaMemcpyAsync(self._device[self.input_name], ctypes.c_void_p(host.ctypes.data),
                                               host.nbytes, 1, self._stream), "copy input to GPU")
            try:
                if not self._context.execute_async_v3(stream_handle=self._stream.value):
                    raise RuntimeError("TensorRT inference failed.")
                for name in self.output_names:
                    output = self._host[name]
                    cuda.check(cuda.lib.cudaMemcpyAsync(ctypes.c_void_p(output.ctypes.data), self._device[name],
                                                       output.nbytes, 2, self._stream), "copy output from GPU")
            finally:
                # Also drain queued work before buffers can be reused on an error.
                cuda.check(cuda.lib.cudaStreamSynchronize(self._stream), "finish inference and copies")
            inferred = time.perf_counter()
            values = [self._host[name] for name in self.output_names]
            outputs = values[0] if self.task == "detect" else tuple(values)
            if decoder is not None:
                result = decoder(outputs)
            else:
                result = outputs.copy() if self.task == "detect" else tuple(value.copy() for value in outputs)
            self.last_timings = {"inference_ms": (inferred - started) * 1000,
                                 "decode_ms": (time.perf_counter() - inferred) * 1000}
            return result

    def _release(self) -> None:
        if self._cuda is not None:
            library = self._cuda.lib
            library.cudaSetDevice(0)
            if self._stream.value:
                library.cudaStreamSynchronize(self._stream)
            # Views are private and never returned by the default inference API.
            self._host.clear()
            for pointer in self._host_pointers.values():
                if pointer.value:
                    library.cudaFreeHost(pointer)
                    pointer.value = None
            for pointer in self._device.values():
                if pointer.value:
                    library.cudaFree(pointer)
                    pointer.value = None
            if self._stream.value:
                library.cudaStreamDestroy(self._stream)
                self._stream.value = None
        self._closed = True
        self._context = None
        self._engine = None
        self._runtime = None

    def close(self) -> None:
        with self._lock:
            self._release()

    def __enter__(self) -> "TensorRTEngine":
        return self

    def __exit__(self, *_args) -> None:
        self.close()

    def __del__(self) -> None:
        try:
            self._release()
        except Exception:
            pass
