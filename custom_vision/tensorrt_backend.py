"""Small TensorRT 10 runtime using JetPack's CUDA runtime, without PyTorch."""

from __future__ import annotations

import ctypes
import ctypes.util
from pathlib import Path
import threading

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
            "cudaMemcpy": [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int],
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
    """Execute one static, batch-one, RGB NCHW detector engine on CUDA device 0.

    The supported contract is one FP32/FP16 input [1,3,H,W] and one raw
    FP32/FP16 output [1,4+num_classes,N]. Build the engine on the target Jetson
    with trtexec; Ultralytics metadata-prefixed .engine files are not raw plans.
    Calls are serialized because one execution context owns reusable buffers.
    """

    def __init__(self, model_path: str | Path, num_classes: int) -> None:
        self._lock = threading.Lock()
        self._cuda = None
        self._stream = ctypes.c_void_p()
        self._input_device = ctypes.c_void_p()
        self._output_device = ctypes.c_void_p()
        self._closed = True
        path = Path(model_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"TensorRT engine does not exist: {path}")
        if num_classes < 1:
            raise ValueError("At least one object label is required.")
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
        except Exception as exc:
            raise RuntimeError(f"Cannot load TensorRT engine {path}; rebuild it on this Jetson with trtexec.") from exc
        if self._engine is None:
            raise RuntimeError(
                f"Cannot deserialize {path}. Build a raw TensorRT plan on this Jetson using trtexec; "
                "engines from other TensorRT versions/devices or with an Ultralytics metadata header "
                "are unsupported."
            )
        names = [self._engine.get_tensor_name(i) for i in range(self._engine.num_io_tensors)]
        inputs = [name for name in names if self._engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT]
        outputs = [name for name in names if self._engine.get_tensor_mode(name) == trt.TensorIOMode.OUTPUT]
        if len(inputs) != 1 or len(outputs) != 1:
            raise ValueError("Expected exactly one input and one raw YOLO output; export with nms=False.")
        self.input_name, self.output_name = inputs[0], outputs[0]
        self.input_shape = tuple(self._engine.get_tensor_shape(self.input_name))
        self.output_shape = tuple(self._engine.get_tensor_shape(self.output_name))
        if any(dimension <= 0 for shape in (self.input_shape, self.output_shape) for dimension in shape):
            raise ValueError("Dynamic TensorRT shapes are unsupported; export with dynamic=False, batch=1.")
        if len(self.input_shape) != 4 or self.input_shape[:2] != (1, 3):
            raise ValueError(f"Expected detector input [1,3,H,W], got {self.input_shape}.")
        if len(self.output_shape) != 3 or self.output_shape[:2] != (1, 4 + num_classes):
            raise ValueError(
                f"Expected raw YOLO output [1,{4 + num_classes},N] for {num_classes} labels; "
                f"got {self.output_shape}. YOLOv5, segmentation, pose, and embedded-NMS outputs are unsupported."
            )
        for name in names:
            if self._engine.get_tensor_location(name) != trt.TensorLocation.DEVICE:
                raise ValueError(f"Tensor {name} must reside on the device.")
            if self._engine.get_tensor_format(name) != trt.TensorFormat.LINEAR:
                raise ValueError(f"Tensor {name} must use the LINEAR memory format.")
            if self._engine.get_tensor_dtype(name) not in (trt.float32, trt.float16):
                raise ValueError(f"Tensor {name} must use FP32 or FP16 I/O.")
        self.input_dtype = np.dtype(trt.nptype(self._engine.get_tensor_dtype(self.input_name)))
        output_dtype = np.dtype(trt.nptype(self._engine.get_tensor_dtype(self.output_name)))
        self._output_host = np.empty(self.output_shape, dtype=output_dtype)
        self._context = self._engine.create_execution_context()
        if self._context is None:
            raise RuntimeError("TensorRT could not create an execution context; check available GPU memory.")
        try:
            self._input_device = self._cuda.allocate(int(np.prod(self.input_shape)) * self.input_dtype.itemsize)
            self._output_device = self._cuda.allocate(self._output_host.nbytes)
            self._cuda.check(self._cuda.lib.cudaStreamCreate(ctypes.byref(self._stream)), "create a CUDA stream")
            for name, pointer in ((self.input_name, self._input_device), (self.output_name, self._output_device)):
                if not self._context.set_tensor_address(name, pointer.value):
                    raise RuntimeError(f"TensorRT rejected the buffer address for {name}.")
            self._closed = False
        except Exception:
            self._release()
            raise

    def infer(self, tensor: np.ndarray) -> np.ndarray:
        """Return an independent host output; input contains RGB values in [0,1]."""
        with self._lock:
            if self._closed:
                raise RuntimeError("TensorRT engine is closed.")
            if tuple(tensor.shape) != self.input_shape:
                raise ValueError(f"Expected TensorRT input {self.input_shape}, got {tuple(tensor.shape)}.")
            host = np.ascontiguousarray(tensor, dtype=self.input_dtype)
            cuda = self._cuda
            cuda.check(cuda.lib.cudaSetDevice(0), "select CUDA device 0")
            # Synchronous copies deliberately support ordinary NumPy (pageable) memory.
            cuda.check(cuda.lib.cudaMemcpy(self._input_device, ctypes.c_void_p(host.ctypes.data), host.nbytes, 1), "copy input to GPU")
            if not self._context.execute_async_v3(stream_handle=self._stream.value):
                raise RuntimeError("TensorRT inference failed.")
            cuda.check(cuda.lib.cudaStreamSynchronize(self._stream), "finish inference")
            cuda.check(cuda.lib.cudaMemcpy(ctypes.c_void_p(self._output_host.ctypes.data), self._output_device, self._output_host.nbytes, 2), "copy output from GPU")
            return self._output_host.copy()

    def _release(self) -> None:
        if self._cuda is not None:
            library = self._cuda.lib
            if self._stream.value:
                library.cudaStreamSynchronize(self._stream)
            for pointer in (self._input_device, self._output_device):
                if pointer.value:
                    library.cudaFree(pointer)
                    pointer.value = None
            if self._stream.value:
                library.cudaStreamDestroy(self._stream)
                self._stream.value = None
        self._closed = True
        # Release TensorRT objects in dependency order, including context workspace.
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
