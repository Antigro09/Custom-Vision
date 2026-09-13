"""Real GPU smoke test with synthetic detections, never a trained game model."""

import json

import numpy as np
import pytest

from custom_vision.objects import ObjectPipeline
from custom_vision.tensorrt_backend import TensorRTEngine, _CudaRuntime


@pytest.fixture(scope="module")
def synthetic_engine(tmp_path_factory):
    trt = pytest.importorskip("tensorrt", reason="TensorRT is only supplied on JetPack/GPU hosts")
    try:
        _CudaRuntime()
    except RuntimeError as exc:
        pytest.skip(f"CUDA device unavailable: {exc}")
    logger = trt.Logger(trt.Logger.WARNING)
    builder = trt.Builder(logger)
    builder.max_threads = 2
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    config = builder.create_builder_config()
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 32 << 20)
    image = network.add_input("images", trt.float32, (1, 3, 32, 32))
    reduced = network.add_reduce(image, trt.ReduceOperation.SUM, 15, True).get_output(0)
    reshape = network.add_shuffle(reduced)
    reshape.reshape_dims = (1, 1, 1)
    multiplier = network.add_constant((1, 1, 1), np.array([[[1e-7]]], np.float32)).get_output(0)
    offset = network.add_elementwise(reshape.get_output(0), multiplier, trt.ElementWiseOperation.PROD).get_output(0)
    detection = np.array([[[16], [16], [12], [12], [0.875]]], np.float32)
    constant = network.add_constant(detection.shape, detection).get_output(0)
    output = network.add_elementwise(constant, offset, trt.ElementWiseOperation.SUM).get_output(0)
    output.name = "output0"
    network.mark_output(output)
    plan = builder.build_serialized_network(network, config)
    assert plan is not None, "TensorRT failed to build the synthetic smoke-test network"
    path = tmp_path_factory.mktemp("synthetic-tensorrt") / "synthetic.engine"
    path.write_bytes(bytes(plan))
    return path, detection


def test_actual_cuda_inference_and_object_pipeline(synthetic_engine):
    path, expected = synthetic_engine
    with TensorRTEngine(path, num_classes=1) as engine:
        zeros = np.zeros((1, 3, 32, 32), np.float32)
        first = engine.infer(zeros)
        np.testing.assert_allclose(first, expected, rtol=0, atol=1e-6)
        # Input-dependent output proves execution/copies, rather than a stubbed constant.
        second = engine.infer(np.ones_like(zeros))
        np.testing.assert_allclose(second, expected + zeros.size * 1e-7, rtol=0, atol=1e-5)
        np.testing.assert_allclose(first, expected, rtol=0, atol=1e-6)
        with pytest.raises(ValueError, match="Expected TensorRT input"):
            engine.infer(np.zeros((1, 3, 64, 64), np.float32))
    with pytest.raises(RuntimeError, match="closed"):
        engine.infer(zeros)
    pipeline = ObjectPipeline({
        "backend": "tensorrt", "model_path": str(path),
        "labels": ["synthetic_ball"], "input_size": 32,
    })
    try:
        detections = pipeline.process(np.zeros((32, 32), np.uint8))
        assert len(detections) == 1
        assert detections[0]["bbox_xyxy"] == [10, 10, 22, 22]
        assert detections[0]["confidence"] == 0.875
        assert detections[0]["label"] == "synthetic_ball"
        json.dumps(detections, allow_nan=False)
    finally:
        pipeline.close()


def test_engine_rejects_mismatched_labels_and_input_size(synthetic_engine):
    path, _expected = synthetic_engine
    with pytest.raises(ValueError, match="2 labels"):
        TensorRTEngine(path, num_classes=2)
    with pytest.raises(ValueError, match="Configured input_size"):
        ObjectPipeline({"backend": "tensorrt", "model_path": str(path), "labels": ["ball"], "input_size": 640})


@pytest.fixture(scope="module", params=[("detect", "float32"), ("segment", "float16")])
def synthetic_yolo26_engine(tmp_path_factory, request):
    """Actual static GPU graph, with input-dependent predictions and mask prototypes."""
    trt = pytest.importorskip("tensorrt")
    try:
        _CudaRuntime()
    except RuntimeError as exc:
        pytest.skip(f"CUDA device unavailable: {exc}")
    task, precision = request.param
    dtype = np.float16 if precision == "float16" else np.float32
    trt_dtype = trt.float16 if precision == "float16" else trt.float32
    logger = trt.Logger(trt.Logger.WARNING)
    builder = trt.Builder(logger)
    builder.max_threads = 2
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    config = builder.create_builder_config()
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 32 << 20)
    if precision == "float16":
        config.set_flag(trt.BuilderFlag.FP16)
    image = network.add_input("images", trt_dtype, (1, 3, 32, 32))
    reduced = network.add_reduce(image, trt.ReduceOperation.AVG, 15, True).get_output(0)
    reshaped = network.add_shuffle(reduced)
    reshaped.reshape_dims = (1, 1, 1)
    data = [[[8, 8, 24, 24, .875, 0] + ([1, 0] if task == "segment" else [])]]
    expected = np.array(data, dtype)
    constant = network.add_constant(expected.shape, expected).get_output(0)
    coordinate_mask = np.array([[[1, 1, 1, 1, 0, 0] + ([0, 0] if task == "segment" else [])]], dtype)
    mask = network.add_constant(coordinate_mask.shape, coordinate_mask).get_output(0)
    offset = network.add_elementwise(reshaped.get_output(0), mask, trt.ElementWiseOperation.PROD).get_output(0)
    output = network.add_elementwise(constant, offset, trt.ElementWiseOperation.SUM).get_output(0)
    output.name = "arbitrary_prediction_name"
    output.dtype = trt_dtype
    if task == "segment":
        prototype = np.full((1, 2, 8, 8), -8, dtype)
        prototype[0, 0, 2:6, 2:6] = 8
        proto_output = network.add_constant(prototype.shape, prototype).get_output(0)
        proto_output.name = "arbitrary_prototypes_first"
        proto_output.dtype = trt_dtype
        network.mark_output(proto_output)  # Deliberately reverse typical exporter order.
    network.mark_output(output)
    plan = builder.build_serialized_network(network, config)
    assert plan is not None
    path = tmp_path_factory.mktemp("synthetic-yolo26") / f"{task}-{precision}.engine"
    path.write_bytes(bytes(plan))
    return path, task, dtype, expected


def test_yolo26_tensor_contracts_actual_gpu_and_mask_output(synthetic_yolo26_engine):
    path, task, dtype, expected = synthetic_yolo26_engine
    with TensorRTEngine(path, 1, output_format="yolo26_end2end", task=task) as engine:
        assert engine.input_dtype == dtype
        zeros = np.zeros(engine.input_shape, dtype)
        outputs = engine.infer(zeros)
        predictions = outputs[0] if task == "segment" else outputs
        np.testing.assert_allclose(predictions, expected, atol=1e-3)
        changed = engine.infer(np.ones(engine.input_shape, np.float32))
        changed = changed[0] if task == "segment" else changed
        np.testing.assert_allclose(changed[..., :4], expected[..., :4] + 1, atol=1e-3)
        np.testing.assert_allclose(predictions, expected, atol=1e-3)
        assert engine.last_timings["inference_ms"] > 0
    engine.close()  # Idempotent release of pinned/device memory.
    pipeline = ObjectPipeline({"backend": "tensorrt", "model_path": str(path), "labels": ["synthetic_ball"],
                               "input_size": 32, "task": task, "output_format": "yolo26_end2end"})
    try:
        result = pipeline.process(np.zeros((32, 32, 3), np.uint8))
        assert result[0]["bbox_xyxy"] == [8, 8, 24, 24]
        if task == "segment":
            assert result[0]["segmentation_status"] == "valid"
            assert result[0]["segmentation"]["centroid_px"] == pytest.approx([16, 16])
            assert result[0]["segmentation"]["bottom_px"] == pytest.approx([16, 22])
        assert pipeline.last_timings["total_ms"] > 0
        json.dumps(result, allow_nan=False)
    finally:
        pipeline.close()


def test_tensorrt_reusable_buffers_are_serialized_across_threads(synthetic_yolo26_engine):
    from concurrent.futures import ThreadPoolExecutor
    path, task, dtype, expected = synthetic_yolo26_engine
    with TensorRTEngine(path, 1, output_format="yolo26_end2end", task=task) as engine:
        def run(value):
            outputs = engine.infer(np.full(engine.input_shape, value, dtype))
            return outputs[0] if task == "segment" else outputs
        with ThreadPoolExecutor(max_workers=2) as pool:
            results = list(pool.map(run, [0, 1, 0, 1]))
        for result, value in zip(results, [0, 1, 0, 1]):
            np.testing.assert_allclose(result[..., :4], expected[..., :4] + value, atol=1e-3)
        assert engine.infer(np.zeros(engine.input_shape, dtype), decoder=lambda outputs: {"decoded": True}) == {"decoded": True}


def test_two_engine_contexts_use_independent_buffers_and_streams(synthetic_yolo26_engine):
    from concurrent.futures import ThreadPoolExecutor
    path, task, dtype, expected = synthetic_yolo26_engine
    with TensorRTEngine(path, 1, output_format="yolo26_end2end", task=task) as first, \
            TensorRTEngine(path, 1, output_format="yolo26_end2end", task=task) as second:
        assert first._stream.value != second._stream.value
        assert first._device[first.input_name].value != second._device[second.input_name].value
        with ThreadPoolExecutor(max_workers=2) as pool:
            futures = [pool.submit(engine.infer, np.full(engine.input_shape, value, dtype))
                       for engine, value in ((first, 0), (second, 1))]
            results = [future.result() for future in futures]
        for result, value in zip(results, [0, 1]):
            prediction = result[0] if task == "segment" else result
            np.testing.assert_allclose(prediction[..., :4], expected[..., :4] + value, atol=1e-3)
