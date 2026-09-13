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
