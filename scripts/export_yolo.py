#!/usr/bin/env python3
"""Export a trusted local YOLO26 checkpoint to static ONNX and optional FP16 TensorRT."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
CACHE = ROOT / '.cache' / 'ultralytics'
CACHE.mkdir(parents=True, exist_ok=True)
os.environ.setdefault('YOLO_CONFIG_DIR', str(CACHE))
os.environ['YOLO_AUTOINSTALL'] = 'false'


def validate_export_shapes(inputs, outputs, task, input_size):
    """Check the static runtime contract before building an engine or manifest.

    This helper only consumes Python shapes; validating an exported contract does
    not require importing ONNX, Torch, Ultralytics, or a CUDA runtime.
    """
    if task not in ('detect', 'segment'):
        raise ValueError('Only detect and segment exports are supported')
    expected_input = [1, 3, input_size[1], input_size[0]]
    if len(inputs) != 1 or list(next(iter(inputs.values()))) != expected_input:
        raise ValueError(f'Expected one static RGB input {expected_input}; got {inputs}')
    expected_count = 2 if task == 'segment' else 1
    if len(outputs) != expected_count:
        raise ValueError(f'Expected {expected_count} {task} output tensor(s); got {outputs}')
    if any(not shape or any(type(dimension) is not int or dimension <= 0 for dimension in shape)
           for shape in outputs.values()):
        raise ValueError(f'Output dimensions must be positive static integers; got {outputs}')
    boxes = [shape for shape in outputs.values() if len(shape) == 3 and shape[0] == 1]
    prototypes = [shape for shape in outputs.values() if len(shape) == 4 and shape[0] == 1]
    if len(boxes) != 1 or len(prototypes) != (1 if task == 'segment' else 0):
        raise ValueError(f'Expected one rank-3 prediction and rank-4 prototypes only for segment; got {outputs}')
    mask_channels = prototypes[0][1] if prototypes else 0
    if prototypes:
        shape = prototypes[0]
        if not 1 <= mask_channels <= 128 or mask_channels * shape[2] * shape[3] > 128 * 320 * 320:
            raise ValueError(f'Mask prototype dimensions exceed runtime limits; got {shape}')
    if not 1 <= boxes[0][1] <= 300 or boxes[0][2] != 6 + mask_channels:
        raise ValueError(f'Expected YOLO26 end-to-end [1,N,{6 + mask_channels}], N in1..300; got {outputs}')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--weights',type=Path,required=True,help='Trusted local .pt checkpoint; no implicit model download')
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--width',type=int,default=640)
    p.add_argument('--height',type=int,default=640)
    p.add_argument('--max-detections',type=int,default=64)
    p.add_argument('--build-engine',action='store_true')
    args=p.parse_args()
    if not args.weights.is_file(): p.error('weights must be an existing trusted checkpoint')
    if any(v < 32 or v > 1280 or v % 32 for v in (args.width,args.height)): p.error('dimensions must be multiples of32 in32..1280')
    if not 1<=args.max_detections<=300: p.error('max-detections must be in1..300')
    import ultralytics
    from ultralytics import YOLO
    import torch
    if ultralytics.__version__!='8.4.150': p.error('Use scripts/setup_yolo_tools.sh and .venv-export/bin/python (pinned8.4.150)')
    torch.set_num_threads(2)
    output=args.output_dir.resolve()
    output.mkdir(parents=True,exist_ok=True)
    checkpoint=output/args.weights.name
    if checkpoint != args.weights.resolve(): shutil.copy2(args.weights,checkpoint)
    model=YOLO(str(checkpoint))
    if model.task not in ('detect','segment'): p.error('Only object detection and instance segmentation are supported')
    model_name=type(model.model.model[-1]).__name__
    # Explicit NMS-free one-to-one head, FP32 I/O, fixed batch1; TensorRT selects
    # FP16 internal kernels later. No package auto-installs or system changes.
    exported=Path(model.export(format='onnx',imgsz=[args.height,args.width],batch=1,
                              dynamic=False,simplify=False,opset=17,nms=False,
                              half=False,device='cpu',max_det=args.max_detections))
    import onnx
    graph=onnx.load(str(exported));onnx.checker.check_model(graph)
    inputs={node.name:[dim.dim_value for dim in node.type.tensor_type.shape.dim] for node in graph.graph.input}
    outputs={node.name:[dim.dim_value for dim in node.type.tensor_type.shape.dim] for node in graph.graph.output}
    validate_export_shapes(inputs,outputs,model.task,[args.width,args.height])
    labels=[model.names[index] for index in range(len(model.names))]
    metadata={'ultralytics_version':ultralytics.__version__,'task':model.task,'output_format':'yolo26_end2end',
              'input_size':[args.width,args.height],'labels':labels,'head_type':model_name,'outputs':outputs,
              'weights_sha256':hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
              'onnx_sha256':hashlib.sha256(exported.read_bytes()).hexdigest(),
              'note':'Model identity and class order only; this manifest does not certify team game-piece accuracy.'}
    if args.build_engine:
        engine=exported.with_suffix('.engine')
        subprocess.run([str(ROOT/'scripts'/'build_engine.sh'),str(exported),str(engine)],check=True)
        metadata['engine_sha256']=hashlib.sha256(engine.read_bytes()).hexdigest()
        metadata['model_path']=str(engine)
    else: metadata['model_path']=str(exported)
    path=exported.with_suffix('.json');path.write_text(json.dumps(metadata,indent=2)+'\n')
    print(json.dumps({'manifest':str(path),'model_path':metadata['model_path'],'outputs':outputs},indent=2))


if __name__=='__main__': main()
