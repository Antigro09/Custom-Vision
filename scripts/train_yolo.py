#!/usr/bin/env python3
"""Fine-tune YOLO26 on the team's supplied dataset; never runs as part of setup."""
import argparse
from pathlib import Path
import os


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data',type=Path,required=True,help='Team dataset YAML with train/val splits and class order')
    p.add_argument('--weights',type=Path,required=True,help='Trusted local detection or segmentation checkpoint')
    p.add_argument('--epochs',type=int,default=100)
    p.add_argument('--batch',type=int,default=4)
    p.add_argument('--imgsz',type=int,default=640)
    p.add_argument('--device',default='0')
    p.add_argument('--name',default='gamepiece')
    a=p.parse_args()
    if not a.data.is_file() or not a.weights.is_file(): p.error('Provide existing dataset YAML and trusted weights')
    if min(a.epochs,a.batch,a.imgsz)<=0: p.error('epochs, batch and imgsz must be positive')
    root=Path(__file__).resolve().parents[1]
    cache=root/'.cache'/'ultralytics';cache.mkdir(parents=True,exist_ok=True)
    os.environ['YOLO_CONFIG_DIR']=str(cache);os.environ['YOLO_AUTOINSTALL']='false'
    import ultralytics
    from ultralytics import YOLO
    if ultralytics.__version__!='8.4.150': p.error('Run in the pinned .venv-export environment')
    model=YOLO(str(a.weights.resolve()))
    if model.task not in ('detect','segment'): p.error('Only detect/segment training is supported')
    model.train(data=str(a.data.resolve()),epochs=a.epochs,batch=a.batch,imgsz=a.imgsz,
                device=a.device,project=str(root/'data'/'training'),name=a.name,
                workers=2,seed=1086,exist_ok=False)


if __name__=='__main__': main()
