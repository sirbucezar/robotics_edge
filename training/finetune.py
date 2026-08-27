#!/usr/bin/env python3
"""Fine-tune a small YOLO on the classroom frames. Report deliverable #1.

    # on the M2 Pro
    python3 training/finetune.py --data data/dataset/data.yaml --device mps

    # on Colab
    python3 training/finetune.py --data /content/dataset/data.yaml --device 0 --epochs 120

Design choices worth defending in the report:

*Single class.* The mission only cares about people. Dropping 79 COCO classes
removes 79 x (channels) of head parameters and, more importantly, removes the
ways the model can be wrong. It also makes INT8 calibration better behaved.

*Start from COCO weights, do not train from scratch.* 400 classroom frames is
nowhere near enough to learn what a human looks like; it is plenty to teach a
model that already knows what a human looks like what a human looks like *from
20 cm off the floor*.

*Freeze the backbone for the first stage.* With a few hundred images, letting
the whole backbone move overfits fast. Two-stage (freeze then unfreeze at a
lower LR) reliably gives a couple of mAP points here.

*Augmentation.* Heavy scale/translate jitter, and mosaic on -- the failure mode
we care about is partially visible people at the frame edges, and mosaic
manufactures exactly that. ``fliplr`` is safe. ``flipud`` is not: the camera is
never upside down and it wastes capacity.
"""

import argparse
import os
import sys


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", required=True, help="path to data.yaml")
    ap.add_argument("--model", default="yolov8n.pt",
                    help="yolov8n.pt / yolo11n.pt. Anything bigger will not hold "
                         "50 FPS on an Orin Nano once the rest of the stack is running.")
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--epochs", type=int, default=100)
    ap.add_argument("--freeze-epochs", type=int, default=25,
                    help="stage 1: backbone frozen")
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--device", default="mps", help="mps | cpu | 0 | cuda:0")
    ap.add_argument("--project", default="models/runs")
    ap.add_argument("--name", default="person_finetune")
    ap.add_argument("--patience", type=int, default=30)
    ap.add_argument("--single-stage", action="store_true")
    args = ap.parse_args()

    try:
        from ultralytics import YOLO
    except ImportError:
        sys.exit("pip install ultralytics")

    aug = dict(
        hsv_h=0.015, hsv_s=0.7, hsv_v=0.5,   # classroom lighting varies a lot
        degrees=3.0,                          # the robot tilts a little, not a lot
        translate=0.15, scale=0.6, shear=0.0,
        perspective=0.0,
        flipud=0.0, fliplr=0.5,
        mosaic=1.0, mixup=0.05,
        close_mosaic=10,                      # last 10 epochs without mosaic
    )

    common = dict(data=args.data, imgsz=args.imgsz, batch=args.batch,
                  device=args.device, project=args.project,
                  patience=args.patience, val=True, plots=True,
                  seed=0, **aug)

    model = YOLO(args.model)

    if args.single_stage:
        res = model.train(epochs=args.epochs, name=args.name, **common)
        best = os.path.join(str(res.save_dir), "weights", "best.pt")
    else:
        print("=== stage 1: backbone frozen (%d epochs) ===" % args.freeze_epochs)
        r1 = model.train(epochs=args.freeze_epochs, freeze=10, lr0=0.01,
                         name=args.name + "_s1", **common)
        stage1 = os.path.join(str(r1.save_dir), "weights", "best.pt")

        print("=== stage 2: full fine-tune at low LR (%d epochs) ==="
              % (args.epochs - args.freeze_epochs))
        model = YOLO(stage1)
        r2 = model.train(epochs=args.epochs - args.freeze_epochs, lr0=0.002,
                         name=args.name + "_s2", **common)
        best = os.path.join(str(r2.save_dir), "weights", "best.pt")

    os.makedirs("models/baseline", exist_ok=True)
    target = "models/baseline/person_%s" % os.path.basename(args.model)
    try:
        import shutil
        shutil.copy2(best, target)
    except Exception as exc:  # noqa: BLE001
        print("could not copy best weights: %s" % exc)
        target = best

    print("""
best weights: {best}
copied to:    {target}

Record these for the report's baseline row:
  mAP50, mAP50-95, precision, recall  -- printed above and in {proj}/{name}*/results.csv

Next:
  python3 training/prune.py    --weights {target}
  python3 training/export_onnx.py --weights {target} --imgsz {imgsz}
""".format(best=best, target=target, proj=args.project, name=args.name,
           imgsz=args.imgsz))


if __name__ == "__main__":
    main()
