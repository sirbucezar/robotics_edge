#!/usr/bin/env python3
"""Structured channel pruning + recovery fine-tune. Report deliverable #2a.

    python3 training/prune.py --weights models/baseline/person_yolov8n.pt \
        --data data/dataset/data.yaml --ratio 0.3 --recover-epochs 30

Read this before you run it, because the honest version of this section is
worth more marks than a big number:

**Unstructured pruning does nothing for your FPS.** Zeroing individual weights
gives you a sparse tensor that TensorRT will happily run at exactly the same
speed as a dense one, because the GPU still does the same multiply-accumulates.
It shrinks the *file*, not the latency. If you report "50% pruned, same speed",
that is the correct result and you should say why.

**Structured pruning removes whole channels**, so the convolution really does
get smaller and really does get faster. That is what this script does, via
``torch-pruning``, which handles the dependency graph (prune a channel in one
conv and you must prune the matching input channel in the next, plus the
BatchNorm, plus anything a residual connection couples it to).

**Expect modest wins.** YOLOv8n is already a heavily architecture-searched,
width-scaled model -- it is not a fat ResNet with obvious slack. A 30% channel
prune typically buys 15-25% latency and costs 1-3 mAP, recoverable to about
1 point with a short fine-tune. Report that honestly with the numbers; the
rubric asks for "accuracy vs. speed trade-offs with metrics", not for the
trade-off to be free.

    pip install torch-pruning
"""

import argparse
import os
import sys


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--weights", required=True)
    ap.add_argument("--data", required=True)
    ap.add_argument("--ratio", type=float, default=0.30,
                    help="fraction of channels to remove")
    ap.add_argument("--iterative-steps", type=int, default=3,
                    help="prune gradually; one big cut damages the network much "
                         "more than three small ones with a few steps between")
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--recover-epochs", type=int, default=30)
    ap.add_argument("--device", default="mps")
    ap.add_argument("--out", default="models/baseline/person_yolov8n_pruned.pt")
    args = ap.parse_args()

    try:
        import torch
        import torch_pruning as tp
        from ultralytics import YOLO
    except ImportError as exc:
        sys.exit("missing dependency (%s). pip install torch-pruning ultralytics" % exc)

    yolo = YOLO(args.weights)
    model = yolo.model
    model.eval()

    example = torch.zeros(1, 3, args.imgsz, args.imgsz)
    base_macs, base_params = tp.utils.count_ops_and_params(model, example)
    print("before: %.2f GMACs, %.2f M params"
          % (base_macs / 1e9, base_params / 1e6))

    # The detection head must not be pruned: its output channel count encodes
    # (4 box regression bins x reg_max) + num_classes. Change it and the decode
    # in limo_perception/backends.py silently produces garbage boxes.
    ignored = []
    for m in model.modules():
        if m.__class__.__name__ in ("Detect", "v8DetectionLoss", "DFL"):
            ignored.append(m)

    pruner = tp.pruner.MagnitudePruner(
        model,
        example,
        importance=tp.importance.MagnitudeImportance(p=2),
        iterative_steps=args.iterative_steps,
        pruning_ratio=args.ratio,
        ignored_layers=ignored,
        global_pruning=True,
    )

    for step in range(args.iterative_steps):
        pruner.step()
        macs, params = tp.utils.count_ops_and_params(model, example)
        print("  step %d/%d: %.2f GMACs (%.0f%%), %.2f M params (%.0f%%)"
              % (step + 1, args.iterative_steps, macs / 1e9, 100 * macs / base_macs,
                 params / 1e6, 100 * params / base_params))

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    yolo.model = model
    # ultralytics needs these cleared or it will try to resume the old training
    # state against the new, differently-shaped graph.
    yolo.ckpt = None
    yolo.trainer = None
    yolo.save(args.out)
    print("pruned weights -> %s" % args.out)

    if args.recover_epochs > 0:
        print("\n=== recovery fine-tune (%d epochs) ===" % args.recover_epochs)
        recovered = YOLO(args.out)
        r = recovered.train(data=args.data, epochs=args.recover_epochs,
                            imgsz=args.imgsz, device=args.device, lr0=0.001,
                            project="models/runs", name="person_pruned_recover",
                            mosaic=1.0, close_mosaic=5, seed=0, plots=True)
        best = os.path.join(str(r.save_dir), "weights", "best.pt")
        import shutil
        shutil.copy2(best, args.out)
        print("recovered weights -> %s" % args.out)

    macs, params = tp.utils.count_ops_and_params(model, example)
    print("""
=== report row ===
  pruning ratio        {ratio:.0%}
  GMACs                {before_m:.2f} -> {after_m:.2f}  ({dm:.0%} of baseline)
  params (M)           {before_p:.2f} -> {after_p:.2f}  ({dp:.0%} of baseline)
  mAP                  read it off the val output above and compare to the baseline run
  latency              measure it for real: training/benchmark.py, on the Jetson

Next:
  python3 training/export_onnx.py --weights {out} --imgsz {imgsz}
""".format(ratio=args.ratio, before_m=base_macs / 1e9, after_m=macs / 1e9,
           dm=macs / base_macs, before_p=base_params / 1e6, after_p=params / 1e6,
           dp=params / base_params, out=args.out, imgsz=args.imgsz))


if __name__ == "__main__":
    main()
