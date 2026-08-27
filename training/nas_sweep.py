#!/usr/bin/env python3
"""Hardware-aware architecture sweep. Report deliverable #2b ("NAS results").

    # latency sweep on the robot (fast, no training)
    python3 training/nas_sweep.py --images ~/limo_project/data/frames \
        --variants yolov8n,yolo11n,yolov8s --sizes 320,416,512,640 --latency-only

    # with accuracy, using weights you already fine-tuned
    python3 training/nas_sweep.py --images data/frames --data data/dataset/data.yaml \
        --weights-dir models/baseline --variants yolov8n,yolo11n --sizes 416,640

**Be honest about what this is.** Running a real neural architecture search --
a supernet, an evolutionary controller, thousands of GPU-hours -- is not
something you do in three days on an M2. The rubric says "neural architecture
search results", and the defensible reading is: *use* architecture-search
results rather than *run* a search. That means two things, and the report should
say both plainly:

1. **Consuming published NAS.** YOLOv8-n/s/m and YOLO11-n/s are not
   hand-drawn; their width and depth multipliers came out of the authors'
   architecture search, and YOLO-NAS (Deci) is explicitly the output of a
   hardware-aware NAS with quantization-friendly blocks baked in. Picking
   yolov8n over yolov8s *is* acting on a search result.

2. **Running a constrained hardware-aware search yourself.** The deployable
   design space that actually matters here is small and enumerable:
   {backbone variant} x {input resolution} x {precision}. This script
   enumerates it, measures real latency on the real Orin Nano, pairs it with
   mAP, and reports the Pareto front. That is a legitimate -- if coarse --
   hardware-in-the-loop architecture search, and it produces exactly the
   artefact the report needs: a plot with latency on one axis and accuracy on
   the other, with your chosen operating point circled and justified.

Input resolution is usually the strongest lever in that space, and it is the
one people forget: dropping 640 -> 416 cuts MACs by 2.4x, which beats what
pruning and INT8 give you combined. It also hurts small/distant people the
most, which for a robot that only cares about people within about 4 m may be
a cost you are happy to pay -- and being able to say *that*, with numbers, is
the "optimization justification" the rubric is asking for.
"""

import argparse
import csv
import json
import os
import sys


def pareto_front(rows, x_key="inference_fps", y_key="mAP50"):
    """Points not dominated on both axes (higher is better on both)."""
    front = []
    for r in rows:
        dominated = any(
            (o[x_key] >= r[x_key] and o[y_key] >= r[y_key]) and
            (o[x_key] > r[x_key] or o[y_key] > r[y_key])
            for o in rows if o is not r)
        if not dominated:
            front.append(r)
    return sorted(front, key=lambda r: r[x_key])


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--images", required=True, help="frames to time against")
    ap.add_argument("--variants", default="yolov8n,yolo11n",
                    help="comma separated model stems")
    ap.add_argument("--sizes", default="320,416,512,640")
    ap.add_argument("--weights-dir", default="models/baseline")
    ap.add_argument("--data", default="", help="data.yaml; omit for --latency-only")
    ap.add_argument("--latency-only", action="store_true")
    ap.add_argument("--device", default="cuda:0")
    ap.add_argument("--warmup", type=int, default=20)
    ap.add_argument("--iters", type=int, default=150)
    ap.add_argument("--out", default="models/benchmarks/nas_sweep")
    args = ap.parse_args()

    try:
        import cv2  # noqa: F401
        from ultralytics import YOLO
    except ImportError as exc:
        sys.exit("missing dependency: %s" % exc)

    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                    "ros2_ws", "src", "limo_perception"))
    from limo_perception.backends import build_backend

    import glob
    import statistics
    import time

    paths = sorted(glob.glob(os.path.join(args.images, "*.jpg")))[:60]
    if not paths:
        sys.exit("no images in %s" % args.images)
    images = [cv2.imread(p) for p in paths]
    images = [i for i in images if i is not None]

    variants = [v.strip() for v in args.variants.split(",") if v.strip()]
    sizes = [int(s) for s in args.sizes.split(",")]

    rows = []
    for variant in variants:
        weights = os.path.join(args.weights_dir, "person_%s.pt" % variant)
        if not os.path.exists(weights):
            weights = "%s.pt" % variant  # fall back to the COCO checkpoint
            print("!! %s: no fine-tuned weights, using stock %s "
                  "(mAP will not reflect your classroom)" % (variant, weights))

        for size in sizes:
            print("\n=== %s @ %d ===" % (variant, size))
            row = {"variant": variant, "imgsz": size}

            try:
                backend = build_backend("ultralytics", weights, imgsz=size,
                                        conf=0.35, iou=0.45, classes=(0,),
                                        device=args.device)
                for i in range(args.warmup):
                    backend.infer(images[i % len(images)])
                inf = []
                for i in range(args.iters):
                    _, t = backend.infer(images[i % len(images)])
                    inf.append(t["inference_ms"])
                row["inference_ms"] = round(statistics.mean(inf), 2)
                row["inference_fps"] = round(1000.0 / statistics.mean(inf), 1)
                print("  %.2f ms -> %.0f FPS" % (row["inference_ms"], row["inference_fps"]))
            except Exception as exc:  # noqa: BLE001
                print("  latency failed: %s" % exc)
                continue

            if not args.latency_only and args.data:
                try:
                    m = YOLO(weights)
                    metrics = m.val(data=args.data, imgsz=size, device=args.device,
                                    verbose=False, plots=False)
                    row["mAP50"] = round(float(metrics.box.map50), 4)
                    row["mAP50_95"] = round(float(metrics.box.map), 4)
                    print("  mAP50 %.3f  mAP50-95 %.3f" % (row["mAP50"], row["mAP50_95"]))
                except Exception as exc:  # noqa: BLE001
                    print("  val failed: %s" % exc)
                    row["mAP50"] = row["mAP50_95"] = 0.0
            else:
                row["mAP50"] = row["mAP50_95"] = 0.0

            rows.append(row)

    if not rows:
        sys.exit("nothing measured")

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    with open(args.out + ".csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    front = pareto_front(rows) if not args.latency_only else []
    with open(args.out + ".json", "w") as fh:
        json.dump({"all": rows, "pareto": front}, fh, indent=2)

    print("\n=== sweep ===")
    print("| variant | input | inference ms | FPS | mAP50 | on Pareto front |")
    print("|---|---|---|---|---|---|")
    for r in sorted(rows, key=lambda r: (r["variant"], r["imgsz"])):
        print("| %s | %d | %.2f | %.0f | %.3f | %s |"
              % (r["variant"], r["imgsz"], r["inference_ms"], r["inference_fps"],
                 r["mAP50"], "yes" if r in front else ""))

    over50 = [r for r in rows if r["inference_fps"] >= 50]
    if over50:
        best = max(over50, key=lambda r: r["mAP50"])
        print("\nMost accurate configuration that still clears 50 FPS: "
              "%s @ %d (%.0f FPS, mAP50 %.3f)"
              % (best["variant"], best["imgsz"], best["inference_fps"], best["mAP50"]))
    else:
        print("\nNothing cleared 50 FPS in torch. This is expected -- the "
              "ultralytics/torch path is the baseline, not the deployment path. "
              "Re-run the winners through export_onnx.py + "
              "build_engine_on_jetson.sh and measure with benchmark.py.")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(7, 5))
        for variant in variants:
            pts = [r for r in rows if r["variant"] == variant]
            if not pts:
                continue
            ax.plot([p["inference_fps"] for p in pts], [p["mAP50"] for p in pts],
                    "o-", label=variant)
            for p in pts:
                ax.annotate(str(p["imgsz"]), (p["inference_fps"], p["mAP50"]),
                            textcoords="offset points", xytext=(5, 4), fontsize=8)
        ax.axvline(50, ls="--", c="crimson", lw=1)
        ax.text(51, ax.get_ylim()[0], " 50 FPS requirement", color="crimson",
                fontsize=8, va="bottom")
        ax.set_xlabel("inference FPS (Orin Nano)")
        ax.set_ylabel("mAP@50")
        ax.set_title("Accuracy vs latency across the deployable design space")
        ax.grid(alpha=.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig(args.out + ".png", dpi=150)
        print("\nplot -> %s.png  (put this figure in the report)" % args.out)
    except Exception as exc:  # noqa: BLE001
        print("plot skipped: %s" % exc)


if __name__ == "__main__":
    main()
