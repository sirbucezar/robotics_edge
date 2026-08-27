#!/usr/bin/env python3
"""Measure every variant the same way, and emit the report's table. Deliverable #2c.

Run it ON THE ROBOT, because the number the rubric cares about is the one the
robot achieves:

    python3 training/benchmark.py \
        --images ~/limo_project/data/frames \
        --model ultralytics:models/baseline/person_yolov8n.pt \
        --model onnxruntime:models/exported/person_yolov8n_640.onnx \
        --model tensorrt:models/exported/person_yolov8n_640_fp16.engine \
        --model tensorrt:models/exported/person_yolov8n_640_int8.engine \
        --out models/benchmarks/results

Emits ``results.csv`` and ``results.md``. Paste the markdown straight into the
report.

Three things this does that a naive timing loop does not, and each of them is
worth saying out loud in the report:

*Warmup.* The first 20 inferences on a Jetson are not representative: CUDA
context creation, kernel autotuning and clock ramp all land there. Discarded.

*Percentiles, not just the mean.* A control loop lives or dies on p95. A model
averaging 8 ms with a 40 ms tail will make the robot lurch, and the mean hides
that completely.

*Separates inference from pipeline.* ``inference_ms`` is the forward pass --
this is what ">= 50 FPS" means and what you compare across precisions.
``e2e_ms`` adds letterboxing, the host-to-device copy and NMS. The gap between
them is often the more interesting engineering finding: once TensorRT gets the
forward pass under 5 ms, preprocessing on the CPU becomes the bottleneck, and
no amount of further quantization helps.
"""

import argparse
import csv
import glob
import os
import statistics
import sys
import time


def load_images(folder, limit):
    import cv2
    paths = sorted(glob.glob(os.path.join(folder, "*.jpg")) +
                   glob.glob(os.path.join(folder, "*.png")))
    if not paths:
        sys.exit("no images in %s -- run scripts/grab_frames.py first" % folder)
    paths = paths[:limit] if limit else paths
    imgs = [cv2.imread(p) for p in paths]
    imgs = [i for i in imgs if i is not None]
    print("loaded %d images (%dx%d)" % (len(imgs), imgs[0].shape[1], imgs[0].shape[0]))
    return imgs


def bench_one(spec, images, imgsz, conf, iou, warmup, iters):
    backend_name, model_path = spec.split(":", 1)
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                    "ros2_ws", "src", "limo_perception"))
    from limo_perception.backends import build_backend

    kwargs = dict(imgsz=imgsz, conf=conf, iou=iou, classes=(0,))
    if backend_name == "ultralytics":
        kwargs["device"] = os.environ.get("BENCH_DEVICE", "cuda:0")

    print("\n=== %s : %s ===" % (backend_name, os.path.basename(model_path)))
    t_load = time.perf_counter()
    backend = build_backend(backend_name, model_path, **kwargs)
    load_s = time.perf_counter() - t_load
    print("  loaded in %.2f s" % load_s)

    for i in range(warmup):
        backend.infer(images[i % len(images)])

    inf, pre, post, e2e, ndet = [], [], [], [], []
    for i in range(iters):
        img = images[i % len(images)]
        t0 = time.perf_counter()
        boxes, timings = backend.infer(img)
        e2e.append((time.perf_counter() - t0) * 1000.0)
        inf.append(timings.get("inference_ms", 0.0))
        pre.append(timings.get("preprocess_ms", 0.0))
        post.append(timings.get("postprocess_ms", 0.0))
        ndet.append(len(boxes))
        if (i + 1) % 100 == 0:
            print("  %d/%d" % (i + 1, iters))

    def pct(xs, p):
        xs = sorted(xs)
        k = min(len(xs) - 1, int(round(p / 100.0 * (len(xs) - 1))))
        return xs[k]

    row = {
        "backend": backend_name,
        "model": os.path.basename(model_path),
        "size_mb": round(os.path.getsize(model_path) / 1e6, 2),
        "imgsz": imgsz,
        "load_s": round(load_s, 2),
        "pre_ms": round(statistics.mean(pre), 2),
        "inf_mean_ms": round(statistics.mean(inf), 2),
        "inf_p50_ms": round(pct(inf, 50), 2),
        "inf_p95_ms": round(pct(inf, 95), 2),
        "post_ms": round(statistics.mean(post), 2),
        "e2e_mean_ms": round(statistics.mean(e2e), 2),
        "e2e_p95_ms": round(pct(e2e, 95), 2),
        "inference_fps": round(1000.0 / max(1e-6, statistics.mean(inf)), 1),
        "e2e_fps": round(1000.0 / max(1e-6, statistics.mean(e2e)), 1),
        "mean_detections": round(statistics.mean(ndet), 2),
    }
    print("  inference %.2f ms (p95 %.2f) -> %.0f FPS | e2e %.2f ms -> %.0f FPS"
          % (row["inf_mean_ms"], row["inf_p95_ms"], row["inference_fps"],
             row["e2e_mean_ms"], row["e2e_fps"]))
    if row["inference_fps"] < 50:
        print("  ** below the 50 FPS bar. Try: --imgsz 512 or 416, INT8, or "
              "check sudo jetson_clocks is applied. **")
    return row


MD_COLUMNS = [
    ("model", "Model"),
    ("backend", "Backend"),
    ("imgsz", "Input"),
    ("size_mb", "Size (MB)"),
    ("inf_mean_ms", "Inference (ms)"),
    ("inf_p95_ms", "p95 (ms)"),
    ("inference_fps", "Inference FPS"),
    ("e2e_mean_ms", "End-to-end (ms)"),
    ("e2e_fps", "E2E FPS"),
    ("mean_detections", "Avg det/frame"),
]


def write_markdown(rows, path):
    lines = ["| " + " | ".join(h for _, h in MD_COLUMNS) + " |",
             "|" + "|".join(["---"] * len(MD_COLUMNS)) + "|"]
    for r in rows:
        lines.append("| " + " | ".join(str(r[k]) for k, _ in MD_COLUMNS) + " |")
    lines.append("")
    lines.append("_Measured on Jetson Orin Nano 8GB, clocks pinned "
                 "(`nvpmodel -m 0`, `jetson_clocks`). Inference = forward pass "
                 "only; end-to-end adds letterbox, HtoD copy and NMS._")
    with open(path, "w") as fh:
        fh.write("\n".join(lines) + "\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--images", required=True)
    ap.add_argument("--model", action="append", required=True, dest="models",
                    metavar="BACKEND:PATH",
                    help="repeatable, e.g. tensorrt:models/exported/x_fp16.engine")
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--iou", type=float, default=0.45)
    ap.add_argument("--warmup", type=int, default=25)
    ap.add_argument("--iters", type=int, default=300)
    ap.add_argument("--limit-images", type=int, default=100)
    ap.add_argument("--out", default="models/benchmarks/results")
    args = ap.parse_args()

    images = load_images(args.images, args.limit_images)

    rows = []
    for spec in args.models:
        try:
            rows.append(bench_one(spec, images, args.imgsz, args.conf, args.iou,
                                  args.warmup, args.iters))
        except Exception as exc:  # noqa: BLE001
            print("  FAILED: %s" % exc)

    if not rows:
        sys.exit("nothing benchmarked")

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    with open(args.out + ".csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    write_markdown(rows, args.out + ".md")

    print("\nwrote %s.csv and %s.md" % (args.out, args.out))
    print(open(args.out + ".md").read())


if __name__ == "__main__":
    main()
