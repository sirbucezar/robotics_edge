#!/usr/bin/env python3
"""Export .pt -> .onnx. Run this on the Mac; build the .engine on the Jetson.

    python3 training/export_onnx.py --weights models/baseline/person_yolov8n.pt --imgsz 640

Two flags matter more than the rest:

``--opset 12``
    JetPack 5.x ships TensorRT 8.5. Newer opsets emit ops its ONNX parser has
    never heard of, and the failure appears as an opaque parser error at engine
    build time on the robot, an hour after you exported. 12 is safe.

``--no-nms`` (the default)
    Do NOT bake NMS into the graph. TensorRT's handling of the NonMaxSuppression
    op is version-dependent and the plugin path is fragile. We do NMS in numpy
    in ``limo_perception/backends.py`` -- it costs well under a millisecond for
    the handful of boxes a classroom produces, and it means the exact same
    postprocess runs behind every backend, so the latency table compares like
    with like.

Also exports at a fixed batch size of 1. Dynamic shapes cost TensorRT
optimisation opportunities and we only ever infer one frame.
"""

import argparse
import os
import shutil
import sys


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--weights", required=True)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--opset", type=int, default=12)
    ap.add_argument("--simplify", action="store_true", default=True)
    ap.add_argument("--nms", action="store_true", default=False,
                    help="bake NMS into the graph (not recommended, see docstring)")
    ap.add_argument("--out-dir", default="models/exported")
    args = ap.parse_args()

    try:
        from ultralytics import YOLO
    except ImportError:
        sys.exit("pip install ultralytics onnx onnxsim")

    model = YOLO(args.weights)
    path = model.export(format="onnx", imgsz=args.imgsz, opset=args.opset,
                        simplify=args.simplify, nms=args.nms, dynamic=False,
                        half=False, device="cpu")

    os.makedirs(args.out_dir, exist_ok=True)
    stem = os.path.splitext(os.path.basename(args.weights))[0]
    target = os.path.join(args.out_dir, "%s_%d.onnx" % (stem, args.imgsz))
    shutil.copy2(str(path), target)

    try:
        import onnx
        m = onnx.load(target)
        onnx.checker.check_model(m)
        inp = m.graph.input[0]
        out = m.graph.output[0]
        def shape_of(v):
            return [d.dim_value or d.dim_param for d in v.type.tensor_type.shape.dim]
        print("\nvalidated: input %s %s -> output %s %s"
              % (inp.name, shape_of(inp), out.name, shape_of(out)))
        print("(output should be (1, 4+num_classes, num_anchors); with one class "
              "that is (1, 5, 8400) at 640)")
    except Exception as exc:  # noqa: BLE001
        print("onnx check skipped/failed: %s" % exc)

    print("""
exported -> {target}

Copy it to the robot and build the engine THERE:
  ./scripts/sync_to_robot.sh --models
  ssh agilex@<limo-ip>
  bash ~/limo_project/training/build_engine_on_jetson.sh \\
       ~/limo_project/models/exported/{name}
""".format(target=target, name=os.path.basename(target)))


if __name__ == "__main__":
    main()
