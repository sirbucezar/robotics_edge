#!/usr/bin/env bash
# Build TensorRT engines. THIS MUST RUN ON THE ROBOT.
#
#   bash ~/limo_project/training/build_engine_on_jetson.sh \
#        ~/limo_project/models/exported/person_yolov8n_640.onnx
#
# A serialized TensorRT engine is tied to the exact GPU architecture, TensorRT
# version and CUDA version that built it. An engine built anywhere else will
# refuse to deserialize on the Jetson, usually with a message that does not say
# that. There is no cross-compilation shortcut worth taking two days before an
# exam.
#
# Builds three variants so the report has a real table:
#   FP32  the reference, same maths as ONNX Runtime
#   FP16  the workhorse. Orin's tensor cores make this nearly free accuracy-wise
#         (person detection is not numerically delicate) and roughly 2x faster.
#   INT8  fastest, needs calibration images, costs a little mAP. Calibrate on
#         frames from the actual classroom -- calibrating on COCO images and
#         deploying to a 20 cm-high camera gives you the worst of both worlds.

set -euo pipefail

ONNX="${1:?usage: build_engine_on_jetson.sh <model.onnx> [calib_dir]}"
CALIB_DIR="${2:-$HOME/limo_project/data/frames}"
OUT_DIR="$(dirname "$ONNX")"
STEM="$(basename "${ONNX%.onnx}")"

TRTEXEC="${TRTEXEC:-/usr/src/tensorrt/bin/trtexec}"
if [ ! -x "$TRTEXEC" ]; then
  command -v trtexec >/dev/null 2>&1 && TRTEXEC="$(command -v trtexec)" \
    || { echo "trtexec not found. Try: ls /usr/src/tensorrt/bin/"; exit 1; }
fi

echo ">> trtexec: $TRTEXEC"
"$TRTEXEC" --version 2>/dev/null | head -3 || true

# Clocks up, or every number you measure is wrong and low.
echo ">> pinning clocks (needs sudo)"
sudo nvpmodel -m 0 || echo "   (could not set power mode)"
sudo jetson_clocks   || echo "   (could not pin clocks)"

WORKSPACE_MB="${WORKSPACE_MB:-2048}"

build () {
  local precision="$1"; shift
  local engine="${OUT_DIR}/${STEM}_${precision}.engine"
  echo
  echo "=== building ${precision} -> $(basename "$engine") ==="
  # --workspace is MiB on TRT 8.5. On TRT 10 it became --memPoolSize.
  "$TRTEXEC" \
    --onnx="$ONNX" \
    --saveEngine="$engine" \
    --workspace="${WORKSPACE_MB}" \
    --avgRuns=200 --warmUp=1000 --duration=15 \
    "$@" 2>&1 | tee "${OUT_DIR}/${STEM}_${precision}.trtexec.log"

  echo
  echo "--- ${precision} summary ---"
  grep -E "Throughput|mean =|median =|GPU Compute Time" \
    "${OUT_DIR}/${STEM}_${precision}.trtexec.log" | head -8 || true
}

build fp32
build fp16 --fp16

if [ -d "$CALIB_DIR" ] && [ "$(ls -1 "$CALIB_DIR" 2>/dev/null | wc -l)" -ge 50 ]; then
  echo
  echo ">> INT8 calibration cache from $CALIB_DIR"
  echo ">> NOTE: trtexec's built-in --int8 without a calibrator uses random"
  echo ">>       scales and produces a fast engine with meaningless accuracy."
  echo ">>       Use training/calibrate_int8.py for a real entropy calibrator,"
  echo ">>       then re-run with --calib=<cache>."
  build int8 --int8 --fp16
else
  echo
  echo ">> skipping INT8: need >= 50 images in $CALIB_DIR"
  echo ">>   run scripts/grab_frames.py first"
fi

echo
echo "=== engines ==="
ls -lh "${OUT_DIR}"/*.engine 2>/dev/null || true
cat <<EOF

Point the detector at one:
  edit ros2_ws/src/limo_project_bringup/config/mission_params.yaml
    yolo_detector.model_path: ${OUT_DIR}/${STEM}_fp16.engine

Then measure the number that goes in the report, in situ:
  python3 ~/limo_project/training/benchmark.py \\
      --engine ${OUT_DIR}/${STEM}_fp16.engine --images ~/limo_project/data/frames
EOF
