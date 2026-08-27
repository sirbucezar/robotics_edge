#!/usr/bin/env bash
# Record the classroom. Run this ON THE ROBOT.
#
#   bash ~/limo_project/scripts/record_bag.sh classroom_run1
#
# This is the single highest-value thing you can do with robot time on day 1,
# and it pays off twice:
#
#   1. It is your training set. Frames pulled out of this bag are images of
#      real people, at the real camera height, under the real classroom
#      lighting. Fine-tuning on 300 of these will beat fine-tuning on 3000
#      images of pedestrians shot from eye level, because the domain gap on a
#      20 cm-high camera is enormous -- you mostly see legs, torsos and chair
#      backs, which is not what COCO's 'person' class looks like.
#
#   2. It is your regression test. 'ros2 bag play' lets you re-run the entire
#      perception + counting stack, deterministically, at 3 a.m. on day 3
#      without a robot, without people, and without a battery.
#
# Drive it slowly with the remote while people sit, stand, walk and partly
# occlude each other. Two or three minutes is plenty; 20 GB of bag is not.

set -euo pipefail

NAME="${1:-classroom_$(date +%Y%m%d_%H%M%S)}"
OUT_DIR="${OUT_DIR:-$HOME/limo_project/data/bags}"
mkdir -p "$OUT_DIR"

TOPICS=(
  /camera/color/image_raw
  /camera/color/camera_info
  /camera/depth/image_raw
  /camera/depth/camera_info
  /scan
  /odom
  /imu
  /tf
  /tf_static
  /limo_status
)

echo ">> recording to ${OUT_DIR}/${NAME}"
echo ">> topics: ${TOPICS[*]}"
echo ">> Ctrl-C to stop"
echo
df -BG --output=avail "$OUT_DIR" | tail -1 | xargs echo ">> free space:"

cd "$OUT_DIR"
# --compression-mode file keeps the colour stream from eating the NVMe.
ros2 bag record -o "$NAME" \
  --compression-mode file --compression-format zstd \
  "${TOPICS[@]}"

echo
echo ">> done. Size:"
du -sh "${OUT_DIR}/${NAME}"
cat <<EOF

Next:
  # pull frames out for labelling (run on the robot, in another terminal,
  # while the bag plays):
  ros2 bag play ${OUT_DIR}/${NAME} --rate 0.5
  python3 ~/limo_project/scripts/grab_frames.py --every 5 --out ~/limo_project/data/frames

  # then copy them to the Mac:
  rsync -azP agilex@<limo-ip>:~/limo_project/data/frames/ ./data/frames/
EOF
