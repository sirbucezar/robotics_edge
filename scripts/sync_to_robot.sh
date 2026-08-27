#!/usr/bin/env bash
# Push the project from the Mac to the robot and (optionally) rebuild.
#
#   ./scripts/sync_to_robot.sh                 # sync only
#   ./scripts/sync_to_robot.sh --build         # sync then colcon build
#   LIMO_HOST=192.168.1.42 ./scripts/sync_to_robot.sh --build
#
# One-time setup so you stop typing 'agx' fifty times a day:
#   ssh-keygen -t ed25519          # if you have no key
#   ssh-copy-id agilex@<limo-ip>
#
# Models are synced separately and only on request, because a .engine is
# ~10-40 MB and you do not want it going over school WiFi on every save.

set -euo pipefail

LIMO_USER="${LIMO_USER:-agilex}"
# Default to the "limo" ssh alias rather than limo.local: mDNS does not
# resolve on the school network, and going through ~/.ssh/config means
# swapping robots is a one-line HostName change instead of touching every
# script. Override with LIMO_HOST=<ip> when there is no alias.
LIMO_HOST="${LIMO_HOST:-limo}"
REMOTE_DIR="${REMOTE_DIR:-/home/${LIMO_USER}/limo_project}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

BUILD=0
WITH_MODELS=0
for a in "$@"; do
  case "$a" in
    --build)  BUILD=1 ;;
    --models) WITH_MODELS=1 ;;
    *) echo "unknown flag: $a"; exit 2 ;;
  esac
done

echo ">> target: ${LIMO_USER}@${LIMO_HOST}:${REMOTE_DIR}"
ssh -o ConnectTimeout=6 "${LIMO_USER}@${LIMO_HOST}" \
  "mkdir -p '${REMOTE_DIR}/ros2_ws/src' '${REMOTE_DIR}/scripts' '${REMOTE_DIR}/training' '${REMOTE_DIR}/models/exported'" \
  || { echo "cannot reach the robot. Same WiFi? Try the raw IP: LIMO_HOST=192.168.x.y $0"; exit 1; }
# rsync 3.1.3 (Ubuntu 20.04's stock version) has no --mkpath, so it cannot
# create these nested destination dirs itself -- pre-create them above.

rsync -az --delete --info=stats1 \
  --exclude '__pycache__' --exclude '*.pyc' \
  --exclude 'build/' --exclude 'install/' --exclude 'log/' \
  "${HERE}/ros2_ws/src/" "${LIMO_USER}@${LIMO_HOST}:${REMOTE_DIR}/ros2_ws/src/"

rsync -az --info=stats1 \
  "${HERE}/scripts/" "${LIMO_USER}@${LIMO_HOST}:${REMOTE_DIR}/scripts/"

rsync -az --info=stats1 \
  "${HERE}/training/" "${LIMO_USER}@${LIMO_HOST}:${REMOTE_DIR}/training/"

if [ "$WITH_MODELS" = "1" ]; then
  echo ">> syncing models (this is the slow one)"
  rsync -azP "${HERE}/models/exported/" \
    "${LIMO_USER}@${LIMO_HOST}:${REMOTE_DIR}/models/exported/"
fi

if [ "$BUILD" = "1" ]; then
  echo ">> building on the robot"
  # 'source /opt/ros/foxy/setup.bash' explicitly: the LIMO image's interactive
  # 1/2 prompt does not run for a non-interactive ssh command.
  ssh "${LIMO_USER}@${LIMO_HOST}" bash -lc "'
    set -e
    source /opt/ros/\$(ls /opt/ros | head -1)/setup.bash
    [ -f \$HOME/limo_ros2_ws/install/setup.bash ] && source \$HOME/limo_ros2_ws/install/setup.bash
    cd ${REMOTE_DIR}/ros2_ws
    colcon build --symlink-install
  '"
fi

cat <<EOF

done.

On the robot:
  source ${REMOTE_DIR}/ros2_ws/install/setup.bash
  ros2 launch limo_project_bringup mission.launch.py detector:=mock
EOF
