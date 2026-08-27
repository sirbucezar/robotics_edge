#!/usr/bin/env bash
# Run this ON THE ROBOT, first thing, every session.
#
#   ssh agilex@<limo-ip>       # password: agx
#   bash ~/limo_project/scripts/robot_check.sh
#
# It answers, in about twenty seconds, the only question that matters at the
# start of a working session: is this robot in a state where the next thing I
# try can possibly work? Every line is something that has silently broken a
# LIMO demo before.

set -uo pipefail

BOLD=$'\033[1m'; DIM=$'\033[2m'; RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; OFF=$'\033[0m'
pass(){ printf "  ${GRN}ok${OFF}    %s\n" "$1"; }
warn(){ printf "  ${YEL}warn${OFF}  %s\n" "$1"; }
fail(){ printf "  ${RED}FAIL${OFF}  %s\n" "$1"; }
head(){ printf "\n${BOLD}%s${OFF}\n" "$1"; }

head "Machine"
printf "  ${DIM}%s${OFF}\n" "$(uname -a)"
[ -f /etc/nv_tegra_release ] && printf "  ${DIM}%s${OFF}\n" "$(head -1 /etc/nv_tegra_release)"
command -v jetson_release >/dev/null 2>&1 && jetson_release -s 2>/dev/null | sed 's/^/  /'

# Disk: the Jetson has a 128 GB NVMe and rosbags fill it fast.
avail=$(df -BG --output=avail / | tail -1 | tr -dc '0-9')
if   [ "${avail:-0}" -lt 5 ];  then fail "only ${avail}G free on / -- delete old bags before recording"
elif [ "${avail:-0}" -lt 15 ]; then warn "${avail}G free on /"
else pass "${avail}G free on /"; fi

# Power mode: a Jetson in 7 W mode will miss the 50 FPS target and you will
# spend an hour blaming TensorRT.
if command -v nvpmodel >/dev/null 2>&1; then
  mode=$(sudo -n nvpmodel -q 2>/dev/null | tail -1)
  [ -n "$mode" ] && printf "  ${DIM}power mode: %s${OFF}\n" "$mode"
  warn "if FPS is low: sudo nvpmodel -m 0 && sudo jetson_clocks"
fi

head "ROS environment"
if [ -z "${ROS_DISTRO:-}" ]; then
  fail "ROS_DISTRO unset. The LIMO image asks you to type 1 (ros1) or 2 (ros2) in every new shell -- type 2."
else
  pass "ROS_DISTRO=$ROS_DISTRO"
fi
printf "  ${DIM}ROS_DOMAIN_ID=%s  RMW=%s${OFF}\n" "${ROS_DOMAIN_ID:-0}" "${RMW_IMPLEMENTATION:-default}"

if [ -f "$HOME/limo_ros2_ws/install/setup.bash" ]; then
  pass "AgileX workspace present at ~/limo_ros2_ws"
else
  warn "no ~/limo_ros2_ws/install/setup.bash -- AgileX stack not sourced?"
fi
if [ -f "$HOME/limo_project/ros2_ws/install/setup.bash" ]; then
  pass "project workspace built at ~/limo_project/ros2_ws"
else
  warn "project workspace not built yet: cd ~/limo_project/ros2_ws && colcon build --symlink-install"
fi

head "Serial / chassis"
for dev in /dev/ttyTHS1 /dev/ttyUSB0 /dev/ttyACM0 /dev/agx_limo; do
  [ -e "$dev" ] && pass "$dev present"
done
groups | grep -q dialout && pass "user in dialout group" || warn "user not in dialout -- chassis may not open"

head "USB devices"
lsusb 2>/dev/null | grep -iE "orbbec|astra|2bc5|ydlidar|cp210|silicon" | sed 's/^/  /' || warn "no camera/lidar-looking USB device found"

head "Topics"
if ! command -v ros2 >/dev/null 2>&1; then
  fail "ros2 not on PATH -- source the workspace first"
  exit 1
fi
topics=$(timeout 10 ros2 topic list 2>/dev/null)
if [ -z "$topics" ]; then
  fail "no topics at all. Nothing is running. Start: ros2 launch limo_bringup limo_start.launch.py"
else
  for t in /scan /odom /imu /tf /limo_status; do
    echo "$topics" | grep -qx "$t" && pass "$t" || fail "$t missing"
  done
  for t in /camera/color/image_raw /camera/depth/image_raw /camera/color/camera_info; do
    echo "$topics" | grep -qx "$t" && pass "$t" || warn "$t missing (camera driver not started?)"
  done
  echo "$topics" | grep -qx "/map" && pass "/map (nav2 up)" || warn "/map missing (nav2 not started)"
fi

head "Rates (3 s samples)"
rate(){
  local t="$1" min="$2"
  local hz
  hz=$(timeout 6 ros2 topic hz "$t" --window 20 2>/dev/null | grep -m1 "average rate" | awk '{print $3}')
  if [ -z "$hz" ]; then warn "$t: no messages"; return; fi
  awk -v h="$hz" -v m="$min" -v t="$t" \
    'BEGIN{ if (h+0 < m+0) printf "  \033[33mwarn\033[0m  %s: %.1f Hz (expected >= %s)\n", t, h, m;
            else            printf "  \033[32mok\033[0m    %s: %.1f Hz\n", t, h }'
}
rate /scan 4
rate /odom 20
rate /camera/color/image_raw 10

head "TF tree"
frames=$(timeout 8 ros2 run tf2_ros tf2_echo map base_link 2>&1 | head -20)
if echo "$frames" | grep -qi "translation"; then
  pass "map -> base_link resolves (localisation is alive)"
else
  warn "map -> base_link does not resolve. Start nav2 and set the initial pose in RViz."
fi
# camera_link, not depth_camera_link: no robot_state_publisher runs on this
# robot, so the only camera frame that exists is the static transform that
# limo_start.launch.py publishes. depth_camera_link is URDF-only and will
# never resolve here -- checking for it just produces a warning that sends
# you hunting a robot_state_publisher that was never meant to be running.
timeout 8 ros2 run tf2_ros tf2_echo base_link camera_link 2>&1 | grep -qi translation \
  && pass "base_link -> camera_link resolves" \
  || warn "base_link -> camera_link missing -- is limo_start.launch.py running?"

head "Models"
for f in "$HOME"/limo_project/models/exported/*.engine "$HOME"/limo_project/models/exported/*.onnx; do
  [ -e "$f" ] && pass "$(basename "$f") ($(du -h "$f" | cut -f1))"
done
python3 -c "import tensorrt, sys; print('  ok    tensorrt', tensorrt.__version__)" 2>/dev/null \
  || warn "python3 tensorrt module not importable (fine if you only use onnxruntime)"
python3 -c "import cv2; print('  ok    opencv', cv2.__version__)" 2>/dev/null \
  || fail "opencv missing -- the dashboard stream needs it"

head "Next"
cat <<'EOF'
  Terminal 1: ros2 launch limo_bringup limo_start.launch.py
  Terminal 2: ros2 launch astra_camera dabai.launch.py        # or orbbec_camera
  Terminal 3: ros2 launch limo_bringup limo_nav2.launch.py
  Terminal 4: ros2 launch limo_project_bringup mission.launch.py detector:=mock
  Browser   : http://$(hostname -I | awk '{print $1}'):8080
EOF
