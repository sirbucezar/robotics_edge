#!/usr/bin/env bash
# Run this ON THE ROBOT, once, on every LIMO you deploy to.
#
#   ssh limo 'bash ~/limo_project/scripts/apply_cartographer_fix.sh'
#
# Why this script exists at all: the two settings it changes live in AgileX's
# own workspace (~/limo_ros2_ws), which this repo does not track and which a
# fresh robot image ships unmodified. Everything else we deploy travels in
# ~/limo_project via sync_to_robot.sh. This is the ONLY change that does not,
# which makes it the one thing silently lost when you swap robots.
#
# What it changes, and why each one matters:
#
#   tracking_frame:  "odom" -> "base_link"
#       Stock LIMO config makes cartographer track the *odom* frame. That
#       hands the scan matcher scans already pre-transformed by wheel
#       odometry, so a slipping in-place spin inserts them at the wrong angle
#       and the map comes out as a starburst of free-space rays.
#       It also makes use_imu_data fatal: cartographer_ros CHECKs that the
#       IMU frame is colocated with the tracking frame, and odom->imu_link
#       grows without bound as the robot drives away from the origin --
#       cartographer_node aborts mid-map. base_link->imu_link is 0 0 0 with
#       identity rotation, so base_link satisfies that check exactly.
#
#   TRAJECTORY_BUILDER_2D.use_imu_data:  false -> true
#       Without the gyro, heading comes from wheel odometry alone. This is a
#       skid-steer robot: an in-place spin slips hard, and with no IMU to
#       correct it the scan matcher loses lock.
#
# Published TF is unchanged: published_frame stays "odom", so cartographer
# still emits map->odom and the nav2 wiring is unaffected.
#
# Idempotent. Safe to run repeatedly; re-running a fixed robot changes nothing.

set -uo pipefail

BOLD=$'\033[1m'; DIM=$'\033[2m'; RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; OFF=$'\033[0m'
pass(){ printf "  ${GRN}ok${OFF}    %s\n" "$1"; }
warn(){ printf "  ${YEL}warn${OFF}  %s\n" "$1"; }
fail(){ printf "  ${RED}FAIL${OFF}  %s\n" "$1"; }

WS="${LIMO_VENDOR_WS:-$HOME/limo_ros2_ws}"
REL="limo_bringup/config_files/limo_lds_2d.lua"
SRC="$WS/src/limo_ros2/$REL"
SHARE="$WS/install/limo_bringup/share/$REL"
STAMP=$(date +%Y%m%d_%H%M%S)
RC=0

printf "\n${BOLD}Cartographer IMU fix${OFF}\n"
printf "  ${DIM}vendor workspace: %s${OFF}\n" "$WS"

is_fixed() {
  grep -q 'tracking_frame *= *"base_link"' "$1" 2>/dev/null &&
  grep -q 'use_imu_data *= *true'          "$1" 2>/dev/null
}

fix_file() {
  local f="$1" label="$2"

  if [ ! -f "$f" ]; then
    fail "$label not found: $f"
    return 1
  fi

  if is_fixed "$f"; then
    pass "$label already correct"
    return 0
  fi

  cp "$f" "$f.bak.$STAMP" || { fail "$label could not be backed up, refusing to edit"; return 1; }

  # Anchored on the key name so published_frame / odom_frame, which are also
  # "odom", are left alone.
  sed -i 's/tracking_frame *= *"odom"/tracking_frame = "base_link"/' "$f"
  sed -i 's/TRAJECTORY_BUILDER_2D\.use_imu_data *= *false/TRAJECTORY_BUILDER_2D.use_imu_data = true/' "$f"

  if is_fixed "$f"; then
    pass "$label patched (backup: $(basename "$f.bak.$STAMP"))"
    return 0
  fi

  fail "$label edit did not take -- unexpected upstream format, patch by hand:"
  printf "        ${DIM}%s${OFF}\n" "$f"
  grep -nE 'tracking_frame|use_imu_data' "$f" | sed 's/^/          /'
  return 1
}

fix_file "$SRC" "src config" || RC=1

# The launch file resolves config_files from the INSTALL share directory, not
# from src. A plain `colcon build` copies the file there, so editing src alone
# silently does nothing -- this is the trap that makes the fix look applied
# when it is not. Under --symlink-install the share path is a symlink and
# fixing src is enough.
if [ -L "$SHARE" ]; then
  pass "install share is a symlink to src (symlink-install) -- nothing to copy"
elif [ -f "$SHARE" ]; then
  if is_fixed "$SHARE"; then
    pass "install share already correct"
  else
    cp "$SHARE" "$SHARE.bak.$STAMP" 2>/dev/null
    if cp "$SRC" "$SHARE"; then
      pass "install share updated from src"
    else
      fail "could not write install share: $SHARE"
      RC=1
    fi
  fi
else
  warn "no install share at $SHARE -- vendor workspace not built? cartographer will use src only if you launch from a sourced overlay"
fi

printf "\n${BOLD}Effective config${OFF}\n"
EFFECTIVE="$SHARE"
[ -f "$EFFECTIVE" ] || EFFECTIVE="$SRC"
if [ -f "$EFFECTIVE" ]; then
  grep -nE 'tracking_frame|published_frame|odom_frame|use_odometry|use_imu_data|max_range' "$EFFECTIVE" \
    | sed 's/^/  /'
  printf "  ${DIM}read from: %s${OFF}\n" "$EFFECTIVE"
fi

printf "\n${BOLD}Before you map${OFF}\n"
cat <<'EOF'
  The fix is worthless if the IMU is not actually publishing -- cartographer
  will wait on data that never arrives. Confirm the chassis is alive first:

    ros2 launch limo_bringup limo_start.launch.py
    ros2 topic hz /imu          # expect ~100 Hz, /odom ~50 Hz, /scan ~10 Hz

  Total silence on all three (no "Check sum failed", just nothing) means the
  chassis MCU is not talking: power cycle the chassis, 5 s off, 15-20 s to
  come back. `ros2 topic hz` also under-reports on this platform over ssh --
  if it shows nothing, confirm with a real subscriber before believing it.

  Then, while driving: no full in-place spins. Stop, pause, turn partially.
EOF

if [ "$RC" -eq 0 ]; then
  printf "\n${GRN}${BOLD}done${OFF} -- this robot is ready to map\n\n"
else
  printf "\n${RED}${BOLD}incomplete${OFF} -- see FAIL lines above\n\n"
fi
exit "$RC"
