#!/bin/bash
# Start a ROS2 launch in its own tmux window on the robot.
# bash -c (non-login, non-interactive) so the profile "1 (ros1) or 2 (ros2)"
# prompt never appears and never eats the command -- robot_env.sh restores the
# ROS env that .bashrc would otherwise have set.
#   tmux_run.sh <window-name> <command...>
set -e
WIN="$1"; shift
tmux has-session -t limo 2>/dev/null || tmux new-session -d -s limo -n idle
tmux kill-window -t "limo:$WIN" 2>/dev/null || true
tmux new-window -d -t limo -n "$WIN" \
  "bash -c 'source ~/limo_project/scripts/robot_env.sh; $*; echo \"--- WINDOW EXITED ---\"; sleep 100000'"
echo "started limo:$WIN"
