# LIMO Pro — Perception-Driven Autonomous Navigation

Edge-AI & Robotics final project (2nd chance), HOWEST CTAI.
Robot visits every person in a classroom, counts them, doesn't hit anyone, and
shows the count on a dashboard served from the robot itself.

## The design in one paragraph

The detector is defined by a **message contract**, not by a model.
`limo_mission_msgs/DetectionArray` on `/perception/detections` can be published
by `mock_detector_node` (no model, no CUDA, no camera) or by
`yolo_detector_node` (TensorRT → ONNX Runtime → PyTorch, in that fallback
order). Nothing downstream can tell the difference. So navigation, counting,
the mission state machine and the dashboard all get built and debugged on day 1
against the mock, the model gets developed in isolation on day 2, and day 3 is
one launch argument. Building the model first and integrating last is how this
project runs out of time.

## Layout

```
docs/
  00_limo_pro_reference.md    hardware, credentials, commands, gotchas
  01_architecture.md          node graph + TF + topics — the 15% deliverable
  02_three_day_plan.md        hour by hour
  03_optimization_report.md   skeleton for the 30% deliverable
ros2_ws/src/
  limo_mission_msgs/          the contract: Detection, Person, MissionStatus
  limo_perception/            mock + YOLO detectors, pluggable backends, geometry
  limo_people/                bbox → 3D → map, association, counting
  limo_mission/              patrol / approach / dwell / hold state machine
  limo_dashboard/             stdlib http.server dashboard, MJPEG + JSON
  limo_project_bringup/       launch, params, maps, rviz
training/
  autolabel.py                pre-label robot frames with a big teacher model
  finetune.py                 two-stage fine-tune, single class
  prune.py                    structured channel pruning + recovery
  export_onnx.py              opset 12, no baked NMS, static batch
  build_engine_on_jetson.sh   FP32 / FP16 / INT8 — must run on the robot
  benchmark.py                the report's latency table
  nas_sweep.py                variant × resolution Pareto front
scripts/
  robot_check.sh              20-second "can anything I try now possibly work"
  sync_to_robot.sh            rsync Mac → robot, optionally colcon build
  record_bag.sh               record the classroom (training set + regression test)
  grab_frames.py              bag or live stream → JPEGs
```

## Quick start

On the Mac:

```bash
LIMO_HOST=<limo-ip> ./scripts/sync_to_robot.sh --build
```

On the robot (four terminals — remember to answer **2** to the ROS-version prompt):

```bash
ros2 launch limo_bringup limo_start.launch.py       # chassis + lidar + TF
ros2 launch astra_camera dabai.launch.py            # or orbbec_camera
ros2 launch limo_bringup limo_nav2.launch.py        # then set the pose in RViz
ros2 launch limo_project_bringup mission.launch.py detector:=mock
```

Then open `http://<limo-ip>:8080` and press **Start**.

Swap `detector:=mock` for `detector:=yolo` once you have an engine.

## Everything you'll want to change lives in one file

`ros2_ws/src/limo_project_bringup/config/mission_params.yaml` — waypoints,
camera topic names, association radius, approach and hold distances, model
path. Never edit Python during a demo.

## Before touching anything

Read `docs/00_limo_pro_reference.md`, especially:

- Every new shell asks **1 (ros1) or 2 (ros2)**. Type 2. Non-interactive `ssh`
  never sees the prompt.
- `sudo nvpmodel -m 0 && sudo jetson_clocks` before any timing measurement.
- TensorRT engines are **not portable** — build them on the robot.
- Nothing in the map frame works until you set the initial pose in RViz.
