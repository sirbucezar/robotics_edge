# LIMO Pro: perception-driven autonomous navigation

A ROS 2 Foxy stack that maps a classroom, localizes in it, detects and counts
people, drives to each person, and reports the count on a web dashboard. It runs
on an AgileX LIMO Pro with an NVIDIA Jetson Orin Nano 8 GB.

The detector runs at 199 frames per second (FPS) inference in TensorRT FP16.
Navigation uses Nav2 with Adaptive Monte Carlo Localization (AMCL) against a map
the robot built itself.

## How the design works

A message contract defines the detector, not a model. Two nodes publish
`limo_mission_msgs/DetectionArray` on `/perception/detections`:
`mock_detector_node`, which needs no model, no CUDA, and no camera, and
`yolo_detector_node`, which runs TensorRT. Nothing downstream can tell which one
is running.

This makes `detector:=mock` a working fallback. If the model fails during a
demo, one launch argument keeps the rest of the stack running.

Two consequences follow from that contract:

- Only `limo_perception/backends.py` imports `torch`, `tensorrt`, or
  `onnxruntime`.
- No field is added to `DetectionArray` that only one detector can populate.

## Repository layout

| Path | Contents |
|---|---|
| `docs/` | Hardware reference, and the live captures the submitted documents cite |
| `ros2_ws/src/limo_mission_msgs/` | The message contract (`ament_cmake` and `rosidl`) |
| `ros2_ws/src/limo_perception/` | Mock and YOLO detectors, inference backends, projection geometry |
| `ros2_ws/src/limo_people/` | Bounding box to map projection, track association, counting |
| `ros2_ws/src/limo_mission/` | Mission state machine and the Nav2 action client |
| `ros2_ws/src/limo_dashboard/` | Dashboard served from the Python standard library, no external dependencies |
| `ros2_ws/src/limo_project_bringup/` | Launch files, parameters, maps, room skeleton |
| `training/` | Auto-labeling, fine-tuning, pruning, export, engine build, benchmark |
| `scripts/` | Robot operations: sync, mapping, localization checks, missions |

## Before you start

Read `docs/limo_pro_reference.md` before you touch anything hardware-related.
It covers the chassis, the camera, the lidar, and the traps specific to this
robot.

## Quick start

Deploy to the robot and build:

```bash
./scripts/sync_to_robot.sh --build
```

Start the stack on the robot, in this order. Each command runs in its own
terminal or tmux window:

```bash
ros2 launch limo_bringup limo_start.launch.py             # chassis and lidar
ros2 launch limo_project_bringup nav2_amcl.launch.py      # navigation and AMCL
ros2 launch limo_project_bringup mission.launch.py detector:=yolo
```

Open the dashboard at `http://<robot-ip>:8080`. It shows the camera feed with
labeled detections, the robot's position, the live count, and an emergency stop
button.

Seed the localization and confirm the robot knows where it is:

```bash
python3 scripts/set_initial_pose.py 0 0 0     # the home pose is the map origin
python3 scripts/match_score.py                # expect 0.8 or higher
```

Run the survey mission:

```bash
python3 scripts/room_survey.py
```

The mission visits each row junction, scans from a standstill, approaches anyone
it finds, returns to the junction, and drives home when every row is done.

## Operations scripts

| Script | Purpose |
|---|---|
| `sync_to_robot.sh` | Copy the workspace to the robot and build it |
| `robot_env.sh` | Restore the ROS environment in non-interactive shells |
| `pose_probe.py` | Print the map pose, occupancy counts, and forward clearance |
| `match_score.py` | Score the live scan against the map. Above 0.7 is a good lock. |
| `pose_search.py` | Find the pose that best explains the scan when AMCL is wrong |
| `set_initial_pose.py` | Publish an AMCL initial pose |
| `send_goal.py` | Send one Nav2 goal and report what actually happened |
| `room_survey.py` | The full survey mission |
| `find_and_visit.py` | Single-person mission. The proven fallback. |
| `save_map.py` | Save a map with a chosen occupancy threshold |
| `md_to_pdf.py` | Render the documentation to PDF |

## Configuration

All tunable values live in
`ros2_ws/src/limo_project_bringup/config/mission_params.yaml`. Nothing a demo
might need to change requires editing Python.

The room geometry lives in
`ros2_ws/src/limo_project_bringup/config/room_skeleton.yaml`: every junction,
every row, and the home pose, recorded while mapping. The mission reads its route
from that file.

## Test

Run the projection geometry test after any change to `geometry.py` or the range
estimators:

```bash
python3 ros2_ws/src/limo_perception/test/test_geometry_roundtrip.py
```

The test needs no ROS installation and no robot. Sign-convention errors in this
projection resemble detector, transform, or localization bugs and take hours to
isolate, which is why the test exists.

## Constraints to respect

- The robot runs Ubuntu 20.04 and ROS 2 Foxy on JetPack 5.x, with CUDA 11.4 and
  TensorRT 8.5. Python is 3.8: no `match` statements, no `X | Y` type unions, no
  `list[str]` annotations at runtime.
- Use the project's own messages. `vision_msgs` field names differ between Foxy
  and Humble. `cv_bridge` breaks the NumPy application binary interface (ABI) on
  Foxy, so `imgmsg_to_bgr` in `yolo_detector_node.py` replaces it.
- TensorRT engines are not portable. Build the `.onnx` file on a development
  machine and the `.engine` file on the Jetson with
  `training/build_engine_on_jetson.sh`.
- The power mode resets on every boot. Reapply it with `sudo nvpmodel -m 0` and
  `sudo jetson_clocks`.
- Cartographer and AMCL both publish the `map` to `odom` transform. Never run
  both at once.
- The camera sits about 18 cm above the floor. Bounding box height is not a
  usable range cue at that height. See the measured estimator table in
  `limo_people/people_tracker_node.py`.

## Documentation

The model optimization report, the ROS 2 integration document, and the transform
diagram are submitted separately as PDFs. This repository keeps the raw captures
those documents cite.

| Path | Contents |
|---|---|
| `docs/limo_pro_reference.md` | Hardware reference for this robot |
| `docs/artifacts/` | Live captures: `topics.txt` (58 topics), `topics_with_types.txt`, `nodes.txt` (36 nodes), `node_info.txt`, `frames.gv` |
| `docs/figures/` | Dataset samples and saved maps |
