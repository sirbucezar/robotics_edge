# Architecture

This document is the *ROS2 Integration Code & TF diagram* deliverable (15% of
the grade). It should end up in the report largely as-is.

---

## The one idea

Everything hinges on a single decision: **the detector is defined by a message
contract, not by a model.**

`limo_mission_msgs/DetectionArray` is published on `/perception/detections`.
Two nodes can publish it — `mock_detector_node` and `yolo_detector_node` — and
nothing downstream can tell which one is running.

That buys the three days:

- **Day 1** the mock detector runs on the real robot with no model, no CUDA,
  no TensorRT and no camera. Navigation, counting, the mission state machine
  and the dashboard are all built and *debugged* against it.
- **Day 2** the real model is developed and optimised in isolation, judged on
  its own numbers, with no risk of a nav2 problem being mistaken for a model
  problem.
- **Day 3** one launch argument swaps them. Everything that was already
  working keeps working.

The inverse order — build the model first, then integrate — is how this project
goes wrong, because integration bugs and model bugs look identical from the
outside and you meet them all at once on the last day.

---

## Node graph

```
      ┌──────────────────────── AgileX stack (do not modify) ───────────────────────┐
      │                                                                             │
      │  limo_base ──/odom──┐   ┌──/scan── ydlidar_node                             │
      │      ▲              │   │                                                   │
      │   /cmd_vel          ├──/tf── robot_state_publisher                          │
      │      │              │   │                                                   │
      │  ┌───┴────┐         │   └── astra_camera ──/camera/color/image_raw ────┐    │
      │  │  nav2  │◄────────┘                     /camera/depth/image_raw ──┐  │    │
      │  │ amcl   │                               /camera/color/camera_info │  │    │
      │  │planner │──/map──►                                                │  │    │
      │  │ctrl bt │                                                         │  │    │
      │  └───▲────┘                                                         │  │    │
      └──────┼──────────────────────────────────────────────────────────────┼──┼────┘
             │ NavigateToPose (action)                                      │  │
             │                                                              │  │
    ┌────────┴─────────┐                                                    │  │
    │   mission_node   │                                                    │  │
    │                  │                                                    │  │
    │  IDLE            │                       ┌────────────────────────┐   │  │
    │  LOCALIZING      │◄──/people/tracked─────┤   people_tracker_node  │◄──┘  │
    │  PATROLLING      │                       │                        │      │
    │  APPROACHING     │──/people/mark_visited►│  bbox ──► 3D ──► map    │      │
    │  DWELLING        │                       │  associate, confirm     │      │
    │  HOLDING         │                       └───────────▲────────────┘      │
    │  DONE            │                                   │                    │
    └────────┬─────────┘                    /perception/detections              │
             │                                              │                    │
        /mission/status                    ┌────────────────┴──────────────┐    │
             │                             │  ONE OF:                       │    │
             │                             │   mock_detector_node           │    │
             │                             │   yolo_detector_node ──────────┼────┘
             │                             │     tensorrt → onnxruntime     │
             │                             │              → ultralytics     │
             │                             └────────────────────────────────┘
             ▼
    ┌──────────────────┐
    │  dashboard_node  │  http://<limo-ip>:8080
    │  count · stream  │  MJPEG + JSON, stdlib http.server only
    │  map · telemetry │
    └──────────────────┘
```

## Topics

| Topic | Type | Pub | Sub |
|---|---|---|---|
| `/perception/detections` | `limo_mission_msgs/DetectionArray` | mock **or** yolo detector | tracker, dashboard |
| `/perception/inference_fps` | `std_msgs/Float32` | yolo detector | (logging) |
| `/people/tracked` | `limo_mission_msgs/PersonArray` | tracker | mission, dashboard |
| `/people/count` | `std_msgs/Int32` | tracker | (convenience) |
| `/people/markers` | `visualization_msgs/MarkerArray` | tracker | RViz |
| `/people/mark_visited` | `std_msgs/Int32` | mission | tracker |
| `/people/reset` | `std_msgs/Empty` | dashboard | tracker |
| `/mission/status` | `limo_mission_msgs/MissionStatus` | mission | dashboard |
| `/mission/start`, `/mission/stop` | `std_msgs/Empty` | dashboard | mission |
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | nav2 (server) | mission (client) |

QoS: detections and people are **reliable, depth 5** — losing a detection frame
is fine but losing a *count update* is not. Image and depth subscriptions are
**best-effort, depth 1**, because a stale camera frame is worse than no frame.

## TF frames

```
map
 └── odom                     ← amcl (nav2), corrects odometry drift
      └── base_footprint      ← limo_base, from wheel odometry + IMU
           └── base_link      (+0.15 m)
                ├── laser_link            (+0.103, 0, −0.034 from base_link)
                ├── depth_camera_link ──► depth_link      ← camera; VERIFY THIS ONE
                ├── imu_link
                └── {front,rear}_{left,right}_wheel_link
```

The tracker's whole job depends on `map → depth_camera_link`. If AMCL is not
converged, or the camera transform is wrong, people land in the wrong place and
the count inflates as the robot moves. Check it before blaming the detector:

```bash
ros2 run tf2_ros tf2_echo map depth_camera_link
ros2 run tf2_tools view_frames        # writes frames.pdf — put this in the report
```

---

## How perception changes navigation

The rubric asks that "model inference ... affects navigation decisions". Three
separate mechanisms, all in `mission_node.py`:

1. **Preemption.** A newly confirmed, unvisited person cancels the active
   waypoint goal. Detection latency sits directly on the control path.
2. **Goal synthesis.** The approach pose is computed from the tracked map
   position: stand `approach_distance_m` short of the person on the robot–person
   line, yawed to face them. A different detection produces a different goal.
3. **Veto.** Anyone inside `hold_distance_m` cancels the goal and holds until
   they move (with hysteresis via `hold_release_distance_m`). This is the
   "without crushing" requirement, enforced *above* nav2 so it works even before
   the costmap has seen the person.

## How the count stays honest

Naively counting detections gives you a number in the hundreds. Four mechanisms
keep it right, and each is worth a sentence in the report:

1. **Count in the map frame, not the image.** Position is viewpoint-invariant;
   pixels are not. This alone does most of the work. Range comes from a
   three-estimator cascade — depth sensor (0.3–3 m), floor intersection
   (< 2.5 m only), apparent shoulder width (everywhere else). The ordering is a
   *measured* result: floor intersection looks like the principled choice but
   amplifies bbox noise catastrophically at an 18 cm camera height, and the
   numbers are in `limo_perception/test/test_geometry_roundtrip.py`. Good
   report material.
2. **Nearest-neighbour association with a gate** (`association_radius_m`,
   default 0.9 m). Too small and one person is counted twice as the robot drives
   past; too large and two people at adjacent desks merge. Tune it with the mock
   detector against known positions — that's exactly what `people_xy` is for.
3. **Confirmation threshold** (`min_observations_to_confirm`, default 6). A
   single-frame false positive never reaches the count.
4. **Stale, not deleted.** A confirmed person who leaves the field of view is
   marked `STATE_STALE` and stays in the count. People in a classroom sit down;
   they do not cease to exist.

## Failure modes, and what happens

| Failure | Behaviour |
|---|---|
| TensorRT engine missing or stale | detector falls back to onnxruntime, then to ultralytics; logs a loud warning |
| Camera not publishing | detector logs every 5 s; dashboard shows "no camera stream"; mission still patrols |
| AMCL not localised | tracker refuses to place people (warns, throttled); mission stays in `LOCALIZING` |
| nav2 rejects a goal | retry up to `max_goal_retries`, then mark the person unreachable and continue |
| Person unreachable (behind a desk) | approach times out at `goal_timeout_s`, marked visited-with-reason, patrol resumes |
| Someone steps in front of the robot | `HOLDING`, goal cancelled, red border on the video stream |
| Person detected twice at one spot | association gate merges them |
| Two people standing shoulder to shoulder | **known limitation** — they merge into one track. Say so in the reflection; the fix is per-detection appearance embedding or a proper multi-object tracker, which is out of scope in three days |

---

## Deliverable mapping

| Deliverable | Weight | Where it comes from |
|---|---|---|
| Live demo | 40% | `mission.launch.py detector:=yolo` + the nav2 stack |
| Model optimization report | 30% | `training/benchmark.py` → `results.md`, `training/nas_sweep.py` → `nas_sweep.png`, `docs/03_optimization_report.md` |
| ROS2 integration code & TF diagram | 15% | this document + `ros2 run tf2_tools view_frames` |
| Video | 15% | screen-record the dashboard alongside a phone video of the robot |
