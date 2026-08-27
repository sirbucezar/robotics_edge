# ROS 2 integration: node graph, topics, and transforms

## Summary

This document describes how perception and navigation connect in ROS 2 Foxy on
the robot: the transform (TF) tree, the node graph, the message contract between
detection and everything downstream, and the launch files that start each
configuration.

Every table comes from the running system. The raw captures are in
`docs/artifacts/`: `topics.txt`, `topics_with_types.txt`, `nodes.txt`,
`node_info.txt`, `frames.pdf`, and `frames.gv`. The system publishes 58 topics
across 36 nodes, with 911 lines of per-node interface detail.


## 1. Transform tree

Capture the tree with `ros2 run tf2_tools view_frames.py` while the stack runs.
The rendered diagram is `docs/artifacts/frames.pdf`.

```
map
 └── odom                    AMCL,     10.2 Hz
      └── base_link          chassis,  50.2 Hz
           ├── laser_link    static
           ├── camera_link   static
           └── imu_link      static
```

| Edge | Broadcaster | Rate | Purpose |
|---|---|---|---|
| `map` → `odom` | `amcl` | 10.2 Hz | Localization correction. Only one node may publish this. |
| `odom` → `base_link` | `limo_base` | 50.2 Hz | Wheel and inertial odometry from the chassis controller |
| `base_link` → `laser_link` | static | — | Lidar mount |
| `base_link` → `camera_link` | static | — | Camera mount, about 18 cm above the floor |
| `base_link` → `imu_link` | static | — | Inertial measurement unit mount |

**Caution:** During mapping, Cartographer publishes `map` → `odom`. During the
mission, AMCL publishes it. Never run both. Two publishers on one transform do
not fall back to each other. The tree alternates between them, and every pose in
the stack becomes unreliable in a way that resembles random navigation failure.
`nav2_amcl.launch.py` documents this requirement at the top of the file.


## 2. Node graph

<svg viewBox="0 0 880 660" xmlns="http://www.w3.org/2000/svg"
     font-family="Calibri, Helvetica, Arial, sans-serif">
  <defs>
    <marker id="a" markerWidth="9" markerHeight="7" refX="9" refY="3.5"
            orient="auto"><path d="M0,0 L9,3.5 L0,7 z" fill="#555"/></marker>
  </defs>
  <style>
    .n  { fill:#ffffff; stroke:#333333; stroke-width:1.4; rx:5; }
    .grp{ fill:#fafafa; stroke:#9a9a9a; stroke-width:1.2; stroke-dasharray:5 3; rx:6; }
    .t  { font-size:13px; fill:#111; }
    .ts { font-size:11px; fill:#555; stroke:#fff; stroke-width:3.5px;
          paint-order:stroke; }
    .tt { font-size:10.5px; fill:#333; stroke:#fff; stroke-width:3.5px;
          paint-order:stroke; }
    .e  { stroke:#555; stroke-width:1.3; fill:none; marker-end:url(#a); }
  </style>

  <!-- Painted in three passes: edges, then boxes, then every label. SVG draws
       in document order, so labels last is what guarantees no arrow can strike
       through text. The white halo on each label handles the rest. -->

  <g id="edges">
    <path class="e" d="M135,66 L135,108"/>
    <path class="e" d="M230,186 L230,232"/>
    <path class="e" d="M230,282 L230,330"/>
    <path class="e" d="M320,348 C392,340 392,282 324,270"/>
    <path class="e" d="M230,380 L230,428"/>
    <path class="e" d="M250,526 L250,572"/>
    <path class="e" d="M320,258 L558,258"/>
    <path class="e" d="M675,282 L675,330"/>
    <path class="e" d="M675,380 C675,470 470,478 442,478"/>
    <path class="e" d="M640,86 C580,150 420,210 322,240"/>
  </g>

  <g id="boxes">
    <rect class="grp" x="20" y="110" width="420" height="76"/>
    <rect class="grp" x="60" y="430" width="380" height="96"/>
    <rect class="n" x="40"  y="20"  width="190" height="46"/>
    <rect class="n" x="40"  y="124" width="180" height="48"/>
    <rect class="n" x="240" y="124" width="180" height="48"/>
    <rect class="n" x="140" y="234" width="180" height="48"/>
    <rect class="n" x="140" y="332" width="180" height="48"/>
    <rect class="n" x="160" y="574" width="180" height="46"/>
    <rect class="n" x="560" y="234" width="230" height="48"/>
    <rect class="n" x="560" y="332" width="230" height="48"/>
    <rect class="n" x="560" y="20"  width="230" height="66"/>
  </g>

  <g id="labels">
    <text class="t"  x="135" y="41" text-anchor="middle">color_camera</text>
    <text class="ts" x="135" y="57" text-anchor="middle">V4L2, 640x480 @ 30 FPS</text>
    <text class="t"  x="130" y="145" text-anchor="middle">mock_detector_node</text>
    <text class="ts" x="130" y="161" text-anchor="middle">no model, no CUDA</text>
    <text class="t"  x="330" y="145" text-anchor="middle">yolo_detector_node</text>
    <text class="ts" x="330" y="161" text-anchor="middle">TensorRT FP16, 416</text>
    <text class="ts" x="230" y="181" text-anchor="middle">one contract, either implementation</text>
    <text class="t"  x="230" y="255" text-anchor="middle">people_tracker</text>
    <text class="ts" x="230" y="271" text-anchor="middle">bbox to map, ids, count</text>
    <text class="t"  x="230" y="353" text-anchor="middle">mission / survey</text>
    <text class="ts" x="230" y="369" text-anchor="middle">state machine</text>
    <text class="t"  x="250" y="452" text-anchor="middle">nav2</text>
    <text class="ts" x="250" y="472" text-anchor="middle">amcl · map_server · planner_server</text>
    <text class="ts" x="250" y="489" text-anchor="middle">controller_server · bt_navigator</text>
    <text class="ts" x="250" y="506" text-anchor="middle">recoveries_server · waypoint_follower</text>
    <text class="t"  x="250" y="595" text-anchor="middle">limo_base</text>
    <text class="ts" x="250" y="611" text-anchor="middle">chassis, wheels, odometry</text>
    <text class="t"  x="675" y="255" text-anchor="middle">dashboard</text>
    <text class="ts" x="675" y="271" text-anchor="middle">HTTP :8080</text>
    <text class="t"  x="675" y="353" text-anchor="middle">browser</text>
    <text class="ts" x="675" y="369" text-anchor="middle">feed · map · count · stop</text>
    <text class="t"  x="675" y="41" text-anchor="middle">TF tree</text>
    <text class="ts" x="675" y="58" text-anchor="middle">map to odom to base_link</text>
    <text class="ts" x="675" y="75" text-anchor="middle">to camera_link</text>

    <text class="tt" x="145" y="92">/camera/color/image_raw</text>
    <text class="tt" x="240" y="216">/perception/detections (DetectionArray)</text>
    <text class="tt" x="240" y="312">/people/tracked (PersonArray)</text>
    <text class="tt" x="400" y="336">/people/mark_visited</text>
    <text class="tt" x="240" y="410">/navigate_to_pose (action)</text>
    <text class="tt" x="260" y="553">/cmd_vel</text>
    <text class="tt" x="470" y="196">projects a bbox into the map</text>
    <text class="tt" x="500" y="452">emergency stop</text>
  </g>
</svg>


## 3. Message contract

A message contract defines the detector, not a model. Both
`mock_detector_node`, which needs no model, no CUDA, and no camera, and
`yolo_detector_node`, which runs TensorRT, publish
`limo_mission_msgs/DetectionArray` on `/perception/detections`. No node
downstream can tell which detector is running.

This makes `detector:=mock` a working fallback. If the model misbehaves during
the graded demo, one launch argument keeps the rest of the stack running.

The project defines its own messages instead of using `vision_msgs` because the
`ObjectHypothesis` field names differ between ROS 2 Foxy and Humble. Owning the
contract prevents the robot and the development machine from disagreeing about
the wire format.

| Message | Key fields | Notes |
|---|---|---|
| `Detection` | `label`, `score`, `x`, `y`, `width`, `height` | Image space, in pixels |
| `DetectionArray` | `detections[]`, `inference_ms`, `pipeline_fps`, `backend`, `model_name` | `inference_ms` and `pipeline_fps` stay separate. The camera caps the pipeline at 30 FPS, so combining them misreports the model. |
| `Person` | `id`, `pose` in the map frame, `confidence`, `observation_count`, `visited`, `state` | `state` is `CANDIDATE`, `CONFIRMED`, or `STALE` |
| `PersonArray` | `people[]`, `confirmed_count`, `candidate_count`, `visited_count` | The dashboard renders this message |
| `MissionStatus` | `state`, `detail`, `nav_goals_sent`, `nav_goals_failed`, `replans`, `emergency_stop` | Mission telemetry |


## 4. Topics

| Topic | Type | Direction |
|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | Camera to detector and dashboard |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | Camera to tracker, for intrinsics |
| `/perception/detections` | `limo_mission_msgs/DetectionArray` | Detector to tracker and dashboard |
| `/people/tracked` | `limo_mission_msgs/PersonArray` | Tracker to mission and dashboard |
| `/people/count` | `std_msgs/Int32` | Tracker to any subscriber |
| `/people/mark_visited` | `std_msgs/Int32` | Mission to tracker |
| `/people/reset` | `std_msgs/Empty` | Dashboard to tracker |
| `/mission/status` | `limo_mission_msgs/MissionStatus` | Mission to dashboard |
| `/mission/start`, `/mission/stop` | `std_msgs/Empty` | Dashboard to mission |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Mission to navigation |
| `/cmd_vel` | `geometry_msgs/Twist` | Navigation, mission, or teleoperation to chassis |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | Localization seed to AMCL |

**Important:** `/cmd_vel` has three possible publishers, and only one may be
active at a time. Browser teleoperation implements a 400 ms deadman timer. It
publishes only while a fresh command arrives, then sends one zero velocity
message and goes silent, which returns the topic to navigation. A continuous
stream of zeros would compete with the controller and stall autonomous
navigation.


## 5. Launch files

| File | Starts | Use |
|---|---|---|
| `mission.launch.py` | Camera, detector, tracker, mission, dashboard | Arguments: `detector:=mock\|yolo\|none`, `use_mission:=`, `use_dashboard:=` |
| `nav2_amcl.launch.py` | Navigation, map server, and AMCL on the saved map | Mission day. Stop Cartographer first. |
| `nav2_live_slam.launch.py` | Navigation only, on Cartographer's live map | Unsurveyed room. No AMCL. |

Start the stack in this order:

```bash
ros2 launch limo_bringup limo_start.launch.py             # chassis and lidar
ros2 launch limo_project_bringup nav2_amcl.launch.py      # navigation and AMCL
ros2 launch limo_project_bringup mission.launch.py detector:=yolo
```

All tunable values live in `config/mission_params.yaml`. Nothing a demo might
need to change requires editing Python.


## 6. Emergency stop

The dashboard stop button performs two actions, because either action alone is
insufficient:

1. Cancels every `/navigate_to_pose` goal.
2. Transitions `controller_server` to the lifecycle state `inactive`.

An earlier version published zero velocity messages for half a second. That does
not stop the robot: the navigation controller publishes its own velocities at
10 Hz and wins. Deactivating the controller removes the competing publisher,
which zero velocities cannot do.

Verify the behavior on the robot:

```bash
ros2 lifecycle get /controller_server
# inactive  after pressing stop
# active    after clearing stop
```


## 7. Frames and the geometry test

`camera_link` sits about 18 cm above the floor. At that height, a ray to a
distant foot runs nearly parallel to the ground, so a few pixels of bounding-box
jitter change a floor-intersection range estimate by metres.

Measured at 4 m with 5 px of noise: ground-plane range error is 1.4 m mean and
5.2 m at the 95th percentile, compared to 0.30 m and 0.80 m for the
shoulder-width estimator. The tracker therefore trusts floor intersection only
within `ground_plane_max_range_m: 2.5`.

Sign-convention errors in this projection resemble detector, transform, or
localization bugs, and they take hours to isolate. Run the geometry test after
any change to `geometry.py` or the range estimators:

```bash
python3 ros2_ws/src/limo_perception/test/test_geometry_roundtrip.py
```

The test needs no ROS installation and no robot.
