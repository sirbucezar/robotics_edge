# LIMO Pro — field reference

Everything here is from the AgileX manuals and the `limo_ros2` / `limo_pro_doc`
sources, not from memory. Verify the italicised items on the actual robot in
your first ten minutes, because AgileX ships several image revisions and the
camera driver package name in particular moves around.

---

## Hardware

| | |
|---|---|
| Compute | NVIDIA **Jetson Orin Nano**, 8 GB LPDDR5, 1024-core Ampere GPU (32 tensor cores), 6-core Cortex-A78AE |
| Storage | 128 GB NVMe |
| OS | **Ubuntu 20.04** → JetPack 5.x → **CUDA 11.4, TensorRT 8.5, ROS 2 Foxy** |
| LiDAR | EAI / YDLIDAR **T-mini Pro**, 360°, 0.02–12 m, 6 Hz (recommended), 0.54° resolution |
| Depth camera | **Orbbec Dabai**, binocular structured light. Depth 0.3–3 m, depth 640×400@30, colour up to 1920×1080@30. Depth FOV H 67.9° V 45.3°, colour FOV H 71° |
| IMU | HI226 |
| Chassis | 4× 14.4 W hub motors, 1 m/s max, 4 kg payload, 322×220×251 mm, wheelbase 200 mm |
| Battery | 10 Ah 12 V, ~2.5 h working |
| Displays | 1.54" front OLED, 7" 1024×600 rear touchscreen |

**The Orin Nano has no DLA.** Everything runs on the GPU. Don't plan around
offloading to a deep-learning accelerator the way you could on an Orin NX.

### Steering modes

Set by the physical latches on the front wheels, confirmed by the light colour:

| Latch | Light | Mode |
|---|---|---|
| Down | Yellow | 4-wheel differential (or tracked) |
| Down | Blue | Mecanum |
| Up | Green | Ackermann |

**Use 4-wheel differential.** It can rotate in place, which matters enormously
for a "turn to face the person" behaviour and for AMCL recovery rotations.
Ackermann needs `limo_nav2_ackmann.launch.py` and a 0.4 m turning radius, which
in a classroom full of chair legs is a fight you don't need.

### Camera geometry — the thing the brief is hinting at

> *"Bare in mind the position of the camera in the LimoPro."*

The Dabai sits on the front of the chassis, roughly **15–20 cm above the
floor**, looking level. Consequences that drive every design decision downstream:

- **You will rarely see a whole person.** At 1.5 m you get legs and lower
  torso; at 3 m you get most of a body with the head clipped. COCO's `person`
  class is overwhelmingly full-body, eye-level pedestrians, so a stock
  YOLOv8n will be noticeably worse here than its published mAP suggests. This
  is the single strongest argument for fine-tuning, and it's the argument to
  make in the report.
- **It's the head that gets clipped, not the feet.** A level camera 18 cm up
  with a 480 px sensor sees the floor from about 0.35 m outward, so feet are
  usually in frame; but the top of the visible column is only
  `0.18 + 0.535 × range` metres, so a standing person's head doesn't enter the
  frame until roughly 2.8 m. At 1.5 m you see up to about waist height.
- **Bbox height is therefore a useless range cue** — the top edge is the image
  border, not the person. Bbox *width* (shoulders) is what's stable.
- **Ground-plane projection is worse than it looks.** Feet being visible makes
  it tempting, but at 18 cm camera height the ray to a distant foot is nearly
  parallel to the floor, so a few pixels of jitter swing the intersection by
  metres. Measured at 4 m with 5 px of noise: 1.4 m mean error versus 0.30 m
  for shoulder width. It is used only within 2.5 m. Reproduce with
  `ros2_ws/src/limo_perception/test/test_geometry_roundtrip.py`.
- **Depth range is 0.3–3 m.** Beyond 3 m the Dabai gives you nothing, so
  detections at 4–6 m have to be ranged geometrically or ignored.
- Seated people at desks are mostly occluded. Expect this in the classroom and
  make sure your training frames contain it.

> *Verify on the robot:* `ros2 run tf2_ros tf2_echo base_link depth_camera_link`.
> The URDF in the public `limo_ros2` repo has the camera at `z = +0.3` relative
> to `base_link`, which would put it 45 cm off the floor — taller than the whole
> robot. Something is off in that xacro. Trust `tf2_echo` on the actual machine,
> and if it's wrong there too, fix the xacro, because the tracker's projection
> is only as good as this transform.

---

## Access

| | |
|---|---|
| SSH / desktop user | `agilex` |
| Password | `agx` |
| Remote desktop | **NoMachine** (`nomachine.com/download`), robot and laptop on the same WiFi |
| WiFi setup | Plug a keyboard/mouse into the USB hub behind the right gull-wing door, or use the 7" touchscreen |
| App | "Nexus" (iOS App Store / Android), Bluetooth, ≤10 m |

First thing every session: `ssh-copy-id agilex@<ip>` so you stop typing `agx`.

### The 1/2 prompt

The LIMO image has **both ROS 1 and ROS 2 installed**, and every new
interactive shell asks you to type `1` or `2`. **Type 2.**

This bites you in two places:
- Non-interactive `ssh host "command"` never sees the prompt, so `ROS_DISTRO`
  is unset and `ros2` isn't on `PATH`. Source `/opt/ros/foxy/setup.bash`
  explicitly in scripts — `sync_to_robot.sh` already does.
- A `systemd` unit or a `tmux` pane started the wrong way will silently be a
  ROS 1 shell. `robot_check.sh` catches this on line one.

### Workspaces

| Path | What |
|---|---|
| `~/limo_ros2_ws` | AgileX's stack. `src/limo_ros2/limo_bringup/{launch,maps,param}` |
| `~/limo_project` | Ours (created by `sync_to_robot.sh`) |

---

## Commands that matter

```bash
# chassis + lidar + robot_state_publisher  (always first)
ros2 launch limo_bringup limo_start.launch.py

# camera — name varies by image revision, try both
ros2 launch astra_camera dabai.launch.py
ros2 launch orbbec_camera dabai.launch.py

# --- mapping, once, on day 1 ---
ros2 launch limo_bringup cartographer.launch.py
#   drive SLOWLY with the remote; fast driving wrecks the map
cd ~/limo_ros2_ws/src/limo_ros2/limo_bringup/maps
ros2 run nav2_map_server map_saver_cli -f classroom

# --- navigation ---
# edit limo_nav2.launch.py to point at 'classroom' instead of 'map11'
gedit ~/limo_ros2_ws/src/limo_ros2/limo_bringup/launch/limo_nav2.launch.py
cd ~/limo_ros2_ws && colcon build
ros2 launch limo_bringup limo_nav2.launch.py          # differential/mecanum/track
ros2 launch limo_bringup limo_nav2_ackmann.launch.py  # ackermann only

# --- ours ---
ros2 launch limo_project_bringup mission.launch.py detector:=mock
ros2 launch limo_project_bringup mission.launch.py detector:=yolo
```

After `limo_nav2.launch.py` comes up, the laser scan will not line up with the
map. **Set the initial pose**: RViz → *2D Pose Estimate*, click roughly where
the robot is, then drive it in a small circle with the remote until the scan
snaps onto the walls. Nothing map-frame works until this is done — including
our mock detector, which needs `map → depth_camera_link`.

---

## Topics and frames

Published by AgileX's stack:

| Topic | Type | From |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | *subscribed* by `limo_base` |
| `/odom` | `nav_msgs/Odometry` | `limo_base` |
| `/imu` | `sensor_msgs/Imu` | `limo_base` (HI226) |
| `/limo_status` | `limo_msgs/LimoStatus` | battery voltage, motion mode, error code |
| `/scan` | `sensor_msgs/LaserScan` | T-mini Pro |
| `/map`, `/amcl_pose` | | nav2 |
| `/camera/color/image_raw` | `sensor_msgs/Image` | *verify the namespace* |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 16UC1, millimetres |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | |

> *Verify:* `ros2 topic list | grep -i camera`. The Orbbec driver's topic
> namespace differs between the `astra_camera` and `orbbec_camera` builds. If
> yours differ, change `image_topic` / `depth_topic` / `camera_info_topic` in
> `mission_params.yaml` — nothing else needs touching.

TF chain:

```
map ──amcl──> odom ──limo_base──> base_footprint ──> base_link ──┬──> laser_link
                                                                  ├──> depth_camera_link ──> depth_link
                                                                  ├──> imu_link
                                                                  └──> {front,rear}_{left,right}_wheel_link
```

`limo_msgs/LimoStatus`: `vehicle_state`, `control_mode`, `battery_voltage`,
`error_code`, `motion_mode`. Worth putting `battery_voltage` on the dashboard —
a LIMO whose battery is sagging navigates badly, and knowing that during a demo
saves you debugging a nav2 problem that is really a power problem.

---

## Simulation

The link in the brief — `agilexrobotics/ugv_gazebo_sim/tree/master/limo` — is
**ROS 1 Melodic + Gazebo Classic**. It will not run on the robot's Foxy stack
without a port. Don't sink day-one hours into it.

If you want a sim fallback, `agilexrobotics/limo_ros2` has ROS 2 Gazebo
launches (`limo_car/launch/ackermann_gazebo.launch.py`,
`limo_description/launch/gazebo_models_diff.launch.py`) which is a much shorter
path. But with the robot available all three days, `detector:=mock` on the real
hardware is a better use of the time and a more faithful test.

---

## Gotchas, ranked by how much time they cost

1. **The 1/2 ROS-version prompt.** See above.
2. **Clocks.** `sudo nvpmodel -m 0 && sudo jetson_clocks` before *any* timing
   measurement. A Jetson in a low power mode will quietly halve your FPS and
   you will blame TensorRT.
3. **TensorRT engines are not portable.** Build `.engine` on the robot. An
   engine built elsewhere fails to deserialize with an unhelpful message.
4. **Initial pose.** Nothing in the map frame works until you set it in RViz.
5. **`ROS_DOMAIN_ID`.** If your laptop and the robot are both running ROS 2 on
   the same WiFi with the default domain, you will see each other's nodes and
   get very confusing behaviour. Set `ROS_DOMAIN_ID` explicitly on both.
6. **Battery.** ~2.5 h. The demo runs on the last 20% only once.
7. **Disk.** Colour-stream rosbags fill 128 GB faster than you think. Record
   with `--compression-mode file --compression-format zstd`.
8. **`cv_bridge`.** Frequent numpy-ABI breakage on Foxy. We convert images by
   hand instead — see `imgmsg_to_bgr` in `yolo_detector_node.py`.
9. **`vision_msgs`.** Field names changed between Foxy and Humble
   (`ObjectHypothesis.id` → `class_id`). We define our own messages so this
   can't bite.
10. **The Dabai DC1 is two USB devices.** Depth/IR is an OpenNI interface
    (`2bc5:0657`); colour is a separate UVC interface (`2bc5:0557`). Launching
    `orbbec_camera` claims the UVC interface via libusb and detaches
    `uvcvideo` with no disconnect event — even for depth-only launches.
    Recovery needs more than a rebind; `scripts/camera_up.sh` escalates
    rebind → USB reset → USB deauth/reauth → `uhubctl` port power-cycle.
    Consequence: **never launch `orbbec_camera` / `dabai_d1.launch.py`.**
    The mission stack is colour-only: `color_camera_node.py` reads the Dabai
    as a plain V4L2 webcam, `people_tracker.use_depth` is `false`.

---

## Sources

- [agilexrobotics/limo_pro_doc](https://github.com/agilexrobotics/limo_pro_doc) — the ROS 2 Foxy user manual
- [agilexrobotics/limo_ros2](https://github.com/agilexrobotics/limo_ros2) — `limo_base`, `limo_description`, URDFs
- [agilexrobotics/ugv_gazebo_sim](https://github.com/agilexrobotics/ugv_gazebo_sim) — ROS 1 simulation (the brief's hint)
- [LIMO PRO product page](https://global.agilex.ai/products/limo-pro)
- [Ultralytics NVIDIA Jetson guide](https://docs.ultralytics.com/guides/nvidia-jetson) — benchmark methodology and export flags
