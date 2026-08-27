# Session handover — evening of 26 Aug 2026

Demo is **28 Aug 2026**. This is where the live-SLAM work stopped tonight and
exactly how to pick it up tomorrow morning.

Nothing in this file is committed. Commits are yours.

---

## Robot identity — read this first

We are on the **second robot**. The original was handed back and wiped.

| | |
|---|---|
| ssh alias | `limo` → `172.30.252.161`, user `agilex`, password `agx` |
| Key auth | installed, passwordless |
| Hostname | `master` (both robots use this — not a way to tell them apart) |
| Camera USB serial | `CC1365303WA` (old robot was `CC13653031C`) |

`scripts/sync_to_robot.sh` now defaults to the `limo` **ssh alias** instead of
`limo.local`, which never resolved on the school network. Swapping robots is a
one-line `HostName` change in `~/.ssh/config`; no script edits.
Backup of the previous ssh config: `~/.ssh/config.bak.20260826_133604`.

### The WiFi drops. Repeatedly.

The robot fell off the network twice tonight mid-session. Symptoms: `ping`
100% loss, ssh `Host is down`, ARP entry `(incomplete)`. **A reboot brings it
back on the same IP** — that is what worked, not waiting.

USB-C fallback did not work: macOS enumerated **no device at all** from the
cable tried (`system_profiler SPUSBDataType` showed nothing). Almost certainly
a charge-only cable. If WiFi becomes unworkable tomorrow, find a known-data
USB-C cable; the robot exposes a fixed `192.168.55.1` on its gadget interface,
which is immune to DHCP and to the school network entirely.

---

## What is DONE and verified

### Phase 0 — environment
- `nvpmodel -m 0` + `jetson_clocks`. Mode 0 is **15 W**, which is maximum for
  the Orin Nano 8 GB — there is no MAXN tier on this module. Do not hunt for a
  higher one. **Resets on every reboot — reapply.**
- All 6 packages build clean.

### Phase 1 — detector baseline (rubric: ≥ 50 FPS)

Stock COCO yolov8n. **Measured on this robot, clocks pinned:**

| Model | Input | Inference | p95 | Inference FPS | End-to-end | E2E FPS |
|---|---|---|---|---|---|---|
| **fp16** | 416 | 5.02 ms | 5.05 | **199.1** | 13.75 ms | 72.8 |
| fp32 | 416 | 7.68 ms | 7.71 | 130.1 | 14.53 ms | 68.8 |

`trtexec` GPU-only throughput: 240.6 qps fp16, 145.2 fp32 at 416;
75.7 qps fp32 at 640.

Clears the 50 FPS line by ~4x on inference and still clears it end-to-end.
Avg 1.09 detections/frame across 28 real classroom frames, so the detector is
genuinely finding people, not just running fast.

**The finding worth writing up:** inference is 5.02 ms but end-to-end is
13.75 ms. The forward pass is only 36% of the pipeline — letterbox, HtoD copy
and NMS dominate. Further quantisation buys almost nothing; INT8 would shave
~2 ms off a 5 ms stage. That is the "what we learned" paragraph of the 30%
report.

Engines on the robot at `~/limo_project/models/exported/`:
`yolov8n_416_fp16.engine`, `yolov8n_416_fp32.engine`,
`yolov8n_640_fp16.engine`, `yolov8n_640_fp32.engine`.
Both `.onnx` files are on the Mac in `models/exported/`.

**640 engines are built but never benchmarked** — one command tomorrow, see
below.

### Phase 2 — nav2 on cartographer's live map, no AMCL

`Managed nodes are active`, all five lifecycle nodes, no hang.

| Check | Result |
|---|---|
| `map → base_link` | resolves, from cartographer |
| amcl running | none — correct |
| map_server running | none — correct |
| Global costmap | populated, **same dimensions as `/map`** |
| Local costmap | 60×60 rolling |

New files: `launch/nav2_live_slam.launch.py`, `config/nav2_live_slam.yaml`
(a verbatim copy of AgileX's tuned `nav2.yaml`, so their controller/planner
tuning is preserved; the amcl/map_server sections are deliberately left in and
simply never read).

Three traps, all resolved:
1. **Lifecycle manager** — sidestepped by including nav2's own
   `navigation_launch.py`, whose `node_names` already excludes map_server and
   amcl. Nothing to curate by hand.
2. **`/map` QoS** — measured, not assumed. Cartographer's `occupancy_grid_node`
   publishes **RELIABLE + TRANSIENT_LOCAL**, so nav2's default
   `map_subscribe_transient_local: True` already matches. Exposed as a launch
   argument anyway.
3. **Growing map** — `global_costmap` has `rolling_window` absent (false) with
   `static_layer` enabled. Stock was already correct. Do not "fix" it.

### Dashboard — works

`http://172.30.252.161:8080`. It was never broken; the mission stack simply
had not been launched. Verified: HTTP 200, `/api/state` live JSON, and
`/stream.mjpg` serving real JPEG frames.

### Browser WASD teleop — new, works, then the user stopped it

Added because the AgileX phone app would not connect, and cartographer needs a
manually driven seed map before nav2 has anywhere to plan.

- `POST /api/teleop?vx=..&wz=..` → `/cmd_vel`
- Hold W/A/S/D in the page; badge bottom-right shows live state
- Capped 0.35 m/s / 0.9 rad/s, all limits in `mission_params.yaml`
- **Deadman**: browser resends at 10 Hz; if commands go stale for 400 ms the
  node publishes one zero Twist then goes *silent*, handing `/cmd_vel` back to
  nav2. Silence matters — a stream of zeros would fight the controller.

Verified: endpoint 200, bad input rejected 400, `/cmd_vel` wired.

**The user called "stop it" during the first drive and we never learned why.**
Robot was stopped and confirmed stationary (200 odom samples,
`max|linear.x| = 0.0000`, `max|angular.z| = 0.0000`), dashboard killed.

**Ask before driving again.** The plausible causes need different fixes:
too fast (drop `teleop_max_vx` to 0.15), kept moving after key release
(deadman bug — investigate before anyone drives), drove somewhere unsafe, or
wheels not in 4-wheel differential mode. A fallback exists that needs no key
holding: short fixed bursts commanded one at a time from the Mac.

---

## Two real bugs fixed tonight

**1. TensorRT vs numpy.** `import tensorrt` died with
`module 'numpy' has no attribute 'bool'` — TRT 8.5's bindings do `bool: np.bool`
and JetPack 5.x ships numpy 1.24.4, which removed the alias. Two-line shim in
`limo_perception/backends.py`, the one file allowed to import tensorrt, so it
covers both the node and `benchmark.py`. **This would have broken
`detector:=yolo` on demo day with no obvious cause.**

**2. `mission_params.yaml` pointed at a model that does not exist** —
`person_yolov8n_int8.engine` with `imgsz: 640` against a 416 engine. Now
`yolov8n_416_fp16.engine`, `imgsz: 416`, with a fallback chain down to raw
ONNX.

---

## RViz — fragile, workaround found

Two separate failures:
1. `qt.qpa.xcb: could not connect to display` — `DISPLAY` unset over ssh/tmux.
   `cartographer.launch.py` spawns an rviz2 on every launch, so it dies every
   time. Cosmetic; mapping is unaffected.
2. With `DISPLAY=:0`: `Cannot create GL vertex buffer` — no hardware GL outside
   the NoMachine session.

Works with:
```bash
export DISPLAY=:0 LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1
```
**but `nav2_default_view.rviz` segfaults under llvmpipe (exit 245).**
`demo_2d.rviz` is stable. Add costmap/path displays one at a time.

Do not sink time into RViz. Goal-reaching is verified better headlessly via the
`/navigate_to_pose` action result.

---

## NOT done — tomorrow, needs the robot

1. **Drive to seed the map** (blocked on the teleop question above).
2. **Phase 3: goal reaching** — MAKE OR BREAK. Send goals via
   `/navigate_to_pose`, confirm path planned / moved / arrived.
   If this fails, fall back to saved-map + AMCL. **Decide early.**
3. **Phase 3b: unmapped obstacle** — chair in an unmapped spot, goal routed
   through it, prove the local costmap sees it and the path re-routes.
4. **Phase 4: full stack with `detector:=yolo`** + resource headroom, and
   inference FPS *under load* vs the isolated 199.
5. **Step 4 graded artifacts** — 15% of the grade, needs a live robot:
   `frames.pdf` (`ros2 run tf2_tools view_frames`), `topics.txt`, `nodes.txt`,
   `ros2 node info` per node, and an insurance rosbag.
6. **Verify `detector:=mock` still works end to end** — graded fallback.
7. **Benchmark the 640 engines** (built, unmeasured).

---

## Resume sequence

```bash
# 1. reachable? if not, reboot the robot -- it comes back on the same IP
ssh limo 'hostname -I; uptime -p'

# 2. power mode resets on every reboot
ssh limo 'echo agx | sudo -S nvpmodel -m 0; echo agx | sudo -S jetson_clocks'

# 3. deploy
./scripts/sync_to_robot.sh --build

# 4. vendor cartographer config is NOT in git -- idempotent, safe to re-run
ssh limo 'bash ~/limo_project/scripts/apply_cartographer_fix.sh'

# 5. bring up, in this order, each in its own tmux window
ros2 launch limo_bringup limo_start.launch.py          # expect 0 checksum errors
ros2 launch limo_bringup cartographer.launch.py        # expect imu ~100, odom ~50, scan ~10
ros2 launch limo_project_bringup nav2_live_slam.launch.py   # expect "Managed nodes are active"
ros2 launch limo_project_bringup mission.launch.py detector:=mock

# 6. benchmark the 640 engines (2 min, no driving needed)
ssh limo 'cd ~/limo_project && python3 training/benchmark.py \
    --images ~/limo_project/data/frames \
    --model tensorrt:models/exported/yolov8n_640_fp16.engine \
    --model tensorrt:models/exported/yolov8n_640_fp32.engine \
    --imgsz 640 --out models/benchmarks/results_640'
```

---

## Uncommitted changes (yours to commit)

```
scripts/ab_capture.py                                   new
scripts/apply_cartographer_fix.sh                       new
scripts/sync_to_robot.sh                                default host -> limo alias
scripts/robot_check.sh                                  camera_link, not depth_camera_link
ros2_ws/src/limo_perception/limo_perception/backends.py numpy/TensorRT shim
ros2_ws/src/limo_dashboard/limo_dashboard/dashboard_node.py  WASD teleop + deadman
ros2_ws/src/limo_dashboard/limo_dashboard/web_assets.py      WASD UI
ros2_ws/src/limo_dashboard/package.xml                  + geometry_msgs
ros2_ws/src/limo_project_bringup/launch/nav2_live_slam.launch.py   new
ros2_ws/src/limo_project_bringup/config/nav2_live_slam.yaml        new
ros2_ws/src/limo_project_bringup/config/mission_params.yaml        model path, imgsz, teleop
docs/04_session_handover.md                             this file
models/exported/yolov8n_416.onnx, yolov8n_640.onnx      new
```

## Also on the Mac

- `~/Desktop/ab_test/` — 28 A/B camera-height frames + manifest, verified
  byte-for-byte. Also pushed to the robot as `data/frames` and used as the
  benchmark image set.
- Camera-height A/B artifact: https://claude.ai/code/artifact/cd970db3-87c2-4dcc-a7a2-10093cced9f0

## Mac-only work possible tonight, no robot needed

- The 30% optimization report — the FP16/FP32 table and the
  "preprocessing dominates" finding are both ready to write up.
- The camera-height A/B section, already written up in the artifact.
