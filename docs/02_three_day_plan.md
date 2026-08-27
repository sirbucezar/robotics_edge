# Three days to the 28th

Exam: **Friday 28 August**, live demo. Today is **Friday 21 August**.

The plan is built around one rule: **anything that needs the robot happens
while you have the robot; anything that doesn't, doesn't.** The two things only
the physical robot can give you are the classroom map and camera footage from
20 cm off that classroom's floor. Get both on day 1 and days 2 and 3 stop
depending on the hardware being free.

---

## Day 1 — foundations (today)

**Goal by end of day:** the robot navigates to waypoints on a map you made, the
mock detector drives the counter, and the dashboard shows a number that changes
when you move an imaginary person. No model exists yet, and that's fine.

### Block A — get in (45 min)

```bash
ssh agilex@<limo-ip>            # password: agx
ssh-copy-id agilex@<limo-ip>    # stop typing agx
```

- Note the IP. Set `ROS_DOMAIN_ID` to the same non-zero value on the robot and
  your Mac, and write it in this file.
- `sudo nvpmodel -m 0 && sudo jetson_clocks`
- `git clone` your repo, or `./scripts/sync_to_robot.sh --build` from the Mac.
- `bash ~/limo_project/scripts/robot_check.sh` — read every line.
- Set the wheels to **4-wheel differential** (latches down, yellow light).

### Block B — bring the robot up (45 min)

Four terminals, in order:

```bash
ros2 launch limo_bringup limo_start.launch.py
ros2 launch astra_camera dabai.launch.py          # or orbbec_camera
rviz2                                              # add LaserScan, Image, TF
ros2 topic list | grep -i camera                   # WRITE THE REAL NAMES DOWN
```

Put the real camera topic names into `mission_params.yaml` now. Confirm
`ros2 run tf2_ros tf2_echo base_link depth_camera_link` and note the height —
compare it against a tape measure. Drive around with the remote; watch `/scan`
move in RViz.

### Block C — map the classroom (60 min) ⚑ needs the robot

```bash
ros2 launch limo_bringup limo_start.launch.py
ros2 launch limo_bringup cartographer.launch.py
# drive SLOWLY. Cover the whole room. Close the loop by returning to the start.
cd ~/limo_ros2_ws/src/limo_ros2/limo_bringup/maps
ros2 run nav2_map_server map_saver_cli -f classroom
```

Copy `classroom.pgm` / `classroom.yaml` into
`ros2_ws/src/limo_project_bringup/maps/` **and commit them**. Edit
`limo_nav2.launch.py` to load `classroom`, rebuild `~/limo_ros2_ws`.

Then bring up nav2, set the initial pose in RViz, and send a couple of
*2D Nav Goal*s by hand. Do not proceed until the robot drives to a clicked
point reliably. Everything else is built on this.

Read waypoint coordinates off RViz (`ros2 topic echo /clicked_point` while
using *Publish Point*) and put them in `mission_params.yaml` — in **both** the
`mission` and `dashboard` sections.

### Block D — record the classroom (45 min) ⚑ needs the robot AND people

The highest-leverage 45 minutes of the whole project. Grab whoever is around.

```bash
bash ~/limo_project/scripts/record_bag.sh classroom_run1
```

Drive slowly while people sit at desks, stand, walk past, partly occlude each
other, and stand very close to the robot. Two or three minutes of *varied*
footage beats fifteen minutes of an empty room. Do a second run with different
lighting if the room has blinds.

```bash
ros2 bag play data/bags/classroom_run1 --rate 0.5
python3 ~/limo_project/scripts/grab_frames.py --every 5 --out ~/limo_project/data/frames
rsync -azP agilex@<ip>:~/limo_project/data/frames/ ./data/frames/
```

You want **300–600 frames**. This is your training set and your day-3
regression test.

### Block E — prove the stack (60 min)

```bash
ros2 launch limo_project_bringup mission.launch.py detector:=mock
```

Open `http://<limo-ip>:8080`. Then:

- Put `people_xy` at coordinates you can measure with a tape.
- Press **Start**. The robot should patrol, spot an imaginary person, preempt,
  drive to ~1.1 m from them, dwell, mark them visited, and carry on.
- **Check the recovered coordinates against the tape measure.** If the tracker
  reports the positions you typed in, projection + association + counting are
  all correct and the only unproven component left is the model.
- Deliberately break things: stand in front of the robot (→ `HOLDING`), kill
  nav2 (→ goals fail, mission recovers), set `association_radius_m` to 3.0 and
  watch two people merge. Every one of these is a paragraph in the reflection.

### Block F — close out (30 min)

Commit everything. Write down in this file: robot IP, `ROS_DOMAIN_ID`, real
camera topic names, measured camera height, the waypoint coordinates, anything
`robot_check.sh` flagged.

**Day 1 is a success if:** robot navigates on your map + mock pipeline counts
and visits + 300+ classroom frames on your Mac.

---

## Day 2 — the model

**Goal:** a TensorRT engine on the robot, ≥50 FPS inference, and a benchmark
table you could defend to a hostile examiner.

### Morning — dataset and baseline (3 h)

```bash
python3 training/autolabel.py --images data/frames --out data/dataset \
    --teacher yolov8x.pt --device mps
```

Then **correct the labels**. Budget 45 minutes and do it properly. Fix misses
before you fix box edges — an unlabelled person actively teaches the model that
people are background.

```bash
python3 training/finetune.py --data data/dataset/data.yaml --device mps
```

If the M2 is too slow, push `data/dataset/` to Colab and run the same command
with `--device 0 --batch 32`. Record baseline mAP50 / mAP50-95 / precision /
recall — that's the first row of the report table.

### Afternoon — optimise (3 h)

```bash
python3 training/prune.py --weights models/baseline/person_yolov8n.pt \
    --data data/dataset/data.yaml --ratio 0.3
python3 training/export_onnx.py --weights models/baseline/person_yolov8n.pt --imgsz 640
python3 training/export_onnx.py --weights models/baseline/person_yolov8n.pt --imgsz 416
./scripts/sync_to_robot.sh --models
```

On the robot:

```bash
bash ~/limo_project/training/build_engine_on_jetson.sh \
     ~/limo_project/models/exported/person_yolov8n_640.onnx

python3 ~/limo_project/training/benchmark.py --images ~/limo_project/data/frames \
  --model ultralytics:models/baseline/person_yolov8n.pt \
  --model onnxruntime:models/exported/person_yolov8n_640.onnx \
  --model tensorrt:models/exported/person_yolov8n_640_fp32.engine \
  --model tensorrt:models/exported/person_yolov8n_640_fp16.engine \
  --model tensorrt:models/exported/person_yolov8n_640_int8.engine

python3 ~/limo_project/training/nas_sweep.py --images ~/limo_project/data/frames \
  --data data/dataset/data.yaml --variants yolov8n,yolo11n --sizes 320,416,512,640
```

`results.md` and `nas_sweep.png` go straight into the report.

### Evening — swap it in (1 h)

Point `yolo_detector.model_path` at the winning engine.

```bash
ros2 launch limo_project_bringup mission.launch.py detector:=yolo
```

Sanity-check against the bag before you trust a live run:
`ros2 bag play data/bags/classroom_run1` with the detector and tracker up, and
see whether the count matches the number of people who were actually in the room.

**Day 2 is a success if:** an engine on the robot clears 50 FPS and the
detector node runs it end to end.

---

## Day 3 — make it survive an audience

**Goal:** it works three times in a row, and the paperwork is done.

### Morning — robustness (3 h)

Run the full mission end to end, repeatedly, with real people. The rubric says
*"behaviour works more than once, not just lucky run"* — so run it at least
five times and log what happened each time. Tune:

- `association_radius_m` — the count-inflation vs merge trade-off
- `min_observations_to_confirm` — false positives vs slow confirmation
- `approach_distance_m` / `hold_distance_m` — how close is "visited", how close
  is "too close"
- `conf` — the detector threshold; likely lower than default given the
  low-camera domain

Record a bag of a *good* run. If everything falls apart on exam day, you have a
video and a replayable bag.

### Afternoon — deliverables (3 h)

- **Video (15%)**: screen-record the dashboard, phone-record the robot, cut them
  side by side. Show the count incrementing as the robot visits each person.
- **Report (30%)**: 4–6 pages from `docs/03_optimization_report.md`.
- **TF diagram (15%)**: `ros2 run tf2_tools view_frames` → `frames.pdf`, plus
  the node graph from `docs/01_architecture.md`, plus `ros2 topic list`.
- **Reflection**: the failure-mode table in the architecture doc, plus what you
  broke on purpose on day 1.

### Evening — exam-day runbook (1 h)

Write and rehearse it. Something like:

```
1. Power on, latches down (yellow light). Battery > 60%.
2. sudo nvpmodel -m 0 && sudo jetson_clocks
3. bash ~/limo_project/scripts/robot_check.sh
4. T1: ros2 launch limo_bringup limo_start.launch.py
5. T2: ros2 launch astra_camera dabai.launch.py
6. T3: ros2 launch limo_bringup limo_nav2.launch.py
7. RViz: 2D Pose Estimate, rotate until the scan locks on
8. T4: ros2 launch limo_project_bringup mission.launch.py detector:=yolo
9. Browser: http://<limo-ip>:8080  → Start
10. If the model misbehaves: Ctrl-C T4, relaunch with detector:=mock,
    demo the navigation and counting logic, explain the fallback honestly.
```

Charge both batteries. Bring an ethernet cable and a USB hub. Assume the venue
WiFi is hostile — the dashboard has no external dependencies for exactly this
reason, but you still need to reach the robot's IP, so know how to bring up its
hotspot or plug in directly.

---

## What to cut if you fall behind

In order, cut:

1. INT8 (FP16 alone is a fine report; say why you stopped there)
2. Pruning (report the unstructured-vs-structured reasoning without running it —
   the understanding is what's being graded)
3. The approach behaviour (patrol + count + hold still demonstrates
   perception-driven navigation)
4. YOLO11 in the sweep (one variant × four resolutions is still a real sweep)

Do **not** cut: the map, the classroom frames, the dashboard, or the fallback
chain. Those are what make the demo happen at all.
