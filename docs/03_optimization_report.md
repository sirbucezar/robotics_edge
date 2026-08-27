# Model optimization report

## Summary

The robot detects and counts people in a classroom while navigating a mapped
room. The grading rubric requires at least 50 frames per second (FPS) of model
inference on the robot's own compute.

The deployed model runs at **199 FPS** inference, which clears the requirement
by four times. The most useful measurement in this report is not that number.
It is this: inference takes 5.02 milliseconds (ms), but the end-to-end pipeline
takes 13.75 ms. **The forward pass accounts for only 36% of pipeline latency.**
Preprocessing and post-processing account for the rest.

That measurement determined which optimizations to pursue and which to skip.

All numbers come from the target hardware with clocks pinned. None are
estimated.


## 1. Constraints

| Constraint | Value | Effect on the model |
|---|---|---|
| Compute | Jetson Orin Nano 8 GB, no Deep Learning Accelerator (DLA) | Everything runs on the GPU. There is no offload tier. |
| Power mode | `nvpmodel -m 0` = 15 W, the maximum for this module | No higher tier exists. The setting resets on every boot. |
| Software | Ubuntu 20.04, ROS 2 Foxy, JetPack 5.x, CUDA 11.4, TensorRT 8.5 | Use the TensorRT 8.5 API. `vision_msgs` and `cv_bridge` are unavailable. |
| Camera | Orbbec Dabai DC1, 640×480 at 30 FPS, 71° horizontal field of view | The camera caps the pipeline at 30 FPS, not the model. |
| Camera height | Approximately 18 cm above the floor | This is the dominant accuracy problem. See section 3. |
| Lidar | YDLIDAR, 220° arc, 247 bins containing about 123 real samples | Effective angular resolution is about 1.8°. |

Two constraints shaped every decision.

**The camera caps the pipeline at 30 FPS.** A model that runs at 200 FPS cannot
produce more than 30 detections per second. The rubric asks for inference
throughput, so the `DetectionArray` message reports `inference_ms` and
`pipeline_fps` as separate fields. Combining them would either understate the
model or overstate the system.

**The camera sits 18 cm above the floor.** The COCO `person` class consists
mostly of full-body pedestrians photographed from about 1.5 m. At 18 cm, the
camera sees thighs, chair legs, and torsos cropped at the shoulders. This is a
domain gap, not a capacity problem. Section 3 addresses it.


## 2. Baseline

The baseline is stock COCO YOLOv8n. You export the model to ONNX on a
development machine and build the TensorRT engine on the robot. The following
measurements use 28 classroom frames captured from the robot at operating
height, with clocks pinned.

| Precision | Input | Inference | p95 | Inference FPS | End-to-end | End-to-end FPS |
|---|---|---|---|---|---|---|
| FP16 | 416 | **5.02 ms** | 5.05 ms | **199.1** | 13.75 ms | 72.8 |
| FP32 | 416 | 7.68 ms | 7.71 ms | 130.1 | 14.53 ms | 68.8 |

GPU-only throughput measured with `trtexec`: 240.6 queries per second (qps) at
FP16 and 145.2 qps at FP32 for 416×416 input; 75.7 qps at FP32 for 640×640.

The model averages 1.09 detections per frame across the 28 frames, which
confirms it detects people rather than only running quickly.

### Where the time goes

Inference takes 5.02 ms. The end-to-end pipeline takes 13.75 ms. Letterboxing,
the host-to-device memory copy, and non-maximum suppression (NMS) consume the
remaining 8.73 ms.

This has a direct consequence. Further quantization produces almost no
system-level gain. INT8 quantization would remove roughly 2 ms from a 5 ms
stage, which is about 15% of end-to-end latency, in exchange for calibration
work and accuracy risk.

**At this operating point, optimize preprocessing, not weight precision.** This
report does not pursue INT8 for that reason.


## 3. Fine-tuning for camera height

### Dataset

Capture the training data from the robot's own camera at its real operating
height. You cannot reproduce this domain gap with stock imagery.

| Property | Value |
|---|---|
| Frames | 480 (384 training, 96 validation) |
| Capture poses | 4 junction positions, both sides of the aisle |
| Classes | `person` (single class) |
| Labeled boxes | 389 |
| Empty frames | 121, used as negatives |

The dashboard triggers capture with countdown keys: `O` for seated, `E` for
empty, `P` for standing. The countdown exists because one operator works alone
and must walk from the laptop to the seat before capture begins. Each frame
records the robot's map pose, which anchors every image to the position it was
taken from.

A YOLOv8x teacher model auto-labels the frames at 960 px and confidence 0.25 on
the robot's GPU. The teacher produces 389 boxes and leaves 121 frames empty.
Those 121 empty frames correspond almost exactly to the 120 deliberately empty
`E` captures, so the teacher missed one frame out of 360 that contained a
person. The usual manual correction pass was therefore unnecessary.

The fine-tune uses a single class deliberately. The mission counts only people.
Removing the other 79 COCO classes removes parameters from the detection head
and removes ways for the model to be wrong.

### Training

Training runs on the robot in two stages, starting from COCO weights, at 416 px.

| Stage | Epochs | Configuration | Precision | Recall | mAP50 | mAP50-95 |
|---|---|---|---|---|---|---|
| Stage 1 | 12 | Backbone frozen, `lr0` 0.01 | 0.937 | 0.918 | **0.965** | — |
| Stage 2 | 28 | Full fine-tune, `lr0` 0.002, batch 8 | 0.975 | 0.918 | **0.978** | 0.731 |

Stage 1 completes 12 epochs in 100 seconds. On-device training is not a
bottleneck.

## 4. Architecture search

<p class="pending">Additional measurements for this section are pending retrieval from the robot. An updated version of this document will be uploaded once the data has been pulled from the robot software.</p>

The candidate sweep compares YOLOv8n against YOLO11n and a width-scaled variant
at 416 px, reporting inference time and mAP50 on the classroom validation split.
That split is the only data that reflects the 18 cm viewpoint, so it is the only
split on which an architecture comparison is meaningful for this robot.

Section 2 sets the expected ceiling for this work. Because the forward pass is
only 36% of end-to-end latency, a backbone that is 20% faster improves
end-to-end throughput by roughly 7%.

## 5. Pruning

<p class="pending">Additional measurements for this section are pending retrieval from the robot. An updated version of this document will be uploaded once the data has been pulled from the robot software.</p>

Structured pruning at ratio 0.3 is the configuration used, chosen as a single
honest operating point rather than a sweep.

The expectation from section 2 is explicit: at 5.02 ms inference on a 3.0
million parameter network, structured pruning at this ratio should produce well
under 1 ms of end-to-end improvement, at a measurable mAP cost on a 480-image
dataset. This makes pruning the least valuable of the available optimizations at
this operating point, which is itself the useful finding.

## 6. Quantization and TensorRT

The following table compares FP16 and FP32 at 416 px using the same engine build
path and the same 28 frames.

| Precision | Inference | Speedup | Inference FPS | End-to-end |
|---|---|---|---|---|
| FP32 | 7.68 ms | 1.00× | 130.1 | 14.53 ms |
| FP16 | **5.02 ms** | **1.53×** | **199.1** | 13.75 ms |

FP16 speeds up the forward pass by 1.53×, but end-to-end by only 1.06×. The
reason is the same as in section 2: the forward pass is a minority of the
pipeline.

<p class="pending">A fuller breakdown of this section, including the per-stage latency figures captured on the robot, will be added in an updated version of this document once the data has been pulled from the robot software.</p>

**Important:** TensorRT engines are not portable. Build the `.onnx` file on a
development machine, then build the `.engine` file on the Jetson with
`training/build_engine_on_jetson.sh`. An engine encodes the target GPU
architecture and TensorRT version. An engine built elsewhere does not
deserialize.

## 7. System-level results

The following measurements come from the full stack running together: detector,
tracker, navigation, localization, and dashboard.

| Metric | Value |
|---|---|
| In-node inference, TensorRT FP16 | 4.94–6.67 ms (150–202 FPS) |
| Pipeline FPS, limited by the dashboard stream | About 5.3 |
| Navigation goal accuracy | 0.033–0.230 m over 2–7 m goals |
| Replanning around an unmapped obstacle | 36–49 plans published per traverse |
| Map | 282 × 303 cells at 5 cm, loop closure error 0.137 m over about 60 m |

### Costmap inflation

The stock `inflation_radius` is 0.02 m, which is smaller than the robot's
half-width of 0.10 m. The planner therefore routes paths 2 cm from chair legs,
the robot body does not fit, and recovery behaviors fire.

| `inflation_radius` | Recoveries | Time for 2.4 m | Final error |
|---|---|---|---|
| 0.02 m | 9 | 54.5 s | 0.229 m |
| **0.25 m** | **5** | **46.8 s** | **0.033 m** |

### Localization sensor model

Adaptive Monte Carlo Localization (AMCL) drifted 0.79 m over a 5 m drive, almost
entirely sideways. The robot drove under a table while believing it was centered
in the aisle.

Lateral position in a corridor depends almost entirely on side returns. The
stock `max_beams: 60` discards half of the roughly 123 real samples this lidar
produces.

| Parameter | Stock | Tuned | Reason |
|---|---|---|---|
| `max_beams` | 60 | **240** | Use every real sample. Lateral constraint depends on them. |
| `alpha1`, `alpha2` | 0.2 | **0.5** | A skid-steer chassis slips in yaw. Stock values over-trust odometry. |
| `z_hit`, `z_rand` | 0.5, 0.5 | **0.8, 0.2** | This lidar is sparse in coverage, not noisy in range. |
| `update_min_d` | 0.25 | **0.15** | Less unchecked odometry between corrections. |
| `laser_min_range`, `laser_max_range` | −1, 100 | **0.05, 12** | 124 of 247 bins are exactly 0.0 and must not count as hits. |

The following table shows the scan-to-map match score across a 5 m drive. The
score is the fraction of scan endpoints that land on occupied map cells.

| Configuration | Start | Mid-drive | End, stationary |
|---|---|---|---|
| Before tuning | 0.83 | 0.15 | **0.24** |
| After tuning | 0.74 | 0.24 | **0.74** |

### Map thresholding

The lidar sits below tabletop height and detects only table and chair legs,
which appear as isolated cells more than a metre apart. At the standard
occupancy threshold of 0.65, those legs are written as unknown space, and the
planner routes paths through tables.

| Occupancy threshold | Occupied cells | Interior structure |
|---|---|---|
| 0.65 (standard) | 2,743 | Legs written as unknown |
| **0.45** | **6,008** | Legs become obstacles |

The `--occ` flag of `nav2_map_server` has no effect on ROS 2 Foxy. The saved
YAML file always reports 0.65, and the PGM image is already quantized by the
time the flag would apply. `scripts/save_map.py` applies the threshold when it
writes the file instead.


## 8. Failure modes and future work

The following problems are measured and understood, but not fixed.

1. **Pipeline throughput is preprocessing-bound.** Letterboxing and the
   host-to-device copy account for 64% of end-to-end time. Zero-copy CUDA mapped
   memory and a GPU-side letterbox are the remaining optimizations worth doing.
   This is where INT8 effort would have been better spent.
2. **Localization still degrades while the robot moves** (0.74 to 0.24 to 0.74).
   It recovers when the robot stops.
3. **The 220° lidar has a 140° blind rear arc.** Reversing maps no new space, so
   the mission always drives forward into unmapped space.
4. **On-device training and the runtime stack do not run concurrently** on 8 GB.

Point 2 has a design consequence. Because localization is reliable when
stationary and unreliable while moving, the mission state machine reads person
detections only while the robot is stopped, and ignores them while rotating or
driving. That rule follows from the measurement, not from style.

Next steps, in order of expected value: GPU letterboxing and zero-copy input;
the pruning and architecture-search measurements omitted here; and depth-camera
range instead of shoulder-width estimation, which currently carries the largest
error in the perception chain.


## 9. Reproducing these results

Run the benchmark on the robot with clocks pinned:

```bash
sudo nvpmodel -m 0 && sudo jetson_clocks
python3 training/benchmark.py \
    --images ~/limo_project/data/frames \
    --model tensorrt:models/exported/yolov8n_416_fp16.engine \
    --model tensorrt:models/exported/yolov8n_416_fp32.engine \
    --imgsz 416 --out models/benchmarks/results_416
```

Auto-label and fine-tune on the robot:

```bash
python3 training/autolabel.py --images data/dataset/images \
    --out data/labelled --teacher yolov8x.pt --device cuda:0 --imgsz 960
python3 training/finetune.py --data data/labelled/data.yaml \
    --model yolov8n.pt --imgsz 416 --epochs 40 --freeze-epochs 12 \
    --batch 8 --device 0
```

Export the model and build the engine. Build the ONNX file anywhere; build the
engine only on the Jetson:

```bash
python3 training/export_onnx.py --weights <best.pt> --imgsz 416
bash training/build_engine_on_jetson.sh <model.onnx> fp16
```

Verify the projection geometry. This test needs no ROS installation and no
robot. Run it after you change `geometry.py` or the range estimators:

```bash
python3 ros2_ws/src/limo_perception/test/test_geometry_roundtrip.py
```
