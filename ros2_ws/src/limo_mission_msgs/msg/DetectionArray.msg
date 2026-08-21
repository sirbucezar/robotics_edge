# All detections from one camera frame, plus the timing numbers the
# Model Optimization Report needs. Publishing latency on the same message as
# the detections means the dashboard, the rosbag and the report all quote the
# exact same numbers.

std_msgs/Header header          # stamp = camera frame acquisition time, frame_id = camera optical frame

Detection[] detections

uint32  image_width             # source image size the bboxes refer to
uint32  image_height

float32 preprocess_ms           # letterbox + normalise + HtoD copy
float32 inference_ms            # pure forward pass (this is the >=50 FPS number)
float32 postprocess_ms          # NMS + rescale
float32 pipeline_fps            # measured end-to-end rate of this node
string  backend                 # "mock" | "ultralytics" | "onnxruntime" | "tensorrt"
string  model_name              # e.g. "yolov8n_finetuned_int8"
