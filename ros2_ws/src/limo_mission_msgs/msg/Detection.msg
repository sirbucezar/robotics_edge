# A single 2D detection in image coordinates.
# Deliberately self-contained: we do NOT depend on vision_msgs, because the
# ObjectHypothesis field names changed between Foxy and Humble and the robot
# runs Foxy. Owning the contract means the exam-day robot and your laptop
# can never disagree about the wire format.

string  label       # e.g. "person"
float32 score       # 0.0 - 1.0
int32   x           # bbox top-left, pixels
int32   y
int32   width       # pixels
int32   height
