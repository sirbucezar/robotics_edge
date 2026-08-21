# The authoritative people state. This is what the dashboard renders and
# what the mission node plans against.

std_msgs/Header header

Person[] people

int32 confirmed_count       # people counted (the number on the dashboard)
int32 candidate_count       # tracks not yet confirmed
int32 visited_count         # confirmed people the robot has driven to
