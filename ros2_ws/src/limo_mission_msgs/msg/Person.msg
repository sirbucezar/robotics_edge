# One tracked person, resolved into the map frame so the count is
# position-based and therefore stable across viewpoints.

int32 id                                # stable track id, monotonically assigned
geometry_msgs/PoseStamped pose          # frame_id is normally "map"
float32 confidence                      # running max detection score
int32 observation_count                 # how many frames supported this track
builtin_interfaces/Time first_seen
builtin_interfaces/Time last_seen

bool visited                            # mission has driven to it and dwelled
float32 distance_to_robot               # metres, at time of publish

uint8 STATE_CANDIDATE = 0               # seen too few times to trust
uint8 STATE_CONFIRMED = 1               # counted
uint8 STATE_STALE     = 2               # not seen recently, kept for the count
uint8 state
