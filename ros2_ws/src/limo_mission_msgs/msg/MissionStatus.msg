# Mission state machine telemetry. Also drives the dashboard's status strip.

std_msgs/Header header

string state                    # IDLE | LOCALIZING | PATROLLING | APPROACHING | DWELLING | RECOVERING | DONE | FAULT
string detail                   # human readable, shown verbatim on the dashboard

int32 current_waypoint          # -1 when not patrolling
int32 total_waypoints
int32 target_person_id          # -1 when not approaching anyone

float32 mission_elapsed_s
int32 nav_goals_sent
int32 nav_goals_failed
int32 replans                   # times we aborted and re-issued a goal
bool  emergency_stop            # true while perception is vetoing motion
