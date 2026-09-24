std_msgs/Header header
bool running
string motor
string mode
float64 value
float64 elapsed_s
float64 duration_s
string result                # "idle", "running", "done", or abort reason
uint64 test_id
float64 peak_current_a
float64 peak_velocity_rad_s
float64 moved_rad
float64 mean_velocity_rad_s  # second half of the test
bool heartbeat_ok
# limits (per motor, same order as motor_names)
string[] motor_names
float64[] max_current_a
float64[] max_velocity_rad_s
float64 max_duration_s
