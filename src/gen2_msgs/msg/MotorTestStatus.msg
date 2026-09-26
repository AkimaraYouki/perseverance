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
float64 speed_rad_s          # position mode: speed limit
float64 target_rad           # position mode: absolute joint target
float64 position_error_rad   # position mode: target - final (after the test)
# accel mode results (after the test; + = joint positive direction)
float64 pulse_s
float64 accel_pos_rad_s2     # mean acceleration while +I
float64 accel_neg_rad_s2     # mean acceleration while -I (negative)
float64 accel_asymmetry_pct  # (|a+| - |a-|) / mean * 100
float64 reversal_ms          # current flip -> velocity crosses zero (mean)
float64 current_pos_a        # mean measured current while +I
float64 current_neg_a        # mean measured current while -I
float64 inertia_est_kgm2     # Kt*(|I+|+|I-|)/(|a+|+|a-|), friction included
float64 feedback_hz          # measured upload rate during the test
int32 pulses                 # full pulses used for the estimates
bool heartbeat_ok
# limits (per motor, same order as motor_names)
string[] motor_names
float64[] max_current_a
float64[] max_velocity_rad_s
float64 max_duration_s
float64 max_position_move_deg
