# gen2_control balance_node state (100 Hz). Wheels/legs [L, R]; signs as wbctrl.py.
std_msgs/Header header
string mode                     # disarmed | stand | balance | fault
string fault                    # reason when mode == fault (or the last refusal)
float64 pitch                   # asin(g_x) [rad], + = nose down
float64 roll                    # asin(g_y) [rad], + = left low
float64 th                      # pendulum angle used by the LQR (after balance-point adaptation)
float64 thd
float64 th_kin
float64 th_bias
float64 l_pend
float64 v                       # wheel-axle speed estimate [m/s]
float64 v_ref
float64 x_err
bool lifted
float64[4] act                  # wbctrl action
float64[2] h                    # leg joint values [m]
float64[2] h_target
float64[2] hip_tau_cmd          # [N·m], + = extend
float64[2] hip_cur_cmd          # [A] drive current sent
float64[2] wheel_tau_cmd        # [N·m], + = forward
float64[2] wheel_cur_cmd
float64 vx_cmd
float64 wz_cmd
float64 imu_age_ms
bool imu_shm                    # IMU read from shared memory (else DDS /imu/data)
float64 loop_dt_max_ms          # worst loop period over the last report
uint32 overruns                 # periods > 2x nominal since start
