# One actuator's latest feedback, converted to joint-side SI units using gen2 motor config.
std_msgs/Header header          # stamp = host CAN RX time (SocketCAN timestamp)

string name
uint8 can_id

float64 position_rad            # joint side, direction and offset applied
float64 velocity_rad_s          # joint side
float64 current_a               # motor q-axis current as reported
float64 torque_nm               # current_a * kt_nm_per_a * direction (estimate)
float64 kt_nm_per_a             # output-side Kt used for torque_nm (NaN = unknown)
float64 temperature_c           # driver board temperature (CubeMars servo upload)
uint8 error_code
string error_text

float64 raw_position_deg        # as reported by the drive
float64 raw_speed_erpm

bool stale                      # no feedback within stale timeout
float64 age_s                   # time since last feedback
uint64 rx_count
