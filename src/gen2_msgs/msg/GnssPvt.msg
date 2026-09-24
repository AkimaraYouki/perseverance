# Raw UBX-NAV-PVT epoch as forwarded by the ESP32 sensor hub (HostFrame v1).
# Published only when iTOW changes (one message per GNSS epoch).
std_msgs/Header header          # stamp = host time mapped from ESP32 receive time

uint32 itow_ms
uint16 year
uint8 month
uint8 day
uint8 hour
uint8 minute
uint8 second
uint8 valid_flags               # UBX valid: bit0 validDate, bit1 validTime, bit2 fullyResolved
uint32 t_acc_ns
int32 nano_ns

uint8 fix_type                  # 0 none, 1 DR, 2 2D, 3 3D, 4 GNSS+DR, 5 time only
uint8 fix_flags                 # UBX flags: bit0 gnssFixOK
bool gnss_fix_ok
uint8 num_sv

float64 latitude_deg
float64 longitude_deg
float64 height_ellipsoid_m
float64 height_msl_m
float64 h_acc_m
float64 v_acc_m

float64 vel_north_mps
float64 vel_east_mps
float64 vel_down_mps
float64 ground_speed_mps
float64 heading_of_motion_deg
float64 speed_acc_mps
float64 heading_acc_deg
float64 pdop

bool usable                     # hub gate: gnssFixOK && 3D && hAcc <= hub threshold
