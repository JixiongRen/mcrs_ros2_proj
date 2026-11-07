# Shaft telemetry in SI. Zero values with valid=false are unavailable, not measurements.
int64 raw_position_count
float64 shaft_angle_rad
float64 measured_velocity_rad_s
float64 requested_velocity_rad_s
float64 accepted_velocity_rad_s
bool torque_enabled
uint32 hardware_error
uint64 sample_seq
uint64 sampled_ns
string device_epoch
bool valid
bool configured
float64 sample_age_s
string error_code
int32 watchdog_raw
