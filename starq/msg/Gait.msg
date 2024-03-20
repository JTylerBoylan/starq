# Gait.msg

# FIRST SECTION: Gait Sequence
# Should range between 0-1000 milliseconds
# Should always include time 0 and 1000 as bounds
# Timing/frequency is adjusted by the velocity
# Footfall pattern cols should match the number of legs
# Footfall pattern rows should match the number of time steps
uint16[] footfall_timing
uint8[][] footfall_pattern

# SECOND SECTION: Gait Parameters
# Control mode: 0 = position, 1 = velocity
# Target position: x, y, z
# Target orientation: roll, pitch, yaw
# Target linear velocity: x, y, z
# Target angular velocity: roll, pitch, yaw
# Max linear velocity: x, y, z
# Max angular velocity: roll, pitch, yaw
uint8 control_mode
float64[3] target_position
float64[3] target_orientation
float64[3] target_linear_velocity
float64[3] target_angular_velocity
float64[3] max_linear_velocity
float64[3] max_angular_velocity

# THIRD SECTION: MPC Weights
# Position weights: x, y, z
# Orientaion weights: roll, pitch, yaw
# Linear velocity weights: x, y, z
# Angular velocity weights: roll, pitch, yaw
# Force weights: fx, fy, fz
float64[3] position_weights
float64[3] orientation_weights
float64[3] linear_velocity_weights
float64[3] angular_velocity_weights
float64[3] force_weights