# Utility Commands

### starq_clear_errors
- Clear errors on all ODrives

### starq_zero_motors
- Send all motors to zero position
- Zero position is the position when the motors are first powered
- Reset zero by toggling power to the motors

### starq_motor_info
- Print all ODrive info (encoder estimates, currents, temperatures, etc.)

### starq_set_gains \<p> \<v> \<vi>
- Set gains of all ODrive motor
- See [ODrive Tuning Guide](https://docs.odriverobotics.com/v/latest/guides/tuning.html)
- `<p>`: Position gain *(Required)*
- `<v>`: Velocity gain *(Required)*
- `<vi>`: Velocity integrator gain *(Required)*
- Recommended (wall powered): `20 0.05 0.1`
- Recommended (battery): `100 0.15 0.3`
- *If you get a DC undervoltage error, lower the gains*
- *If the robot can't support itself, raise the gains*

### starq_run_trajectory \<file> \<freq> \<loops>
- Run a trajectory from a file
- `<file>`: Trajectory file name
   - *Relative to ~/starq_ws/src/starq/trajectories*
- `<freq>`: Loop frequency [Hz] *(default: 1.0)*
- `<loops>`: Number of loops *(default: 1)*


### [TODO] starq_set_limits \<v_lim> \<i_lim>

### [TODO] starq_run_circle \<radius> \<freq> \<loops> \<res=100>
