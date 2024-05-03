# Test Executables

## Jetson Test Executables

- Run from the `build/starq` directory on the Jetson AGX Orin

### 1. Test CAN Connection
- Source: `starq/tests/test_can_connection.cpp`
- Executable: $`./test_can_connection`
- This test makes sure the "can0" interface is UP and able to communicate messages
- It can be used to verify that the ODrives are successfully sending messages and/or to view the IDs of the CAN messages that are being recieved
- Troubleshooting:
  - Make sure there is a common ground connection between the CAN bus and the ODrive controllers. Otherwise, no messages will be recieved and/or the program will freeze.
  - Use $`ifconfig` on the host to see that the "can0" interface exists and is in state "UP"
  - If not, try running the $`./docs/loadcan_jetson.sh` script and try again

### 2. Test ODrive Control
- Source: `starq/tests/test_odrive_control.cpp`
- Executable: $`./test_odrive_control`
- This test is used for controlling a single ODrive controller. It should incrementally set the position to higher angles, and print the ODrive info at each position
- It can be used to verify that the ODrive is correctly positioning itself based on a given command
- Troubleshooting:
  - Make sure the ODrive is configured correctly and you are using the correct CAN ID and CAN socket interface
  - Use the `odrivetool` on the host with a USB connection to verify the ODrive can connect

### 3. Test Clear Errors
- Source: `starq/test/test_clear_errors.cpp`
- Executable: $`./test_clear_errors`
- This test is used to clear the errors on all connected ODrives
- Useful for resetting ODrive errors if one occurs

### 4. Test Five-bar 2D Position Control
- Source: `starq/test/test_fivebar2d_position_control.cpp`
- Executable: $`./test_fivebar2d_position_control`
- This test is used to set a fivebar-2d leg to a specific position. It should follow a circular trajectory
- Useful for testing the inverse kinematics of the `STARQFiveBar2DLegDynamics` class
- Troubleshooting:
  - Make sure the inverse kinematics is correctly reaching a solution (can be NaN if the foot position is out of range)
  - Make sure the link lengths given to the leg dynamics are accurate

### 5. Test Five-bar 2D Force Control
- Source: `starq/test/test_fivebar2d_force_control.cpp`
- Executable: $`./test_fivebar2d_force_control`
- This test is used to set a fivebar-2d leg to a specific force
- Useful for testing the Jacobian of the `STARQFiveBar2DLegDynamics` class, as well as measuring the losses due to friction in the gears
- Troubleshooting:
  - Make sure the Jacobian is reaching a solution (can be NaN at singular points)
  - Make sure the link lengths given to the leg dynamics are accurate

## STARQ Test Executables

### 1. Test STARQ Motors
- Source: `starq/test/test_starq_motors.cpp`
- Executable: $`./test_starq_motors`
- This test loops through all STARQ motors and prints their infos
- Useful for checking if there are any motors that are not communicating
- Troubleshooting:
  - Check the connection to the CAN bus
  - Use `odrivetool` with a USB connection to verify the controller is working correctly

### 2. Test STARQ Robot
- Source: `starq/test/test_starq_robot.cpp`
- Executable: $`./test_starq_robot`
- This test runs a simple feed-forward trajectory from a file on the STARQ platform
- You can also run a circle trajectory on all feet to test syncronization between the legs
- Useful for gait development

## MuJoCo Test Executables

### 1. Test MuJoCo
- Source: `starq/test/test_mujoco.cpp`
- Executable: $`./test_mujoco`
- Simplest script for starting a MuJoCo simulation with the base control function
- Troubleshooting:
  - [MuJoCo Docs](https://mujoco.readthedocs.io/en/stable/overview.html)

### 2. Test MuJoCo Controller
- Source: `starq/test/test_mujoco_controller.cpp`
- Executable: $`./test_mujoco_controller`
- Test the simulated PID control to keep the robot in a standing position

### 3. Test MuJoCo Leg Control
- Source: `starq/test/test_mujoco_leg_control.cpp`
- Executable: $`./test_mujoco_leg_control`

### 4. Test MuJoCo Leg Forces
- Source: `starq/test/test_mujoco_leg_forces.cpp`
- Executable: $`./test_mujoco_leg_forces`

### 5. Test MuJoCo Leg Trajectory
- Source: `starq/test/test_mujoco_leg_trajectory.cpp`
- Executable: $`./test_mujoco_leg_trajectory`

### 6. Test MuJoCo Localization
- Source: `starq/test/test_mujoco_localization.cpp`
- Executable: $`./test_mujoco_localization`

## MuJoCo MPC Test Executables

### 1. Test MuJoCo MPC Planner
- Source: `starq/test/test_mujoco_mpc_planner.cpp`
- Executable: $`./test_mujoco_mpc_planner`

### 2. Test MuJoCo MPC Solver
- Source: `starq/test/test_mujoco_mpc_solver.cpp`
- Executable: $`./test_mujoco_mpc_solver`

### 3. Test MuJoCo MPC Controller
- Source: `starq/test/test_mujoco_mpc_controller.cpp`
- Executable: $`./test_mujoco_mpc_controller`

### 4. Test Unitree A1 MuJoCo Robot
- Source: `starq/test/test_unitree_a1_mujoco_robot.cpp`
- Executable: $`./test_unitree_a1_mujoco_robot`

### 5. Test ROS2 Joystick MuJoCo
- Source: `starq/test/test_ros2_joystick_mujoco.cpp`
- Launch: $`ros2 launch starq test_joystick_mujoco.xml`

## MuJoCo SLAM Test Executables

### 1. Test MuJoco Camera
- Source: `starq/test/test_mujoco_camera.cpp`
- Executable: $`./test_mujoco_camera`

### 2. Test ROS2 Camera MuJoCo
- Source: `starq/test/test_ros2_camera_mujoco.cpp`
- Executable: $`ros2 launch starq test_camera_mujoco.xml`