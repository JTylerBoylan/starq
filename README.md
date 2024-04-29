# STARQ Documentation
*Updated 02/01/2023*

## NVIDIA Jetson AGX Orin 64GB Developer Kit

### NVIDIA Store
https://store.nvidia.com/en-us/jetson/store/

### Developer Guide
https://docs.nvidia.com/jetson/archives/r35.1/DeveloperGuide/index.html

### Login
```
username: nvidia
password: nvidia
```

## Motor Controllers

### ODrive Shop
https://odriverobotics.com/shopfolder

### ODrive Documentation
https://docs.odriverobotics.com/v/latest/guides/getting-started.html

### ODrive Pro Data Sheet
https://docs.odriverobotics.com/v/latest/hardware/pro-datasheet.html

### MJ5208 Moteus Motor Shop
https://mjbots.com/products/mj5208

### NTCLE300E3502SB Thermistor Shop
https://www.mouser.com/ProductDetail/Vishay-BC-Components/NTCLE300E3502SB?qs=%2FWiulJ9oly5IYkswf0Y9eA%3D%3D

### Thermistor Data Sheet
Data Sheet: https://www.vishay.com/doc?29051

### MJ5208 ODrive Auto-Configuration Script
1. Plug in USB from the Jetson to the ODrive controller 
2. Open the Terminal on the the Jetson
3. Go to the `docs` folder: $`cd ~/starq_ws/docs`
4. Run the auto-configuration script: $`python3 configure_odrive.py <CAN_ID>`
5. Let the script complete before unplugging

*Note: This script is for ODrive firmware v0.6.8*

## ODrive CAN Communication

### ODrive CAN Guide
https://docs.odriverobotics.com/v/latest/guides/can-guide.html \
Instructions on how to set up a CAN network.

### ODrive CAN Protocol
https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#can-protocol \
CAN frame IDs and required data for the different ODrive commands.

### ODrive Pinout
https://docs.odriverobotics.com/v/latest/hardware/pro-datasheet.html#pro-pinout \
Locations of the CAN, thermistor, and other IO pins.

## Jetson AGX Orin CAN

### Waveshare SN65HVD230 CAN Board
https://www.amazon.com/SN65HVD230-CAN-Board-Communication-Development/dp/B00KM6XMXO \
Converts CAN frames from GPIO to High/Low signals on the CAN bus.

### Jetson GPIO Pinout
https://jetsonhacks.com/nvidia-jetson-agx-orin-gpio-header-pinout/ \
Useful for determining where the 5V, 3.3V, GND, and CAN pins are on the GPIO array.

### Jetson CAN Documentation
https://docs.nvidia.com/jetson/archives/r35.4.1/DeveloperGuide/text/HR/ControllerAreaNetworkCan.html \
Instructions on how to enable CAN on the Jetson.

### Jetson CAN GPIO Pins

| CAN0 RX | CAN0 TX | CAN1 RX | CAN1 TX | 3.3V |     GND     |
| ------- | ------- | ------- | ------- | ---- | ----------- |
|   29    |   31    |    37   |    33   | 1/17 | 25/30/34/39 |

### Jetson CAN Setup
1. Open the Terminal on the Jetson
2. Go to the `docs` folder: $`cd ~/starq_ws/docs`
3. Run the command: $`sudo ./loadcan_jetson.sh` \
*Note: This is run automatically with `./run_dev.sh`.*

## C++ Quick Notes

### Overview
- C++ is used on this project over other languages for it's performance and embedded programming capabilities
- C++ is an object-oriented programming language, meaning that the code is organized into 'objects', or classes, with specific purposes and functions

### Pointers
- The objects in the code are usually single instance, meaning they only exist once in memory, and are shared to other objects using its pointer, or memory address
- Typically, using pointers requires allocating and deallocating space in memory (using `free` and `delete`), but it makes the code prone to memory-leaks (when memory is repeatedly allocated, but never de-allocated), which fills up the memory with useless objects
- To combat this, we use the STL's (C++ Standard Library) implementation of 'special pointers', namely `shared_ptr`, which automatically de-allocates the memory when there are no instances left
- We use the method `std::make_shared<ObjectType>(param1, param2, ...)` to construct a new `shared_ptr` of type `ObjectType` with a constructor of parameters `param1`, `param2`, `...`
- In the code, the shortcut `ObjectType::Ptr` is defined to be used instead of `shared_ptr<ObjectType>`

### Abstraction
- Object-oriented code also allows for the use of abstracted classes, or outlines, that other classes can derive from
- This is useful if there is some common structure for a set of objects used by another object, but each with their own implementation
- For example, we have a `MotorController` abstract class that defines the set of functions for all motor controller types and is used by the `LegController` class to control the motors, but the actual method of control is determined by how the `MotorController` class functions are overwritten by say the `ODriveController` or `MuJoCoController` classes, both of which are motor controller instances
- This enables us to have custom definitions/implementations for certain parts of the code, without having to edit the code at a higher level

## STARQ C++ Library

### Installation
1. Open the Terminal
2. Go to user directoy: $`cd ~`
3. Clone the project into the `starq_ws` folder: $`git clone https://github.com/JTylerBoylan/starq starq_ws`

### Docker + VSCode
1. Go to the workspace folder: $`cd ~/starq_ws`
2. Build the development environment *(if needed)*: $`./build_dev.sh`
3. Run the development enviroment: $`./run_dev.sh`
4. Open VSCode
5. Press the `F1` key > `Dev Containers: Attach to Running Container...` > `starq`
6. `File` > `Open Folder` > `/home/nvidia/starq_ws/src/`

### Building
1. Open Terminal in the container
2. Go to `starq_ws` folder: $`cd ~/starq_ws`
3. Build: $`colcon build`
4. Source: $`source install/setup.bash`
4. Go to the build folder: $`cd build/starq`
5. Run $`ls` to see all test executables

*Note: You can choose the build type using the command: `colcon build --cmake-args -DCMAKE_BUILD_TYPE=<Build Type>` where Build Type can be:*
  - *`Release`: Maximum performance*
  - *`Debug`: Slower, but with debugging information*
  - *`RelWDebInfo`: Mix of the other two*

## Jetson Test Executables

- Run from the `build/starq` directory on the Jetson AGX Orin

### 1. Test CAN Communication
- Source: `starq/tests/test_can_communication.cpp`
- Executable: $`./test_can_communication`
- This test makes sure the "can0" interface is UP and able to communicate messages
- It can be used to verify that the ODrives are successfully sending messages and/or to view the IDs of the CAN messages that are being recieved
- Troubleshooting:
  - Use $`ifconfig` on the host to see that the "can0" interface exists and is in state "UP"
  - If not, try running the $`./docs/loadcan_jetson.sh` script and try again

### 2. Test ODrive Control
- Source: `starq/tests/test_odrive_control.cpp`
- Executable: $`./test_odrive_control`
- This test is used for controlling a single ODrive controller. It should incrementally set the position to higher angles, and print the ODrive info at each position
- It can be used to verify that the ODrive is correctly positioning itself based on a given command
- Troubleshooting:
  - Make sure the ODrive is configured correctly and you are using the correct CAN ID
  - Use the `odrivetool` on the host with a USB connection to verify the ODrive can connect
  - Use an oscilliscope on the CAN wires to verify that messages are being transfered

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

## Creating Executables

### Creating an Executable (Not in ROS)
1. Create the file: `starq/tests/my_executable.cpp`
2. Add the lines:
```
#include <stdio.h>

int main()
{
    printf("Hello World!\n");

    return 0;
}
```
3. Open `starq/CMakeLists.txt`
4. Add the lines near the bottom:
```
add_executable(my_executable tests/my_executable.cpp)
target_include_directories(my_executable PUBLIC include)
target_link_libraries(my_executable PUBLIC starqlib)
```
- *Replace `my_executable` with your executable name*
5. Open the Terminal
6. Go to `~/starq_ws`
7. Run the command: $`colcon build`
8. Go to the executable location: $`cd build/starq`
9. Run the executable: $`./my_executable`

### Creating an Executable (In ROS)
1. Create the file: `starq/tests/my_ros_executable.cpp`
2. Add the lines:
```
#include <stdio.h>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  printf("Hello World!\n");

  rclcpp::init(argc, argv);
  
  rclcpp::Node node;
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
```
3. Open `starq/CMakeLists.txt`
4. Add the lines in the ROS executables section:
```
add_executable(my_executable tests/my_ros_executable.cpp)
target_link_libraries(my_ros_executable PUBLIC starqlib)
```
5. Add executable to install
```
install(TARGETS 
  starq_node
  ... # Other ROS executables
  my_ros_executable
  DESTINATION lib/${PROJECT_NAME}
)
```
5. Open the Terminal
6. Go to `~/starq_ws`
7. Build: $`colcon build`
8. Source: $`source install/setup.bash`
9. Run the executable: $`ros2 run starq my_ros_executable`

## Creating a ROS Launch File
1. Create the file: `starq/launch/my_ros_launch.xml`
2. Add the lines:
```
<launch>

    <node pkg="starq" exec="my_ros_executable" name="my_ros_executable"  output="screen"/>

    <node pkg="rviz2" exec="rviz2" name="rviz2" output="screen"/>

</launch>
```

## Debugging in VSCode
1. Add breakout points in the code by selecting the red dot next to the line number
2. Build in `Debug` mode (See `Building` section)
3. Open the `.vscode/launch.json` file in the editor
4. On the line for `program`, edit the path to the executable you want to debug
5. Go to the debugging panel in VSCode (bug + play arrow icon)
6. Press the green play arrow on `Launch Debugger`
7. Use the debug panel to see local variable values
8. Use the debug button window to continue to the next breakout point
