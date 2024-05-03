# STARQ C++ Library

## Installation
1. Open the Terminal
2. Go to user directoy: $`cd ~`
3. Clone the project into the `starq_ws` folder: $`git clone https://github.com/JTylerBoylan/starq starq_ws`

## Docker + VSCode
1. Go to the workspace folder: $`cd ~/starq_ws`
2. Build the development environment *(if needed)*: $`./build_dev.sh`
3. Run the development enviroment: $`./run_dev.sh`
4. Open VSCode
5. Press the `F1` key > `Dev Containers: Attach to Running Container...` > `starq`
6. `File` > `Open Folder` > `/home/nvidia/starq_ws/src/`

## Building
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

## MATLAB

Several convienence MATLAB scripts can be found in the `matlab` folder. Includes:
- Script for calculating jacobian of the five-bar leg
- Script for calculating forward kinematics and jacobian of the Unitree A1 robot leg
- Script for converting legacy GUI foot trajectories into a correctly formatted trajectory file
- Script to generate a square foot trajectory, which is useful for verifying inverse kinematic equations