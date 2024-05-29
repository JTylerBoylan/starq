# C++ Development in Docker

### Setup for Windows
1. See [WSL](https://learn.microsoft.com/en-us/windows/wsl/install) for OS requirements
2. Open Windows Command Prompt
3. Install Ubuntu 22.04: $`wsl --install -d Ubuntu-22.04`
   - Takes ~10-20 minutes to install
   - Requires restart
4. Open Ubuntu from Start menu
5. Set username and password
6. Update: $`sudo apt update`
7. Install [Docker Desktop for Windows]((https://docs.docker.com/desktop/install/windows-install/))
   - Requires restart
8. Toggle Docker setting: `Settings > Resources > WSL integration > Ubuntu 22.04` then `Apply & Restart`
9. In Ubuntu, run: $`sudo usermod -aG docker $USER && newgrp docker`
10. Install display driver: $`sudo apt install x11-xserver-utils`
- Use Ubuntu terminal for all future steps

### Setup for Ubuntu
1. Install [Docker Engine for Linux](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository) using the apt repository
2. Run: $`sudo usermod -aG docker $USER && newgrp docker`

### Adding SSH Keys
SSH Keys are required to make changes to your Github repositories
1. [Check for existing keys](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/checking-for-existing-ssh-keys)
2. [Generate a new key on your local device](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent)
3. [Add new key to Github profile](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account)
4. [Test SSH key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/testing-your-ssh-connection)

### Workspace Installation
1. Open the Terminal
2. Go to user directoy: $`cd ~`
3. Clone the project into the `starq_ws` folder: $`git clone git@github.com:FAMU-FSU-STRIDe/starq starq_ws`

### Docker + VSCode
1. Go to the workspace folder: $`cd ~/starq_ws`
2. Build the development environment: $`./build_dev.sh`
   - Takes ~10-20 minutes to build
   - Only needs to be ran again if updates are made to the Dockerfile
3. Run the development enviroment: $`./run_dev.sh`
4. Open VSCode
5. Install `Dev Containers` Extension
5. Press the `F1` key > `Dev Containers: Attach to Running Container...` > `starq`
6. `File` > `Open Folder` > `/home/nvidia/starq_ws/src/`

### Building the Project

1. Open Terminal in the container
2. Go to `starq_ws` folder: $`cd ~/starq_ws`
3. Build: $`colcon build`
4. Source: $`source install/setup.bash`
   - Only if editing ROS2 run or launch files

- *Note: You can choose the build type using the command: `colcon build --cmake-args -DCMAKE_BUILD_TYPE=<Build Type>` where Build Type can be:*
  - *`Release`: Maximum performance*
  - *`Debug`: Slower, but with debugging information*
  - *`RelWDebInfo`: Mix of the other two*

### Creating an Executable
1. Create the file: `starq/tests/test_executable.cpp`
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
add_executable(test_executable tests/test_executable.cpp)
target_include_directories(test_executable PUBLIC include)
target_link_libraries(test_executable PUBLIC starqlib)
```
5. Open the Terminal
6. Go to `~/starq_ws`
7. Run the command: $`colcon build`
9. Run the executable: $`test_executable`

### Creating an Executable (ROS)
1. Create the file: `starq/tests/test_ros_executable.cpp`
2. Add the lines:
```
#include <stdio.h>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<rclcpp::Node>("test_node");

  RCLCPP_INFO(node->get_logger(), "Hello world!\n");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```
3. Open `starq/CMakeLists.txt`
4. Add the lines in the ROS executables section:
```
add_executable(test_ros_executable tests/test_ros_executable.cpp)
target_link_libraries(test_ros_executable PUBLIC starqlib)
```
5. Add executable to install
```
install(TARGETS 
  starq_node
  ... # Other ROS executables
  test_ros_executable
  DESTINATION lib/${PROJECT_NAME}
)
```
5. Open the Terminal
6. Go to `~/starq_ws`
7. Build: $`colcon build`
8. Source: $`source install/setup.bash`
9. Run the executable: $`ros2 run starq test_ros_executable`

## Creating a ROS Launch File
1. Create the file: `starq/launch/test_ros_launch.xml`
2. Add the lines:
```
<launch>

    <node pkg="starq" exec="test_ros_executable" name="test_ros_executable"  output="screen"/>

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