# Minimum CMake version and project name
cmake_minimum_required(VERSION 3.8)
project(starq)

# Set C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)

# Set OpenGL preference to GLVND
set(OpenGL_GL_PREFERENCE "GLVND")

# Compiler options for GCC and Clang to ensure code quality
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Core dependencies
find_package(Eigen3 REQUIRED)

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Mujoco dependencies
find_package(mujoco REQUIRED)
find_package(glfw3 3.3 REQUIRED)

# OSQP dependencies
find_package(osqp REQUIRED)

# Generate ROS interfaces from message definitions
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorCommand.msg"
  "msg/MotorState.msg"
  "msg/ODriveInfo.msg"
  "msg/LegCommand.msg"
  "msg/LegState.msg"
  DEPENDENCIES geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Include directories for Eigen and project-specific headers
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
include_directories(include)

# Define and build the main library for the project
add_library(starqlib SHARED
  src/lib/leg_controller.cpp
  src/lib/leg_command_publisher.cpp
  src/lib/trajectory_file_reader.cpp 
  src/lib/trajectory_publisher.cpp
  src/lib/robot.cpp
  src/lib/can/can_socket.cpp
  src/lib/odrive/odrive_socket.cpp
  src/lib/odrive/odrive_controller.cpp
  src/lib/starq/starq_fivebar2d_leg_dynamics.cpp
  src/lib/unitree/unitree_a1_leg_dynamics.cpp
  src/lib/unitree/unitree_a1_robot_parameters.cpp
  src/lib/unitree/unitree_a1_mujoco_robot.cpp
  src/lib/mpc/gait.cpp 
  src/lib/mpc/gait_sequencer.cpp 
  src/lib/mpc/reference_planner.cpp 
  src/lib/mpc/foothold_planner.cpp
  src/lib/mpc/mpc_configuration.cpp 
  src/lib/mpc/mpc_problem.cpp
  src/lib/mpc/qp_problem.cpp
  src/lib/mpc/mpc_controller.cpp
  src/lib/ros2/motor_controller.cpp
  src/lib/ros2/odrive_controller.cpp
  src/lib/ros2/leg_controller.cpp
  src/lib/ros2/leg_command_publisher.cpp
  src/lib/ros2/ros2_joystick.cpp
  src/lib/mujoco/mujoco.cpp
  src/lib/mujoco/mujoco_controller.cpp
  src/lib/mujoco/mujoco_localization.cpp
  src/lib/mujoco/mujoco_robot.cpp
  src/lib/osqp/osqp.cpp
)

# Link the library to the dependencies
ament_target_dependencies(starqlib
  rclcpp
  sensor_msgs
)

target_link_libraries(starqlib 
  stdc++ stdc++fs m pthread 
  Eigen3::Eigen 
  glfw 
  mujoco::mujoco 
  osqp::osqp 
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# Include directories for the library
target_include_directories(starqlib PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Export library for use in other packages
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake rclcpp geometry_msgs sensor_msgs)

# Install the library and headers
install(TARGETS starqlib
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

# Example executables
add_executable(example_control examples/example_control.cpp)
target_include_directories(example_control PUBLIC include)
target_link_libraries(example_control PUBLIC starqlib)

add_executable(two_leg_control examples/two_leg_control.cpp)
target_include_directories(two_leg_control PUBLIC include)
target_link_libraries(two_leg_control PUBLIC starqlib)

# Test executables
add_executable(test_can_connection tests/test_can_connection.cpp)
target_include_directories(test_can_connection PUBLIC include)
target_link_libraries(test_can_connection PUBLIC starqlib)

add_executable(test_odrive_control tests/test_odrive_control.cpp)
target_include_directories(test_odrive_control PUBLIC include)
target_link_libraries(test_odrive_control PUBLIC starqlib)

add_executable(test_fivebar2d_position_control tests/test_fivebar2d_position_control.cpp)
target_include_directories(test_fivebar2d_position_control PUBLIC include)
target_link_libraries(test_fivebar2d_position_control PUBLIC starqlib)

add_executable(test_fivebar2d_force_control tests/test_fivebar2d_force_control.cpp)
target_include_directories(test_fivebar2d_force_control PUBLIC include)
target_link_libraries(test_fivebar2d_force_control PUBLIC starqlib)

add_executable(test_fivebar2d_dummy tests/test_fivebar2d_dummy.cpp)
target_include_directories(test_fivebar2d_dummy PUBLIC include)
target_link_libraries(test_fivebar2d_dummy PUBLIC starqlib)

add_executable(test_clear_errors tests/test_clear_errors.cpp)
target_include_directories(test_clear_errors PUBLIC include)
target_link_libraries(test_clear_errors PUBLIC starqlib)

add_executable(test_command_publisher_dummy tests/test_command_publisher_dummy.cpp)
target_include_directories(test_command_publisher_dummy PUBLIC include)
target_link_libraries(test_command_publisher_dummy PUBLIC starqlib)

# ROS executables
add_executable(starq_node src/starq_node.cpp)
target_link_libraries(starq_node starqlib)

add_executable(test_ros2_joystick_mujoco tests/test_ros2_joystick_mujoco.cpp)
target_link_libraries(test_ros2_joystick_mujoco starqlib)

# Install the ROS executables to the appropriate directory
install(TARGETS 
  starq_node
  test_ros2_joystick_mujoco
  DESTINATION lib/${PROJECT_NAME}
)

# MuJoCo executables
add_executable(test_mujoco tests/test_mujoco.cpp)
target_include_directories(test_mujoco PUBLIC include)
target_link_libraries(test_mujoco PUBLIC starqlib)

add_executable(test_mujoco_controller tests/test_mujoco_controller.cpp)
target_include_directories(test_mujoco_controller PUBLIC include)
target_link_libraries(test_mujoco_controller PUBLIC starqlib)

add_executable(test_mujoco_leg_control tests/test_mujoco_leg_control.cpp)
target_include_directories(test_mujoco_leg_control PUBLIC include)
target_link_libraries(test_mujoco_leg_control PUBLIC starqlib)

add_executable(test_mujoco_leg_trajectory tests/test_mujoco_leg_trajectory.cpp)
target_include_directories(test_mujoco_leg_trajectory PUBLIC include)
target_link_libraries(test_mujoco_leg_trajectory PUBLIC starqlib)

add_executable(test_mujoco_leg_forces tests/test_mujoco_leg_forces.cpp)
target_include_directories(test_mujoco_leg_forces PUBLIC include)
target_link_libraries(test_mujoco_leg_forces PUBLIC starqlib)

add_executable(test_mujoco_localization tests/test_mujoco_localization.cpp)
target_include_directories(test_mujoco_localization PUBLIC include)
target_link_libraries(test_mujoco_localization PUBLIC starqlib)

add_executable(test_unitree_a1_mujoco_robot tests/test_unitree_a1_mujoco_robot.cpp)
target_include_directories(test_unitree_a1_mujoco_robot PUBLIC include)
target_link_libraries(test_unitree_a1_mujoco_robot PUBLIC starqlib)

add_executable(test_mujoco_mpc_planner tests/test_mujoco_mpc_planner.cpp)
target_include_directories(test_mujoco_mpc_planner PUBLIC include)
target_link_libraries(test_mujoco_mpc_planner PUBLIC starqlib)

add_executable(test_mujoco_mpc_solver tests/test_mujoco_mpc_solver.cpp)
target_include_directories(test_mujoco_mpc_solver PUBLIC include)
target_link_libraries(test_mujoco_mpc_solver PUBLIC starqlib)

add_executable(test_mujoco_mpc_controller tests/test_mujoco_mpc_controller.cpp)
target_include_directories(test_mujoco_mpc_controller PUBLIC include)
target_link_libraries(test_mujoco_mpc_controller PUBLIC starqlib)

# Finalize the ament package
ament_package()