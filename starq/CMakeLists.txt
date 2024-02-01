# Minimum CMake version and project name
cmake_minimum_required(VERSION 3.8)
project(starq)

# Compiler options for GCC and Clang to ensure code quality
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required dependencies for the project
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

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
  src/lib/can/can_socket.cpp
  src/lib/odrive/odrive_socket.cpp
  src/lib/odrive/odrive_controller.cpp
  src/lib/dynamics/starq_fivebar2d.cpp
  src/lib/leg_controller.cpp
  src/lib/leg_command_publisher.cpp
  src/lib/ros2/motor_controller.cpp
  src/lib/ros2/odrive_controller.cpp
)
target_include_directories(starqlib PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(starqlib PUBLIC rclcpp::rclcpp Eigen3::Eigen "${cpp_typesupport_target}")

# Export library for use in other packages
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake)

# Install the library and headers
install(TARGETS starqlib
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

# ROS executables
add_executable(starq_node src/starq_node.cpp)
target_link_libraries(starq_node starqlib)

# Install the ROS executables to the appropriate directory
install(TARGETS starq_node DESTINATION lib/${PROJECT_NAME})

# Example executables
add_executable(example_control examples/example_control.cpp)
target_include_directories(example_control PUBLIC include)
target_link_libraries(example_control PUBLIC stdc++ stdc++fs m pthread starqlib)

# Test executables
add_executable(test_can_connection tests/test_can_connection.cpp)
target_include_directories(test_can_connection PUBLIC include)
target_link_libraries(test_can_connection PUBLIC stdc++ stdc++fs m pthread starqlib)

add_executable(test_odrive_control tests/test_odrive_control.cpp)
target_include_directories(test_odrive_control PUBLIC include)
target_link_libraries(test_odrive_control PUBLIC stdc++ stdc++fs m pthread starqlib)

add_executable(test_fivebar2d_position_control tests/test_fivebar2d_position_control.cpp)
target_include_directories(test_fivebar2d_position_control PUBLIC include)
target_link_libraries(test_fivebar2d_position_control PUBLIC stdc++ stdc++fs m pthread starqlib)

add_executable(test_fivebar2d_force_control tests/test_fivebar2d_force_control.cpp)
target_include_directories(test_fivebar2d_force_control PUBLIC include)
target_link_libraries(test_fivebar2d_force_control PUBLIC stdc++ stdc++fs m pthread starqlib)

add_executable(test_fivebar2d_dummy tests/test_fivebar2d_dummy.cpp)
target_include_directories(test_fivebar2d_dummy PUBLIC include)
target_link_libraries(test_fivebar2d_dummy PUBLIC stdc++ stdc++fs m pthread starqlib)

add_executable(test_clear_errors tests/test_clear_errors.cpp)
target_include_directories(test_clear_errors PUBLIC include)
target_link_libraries(test_clear_errors PUBLIC stdc++ stdc++fs m pthread starqlib)

add_executable(test_command_publisher_dummy tests/test_command_publisher_dummy.cpp)
target_include_directories(test_command_publisher_dummy PUBLIC include)
target_link_libraries(test_command_publisher_dummy PUBLIC stdc++ stdc++fs m pthread starqlib)

# Code quality and testing settings
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  # Note: The following lines are for skipping specific linters. Uncomment if necessary.
  # skip copyright check linter
  # set(ament_cmake_copyright_FOUND TRUE)
  # skip cpplint (only applies if the package is in a git repo)
  # set(ament_cmake_cpplint_FOUND TRUE)
endif()

# Finalize the ament package
ament_package()
