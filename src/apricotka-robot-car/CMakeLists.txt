cmake_minimum_required(VERSION 3.8)
project(apricotka-robot-car)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)

install(
  DIRECTORY config description executables launch params worlds maps
  DESTINATION share/${PROJECT_NAME}
)

add_executable(move_and_turn_node executables/move_and_turn.cpp)
add_executable(rotate_node executables/rotate.cpp)
add_executable(move_node executables/move.cpp)
add_executable(twist_to_twist_stamped_node executables/twist_to_twist_stamped.cpp)
ament_target_dependencies(move_and_turn_node rclcpp geometry_msgs nav_msgs tf2 tf2_ros sensor_msgs)
ament_target_dependencies(rotate_node rclcpp geometry_msgs tf2 tf2_ros sensor_msgs)
ament_target_dependencies(move_node rclcpp geometry_msgs nav_msgs tf2 tf2_ros sensor_msgs)
ament_target_dependencies(twist_to_twist_stamped_node rclcpp geometry_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

## Install
install(
  TARGETS move_and_turn_node rotate_node move_node twist_to_twist_stamped_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
