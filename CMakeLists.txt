cmake_minimum_required(VERSION 3.8)
project(kpi_rover)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17 for ROS2 compatibility
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

include_directories(
  include
)

# Example executable
add_executable(example_diff_drive executables/example_diff_drive.cpp)
ament_target_dependencies(example_diff_drive
  rclcpp
  geometry_msgs
)

# First create the ECU bridge library
add_library(ecu_bridge SHARED
  src/ecu_bridge/tcp_transport.cpp
  src/ecu_bridge/ecu_bridge.cpp
)

target_include_directories(ecu_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Add rclcpp dependency to ecu_bridge
ament_target_dependencies(ecu_bridge
  rclcpp
)

# Diff Drive Hardware Interface
add_library(kpi_rover_diff_drive_hw SHARED
  src/hardware_interfaces/kpi_rover_diff_drive_hw.cpp
)
ament_target_dependencies(kpi_rover_diff_drive_hw
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

target_include_directories(kpi_rover_diff_drive_hw PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(kpi_rover_diff_drive_hw
  pluginlib::pluginlib
  ecu_bridge  # Add ECU bridge library
)

pluginlib_export_plugin_description_file(hardware_interface plugin_description.xml)

# Build the test application
add_executable(test_ecu_bridge test/test_ecu_bridge.cpp src/ecu_bridge/tcp_transport.cpp src/ecu_bridge/ecu_bridge.cpp)
ament_target_dependencies(test_ecu_bridge rclcpp pluginlib)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install shared library
install(
  TARGETS kpi_rover_diff_drive_hw
  EXPORT export_kpi_rover_hw
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install example executable
install(
  TARGETS example_diff_drive
  DESTINATION lib/${PROJECT_NAME}
)

# Install test application
install(TARGETS
  test_ecu_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# Install plugin description
install(
  FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config description executables launch params worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install the ECU bridge library
install(
  TARGETS ecu_bridge
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Export package dependencies
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_export_include_directories(include)
ament_export_libraries(kpi_rover_diff_drive_hw)
ament_export_libraries(ecu_bridge)

ament_package()
