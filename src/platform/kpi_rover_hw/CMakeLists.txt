cmake_minimum_required(VERSION 3.8)
project(kpi_rover_hw)

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
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
)

# ECU bridge library
add_library(ecu_bridge SHARED
  src/ecu_bridge/serial_transport.cpp
  src/ecu_bridge/ecu_protocol.cpp
  src/ecu_bridge/ecu_bridge.cpp
)

target_include_directories(ecu_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ecu_bridge
  rclcpp
)

# KPI Rover System Hardware Interface
add_library(kpi_rover_system_hw SHARED
  src/hardware_interfaces/kpi_rover_system_hw.cpp
)

target_include_directories(kpi_rover_system_hw PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(kpi_rover_system_hw
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

target_link_libraries(kpi_rover_system_hw
  pluginlib::pluginlib
  ecu_bridge
)

pluginlib_export_plugin_description_file(hardware_interface kpi_rover_plugin_description.xml)

# Test ECU Protocol stress test
add_executable(test_ecu_protocol test/test_ecu_protocol.cpp)
target_link_libraries(test_ecu_protocol ecu_bridge)
ament_target_dependencies(test_ecu_protocol rclcpp)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install shared library
install(
  TARGETS ecu_bridge kpi_rover_system_hw
  EXPORT export_kpi_rover_hw
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install test executable
install(
  TARGETS test_ecu_protocol
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES kpi_rover_plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(ecu_bridge kpi_rover_system_hw)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

ament_package()
