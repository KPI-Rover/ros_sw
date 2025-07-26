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
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
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
add_library(ecu_bridge_motors SHARED
  src/ecu_bridge/tcp_transport.cpp
  src/ecu_bridge/ecu_bridge_motors.cpp
)

add_library(ecu_bridge_imu SHARED
  src/ecu_bridge/udp_transport.cpp
  src/ecu_bridge/ecu_bridge_imu.cpp
)

target_include_directories(ecu_bridge_motors PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Add rclcpp dependency to ecu_bridge
ament_target_dependencies(ecu_bridge_motors
  rclcpp
)

ament_target_dependencies(ecu_bridge_imu
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
  ecu_bridge_motors  # Add ECU bridge library
)

pluginlib_export_plugin_description_file(hardware_interface diff_drive_plugin_description.xml)

# Build the test application
add_executable(test_ecu_bridge_motors test/test_ecu_bridge_motors.cpp src/ecu_bridge/tcp_transport.cpp src/ecu_bridge/ecu_bridge_motors.cpp)
ament_target_dependencies(test_ecu_bridge_motors rclcpp pluginlib)

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


# IMU Hardware Interface
add_library(kpi_rover_imu_hw SHARED
  src/hardware_interfaces/kpi_rover_imu_hw.cpp
)
ament_target_dependencies(kpi_rover_imu_hw
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

target_include_directories(kpi_rover_imu_hw PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(kpi_rover_imu_hw
  pluginlib::pluginlib
  ecu_bridge_imu  # Add ECU bridge library
)

pluginlib_export_plugin_description_file(hardware_interface imu_plugin_description.xml)

# Build the test application
add_executable(test_ecu_bridge_imu test/test_ecu_bridge_imu.cpp src/ecu_bridge/udp_transport.cpp src/ecu_bridge/ecu_bridge_imu.cpp)
ament_target_dependencies(test_ecu_bridge_imu rclcpp pluginlib)

# Install shared library
install(
  TARGETS kpi_rover_imu_hw
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
  test_ecu_bridge_motors
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  test_ecu_bridge_imu
  DESTINATION lib/${PROJECT_NAME}
)

# Install plugin description
install(
  FILES diff_drive_plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES imu_plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config description executables launch params worlds maps
  DESTINATION share/${PROJECT_NAME}
)

# Install the ECU bridge library
install(
  TARGETS ecu_bridge_motors
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS ecu_bridge_imu
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(PROGRAMS
  scripts/yuv422_compressed_to_rgb8_converter.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/object_detector/object_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

# Export package dependencies
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_export_include_directories(include)
ament_export_libraries(kpi_rover_imu_hw)
ament_export_libraries(ecu_bridge_motors)
ament_export_libraries(ecu_bridge_imu)

ament_package()
