cmake_minimum_required(VERSION 3.8)
project(kpi_rover_worlds)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python module kpi_rover_worlds: world_info and the helpers of the world generators
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR common/python)

install(DIRECTORY
  worlds
  assets
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  common/xacro
  common/config
  DESTINATION share/${PROJECT_NAME}/common
)

ament_package()
