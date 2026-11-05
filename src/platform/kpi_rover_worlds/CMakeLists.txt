cmake_minimum_required(VERSION 3.8)
project(kpi_rover_worlds)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python module kpi_rover_worlds: world_info
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR common/python)

install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
