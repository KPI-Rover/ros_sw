cmake_minimum_required(VERSION 3.8)
project(kpi_rover_worlds)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
