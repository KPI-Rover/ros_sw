cmake_minimum_required(VERSION 3.8)
project(kpi_rover_description)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
