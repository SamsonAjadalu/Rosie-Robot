cmake_minimum_required(VERSION 3.10.2)
project(rosie_description)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

install(
  DIRECTORY
    urdf
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
