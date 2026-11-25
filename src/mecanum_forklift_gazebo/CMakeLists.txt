cmake_minimum_required(VERSION 3.8)
project(mecanum_forklift_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch models worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/obstacle_mover.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
