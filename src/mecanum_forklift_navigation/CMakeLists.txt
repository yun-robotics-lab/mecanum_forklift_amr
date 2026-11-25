cmake_minimum_required(VERSION 3.8)
project(mecanum_forklift_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch maps rviz
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/make_nav_map.py DESTINATION lib/${PROJECT_NAME})

ament_package()
