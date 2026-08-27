cmake_minimum_required(VERSION 3.8)
project(planner_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/synthetic_demo.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME synthetic_demo
)

ament_package()
