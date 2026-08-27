cmake_minimum_required(VERSION 3.8)
project(grid_motion_planner)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(planner_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(astar_node src/astar_node.cpp)
target_compile_features(astar_node PUBLIC cxx_std_17)
ament_target_dependencies(astar_node geometry_msgs nav_msgs planner_interfaces rclcpp)

install(TARGETS astar_node DESTINATION lib/${PROJECT_NAME})
ament_package()
