cmake_minimum_required(VERSION 3.8)
project(unordered_task_planner)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(planner_interfaces REQUIRED)

add_executable(set_cover_node src/set_cover_node.cpp)
target_compile_features(set_cover_node PUBLIC cxx_std_17)
ament_target_dependencies(set_cover_node rclcpp planner_interfaces)

add_executable(dijkstra_node src/dijkstra_node.cpp)
target_compile_features(dijkstra_node PUBLIC cxx_std_17)
ament_target_dependencies(dijkstra_node rclcpp planner_interfaces)

install(TARGETS set_cover_node dijkstra_node DESTINATION lib/${PROJECT_NAME})
ament_package()
