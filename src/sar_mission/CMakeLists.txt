cmake_minimum_required(VERSION 3.16)
project(sar_mission)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(grid_mission_node
  src/grid_mission_node.cpp
  src/coverage_planner.cpp
)
target_include_directories(grid_mission_node PRIVATE src)
target_link_libraries(grid_mission_node PUBLIC
  rclcpp::rclcpp
  ${px4_msgs_TARGETS}
  ${nav_msgs_TARGETS}
)

install(TARGETS grid_mission_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
