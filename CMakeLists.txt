cmake_minimum_required(VERSION 3.8)
project(cone_mapper)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
)

add_library(cone_mapper src/cone_mapper.cpp)
ament_target_dependencies(
  cone_mapper
  rclcpp
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
)

add_executable(cone_mapper_node src/cone_mapper_node.cpp)
ament_target_dependencies(
  cone_mapper_node
  rclcpp
)

target_link_libraries(cone_mapper_node
  cone_mapper
)

install(TARGETS
  cone_mapper_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
