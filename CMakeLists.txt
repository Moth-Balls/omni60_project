cmake_minimum_required(VERSION 3.8)
project(omni60_project)

cmake_policy(SET CMP0144 NEW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)

include_directories(
  launch
  include
  ${cv_bridge_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${pcl_ros_INCLUDE_DIRS} 
  ${tf2_ros_INCLUDE_DIRS}
)

add_executable(omni60_project src/main.cpp)

target_link_libraries(omni60_project 
  ${PCL_LIBRARIES}
)

ament_target_dependencies(omni60_project
  rclcpp
  cv_bridge
  sensor_msgs
  message_filters
  tf2_ros
  tf2_msgs
  tf2_geometry_msgs
  geometry_msgs
  OpenCV
  pcl_ros 
)

# Install target
install(TARGETS omni60_project
  DESTINATION lib/${PROJECT_NAME})

ament_package()
