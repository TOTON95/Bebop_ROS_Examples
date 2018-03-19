cmake_minimum_required(VERSION 2.8.3)
project(bebop_color_follower)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
)
## OpenCV package
find_package(OpenCV REQUIRED)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
## OpenCV directories
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(bebop_color_follower src/bebop_color_follower.cpp)
target_link_libraries(bebop_color_follower ${catkin_LIBRARIES})
target_link_libraries(bebop_color_follower ${OpenCV_LIBRARIES})


