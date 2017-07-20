cmake_minimum_required(VERSION 2.8.3)
project(bebop_simple_test)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  nav_msgs
  parrot_arsdk
  message_generation
)

add_message_files(FILES Altitude.msg)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})
#add_dependencies(bebop_test_src ${catkin_EXPORTED_TARGETS})
add_executable(bebop_test src/bebop_simple_test.cpp src/PID_ROS.cpp src/ArcDrone.cpp)
target_link_libraries(bebop_test ${catkin_LIBRARIES})

