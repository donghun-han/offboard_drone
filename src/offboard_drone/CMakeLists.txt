cmake_minimum_required(VERSION 2.8.3)
project(offboard_drone)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  std_msgs

)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
)

set (CMAKE_CXX_STANDARD 11)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  offboard_node
  src/offboard_node.cpp
)
add_library(${PROJECT_NAME}lib
  lib/offboard.cpp
)
target_link_libraries(
  offboard_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}lib
)