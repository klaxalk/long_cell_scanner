cmake_minimum_required(VERSION 2.8.3)
project(long_cell_scanner)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_runtime
  rospy
  roscpp
  std_msgs
  cmake_modules
  geometry_msgs
  sensor_msgs
  rospix
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gdax_interface
   CATKIN_DEPENDS roscpp std_msgs geometry_msgs rospy std_msgs message_runtime rospix
)
