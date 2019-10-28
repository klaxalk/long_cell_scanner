cmake_minimum_required(VERSION 2.8.3)
project(long_cell_scanner)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_runtime
  message_generation
  rospy
  roscpp
  std_msgs
  cmake_modules
  geometry_msgs
  sensor_msgs
  rospix
  rospix_utils
  gclib_ros
)

add_service_files(FILES
  SetString.srv
  )

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
  )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gdax_interface
   CATKIN_DEPENDS roscpp std_msgs geometry_msgs rospy std_msgs message_runtime rospix rospix_utils message_generation gclib_ros
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

# image ascii saver
add_executable(saver src/saver.cpp)
add_dependencies(saver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(saver
  ${catkin_LIBRARIES}
)
