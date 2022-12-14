cmake_minimum_required(VERSION 3.10)
project(Artemis)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  genmsg
  message_generation
  actionlib_msgs
  actionlib
  move_base
  move_base_msgs
  fiducial_msgs
)

find_package(Boost REQUIRED COMPONENTS system thread)

add_service_files(
  FILES
  WMSTask.srv
)

add_action_files(
  DIRECTORY action
  FILES
  Task.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
  move_base_msgs
  fiducial_msgs
)

include_directories(
  include/Artemis
)

set(artemis_src
  src/wms_service_server.cpp
  src/wms_service_client.cpp
  src/task_action_server.cpp
  src/task_action_client.cpp
  src/move_base_action_wrapper.cpp
  src/navigator.cpp
  src/aruco_detector.cpp)

add_library(artemis_lib ${artemis_src})

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs actionlib_msgs message_runtime move_base_msgs move_base fiducial_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(artemis src/main.cpp)
target_link_libraries(artemis
  artemis_lib
  ${catkin_LIBRARIES})