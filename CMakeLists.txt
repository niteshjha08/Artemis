cmake_minimum_required(VERSION 3.10)
project(Artemis)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  genmsg
  actionlib_msgs
  actionlib
  move_base
  move_base_msgs
  fiducial_msgs
  aruco_msgs
  tf
  tf2
  tf2_ros
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
  aruco_msgs
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
  src/aruco_detector.cpp
  src/pick_and_place.cpp)
  

# add_library(artemis_lib ${artemis_src})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs actionlib_msgs message_runtime move_base_msgs move_base fiducial_msgs aruco_msgs tf tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(wms_server_node src/wms_server_node.cpp ${artemis_src})
target_link_libraries(wms_server_node
  # artemis_lib
  ${catkin_LIBRARIES})
add_dependencies(wms_server_node Artemis_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(task_server_node src/task_server_node.cpp ${artemis_src})
target_link_libraries(task_server_node
  # artemis_lib
  ${catkin_LIBRARIES})
add_dependencies(task_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(artemis_node src/artemis_node.cpp ${artemis_src})
target_link_libraries(artemis_node
  # artemis_lib
  ${catkin_LIBRARIES})

add_dependencies(artemis_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(artemis_test test/artemis_test.test test/artemis_test.cpp ${artemis_src})
  target_link_libraries(artemis_test ${catkin_LIBRARIES})
  add_dependencies(artemis_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endif()

