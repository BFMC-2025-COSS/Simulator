cmake_minimum_required(VERSION 3.0.2)

project(node_to_path)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
)

add_message_files(
  FILES
  DottedPose.msg
  PathWithDotted.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
        CATKIN_DEPENDS message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})

install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/graphml_to_path_node.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
