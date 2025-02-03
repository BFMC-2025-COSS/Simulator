cmake_minimum_required(VERSION 3.0.2)

project(node_to_path)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/graphml_to_path_node.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
