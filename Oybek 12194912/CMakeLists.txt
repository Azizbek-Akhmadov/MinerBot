#This is a Cmake file 
cmake_minimum_required(VERSION 3.0.2)
project(turtlebot_navigation_and_mapping)
find_package(catkin
  REQUIRED
  COMPONENTS
  turtlesim
)

catkin_package(
  CATKIN_DEPENDS
  turtlesim
)
catkin_install_python(PROGRAMS
  nodes/explore.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES
  rviz/explore.rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
