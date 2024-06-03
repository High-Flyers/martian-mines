cmake_minimum_required(VERSION 3.0.2)
project(martian_mines)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES martian-mines
#  CATKIN_DEPENDS cv_bridge rospy sensor_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/detector.py
  src/precision_landing.py
  src/bbox_publisher.py
  src/detection.py
  src/trajectory_generator.py
  src/rviz_environment.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_python_setup()