cmake_minimum_required(VERSION 3.0.2)
project(martian_mines)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
  vision_msgs
  message_generation
)


add_message_files(
  FILES
  FigureMsg.msg
  FigureMsgList.msg
  BoundingBoxLabeled.msg
  BoundingBoxLabeledList.msg
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/figure_finder.py
  src/precision_landing.py
  src/bbox_publisher.py
  src/detection.py
  src/trajectory_generator.py
  src/report_uploader.py
  src/environment_visualization.py
  src/trajectory_tracker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  vision_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES martian-mines
  CATKIN_DEPENDS message_runtime
#  CATKIN_DEPENDS cv_bridge rospy sensor_msgs
#  DEPENDS system_lib
)