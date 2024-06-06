#!/bin/bash

cd $ROS_WORKSPACE
catkin build
source devel/setup.bash
roslaunch martian_mines rviz.launch