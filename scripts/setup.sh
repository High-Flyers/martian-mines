#!/bin/bash
catkin build
source devel/setup.bash
cd src/martian-mines && pip install -e . --no-deps