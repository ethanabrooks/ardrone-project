#! /usr/bin/env bash

echo ------------------------------------------------------------------
echo Xvfb :1 -screen 0 1600x1200x16  &
Xvfb :1 -screen 0 1600x1200x16  &
echo ------------------------------------------------------------------
echo source /opt/ros/kinetic/setup.bash; source /catkin/devel/setup.bash; roscd a3c
source /opt/ros/kinetic/setup.bash; source /catkin/devel/setup.bash; roscd a3c
echo ------------------------------------------------------------------
echo roslaunch a3c train.launch worker-args:="$1" gui:=$2
roslaunch a3c train.launch worker-args:="$1" gui:=$2
echo ------------------------------------------------------------------
