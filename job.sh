#! /usr/bin/env bash


echo ------------------------------------------------------------------
echo source /opt/ros/kinetic/setup.bash; source /catkin/devel/setup.bash; roscd a3c
source /opt/ros/kinetic/setup.bash; source /catkin/devel/setup.bash; roscd a3c
echo ------------------------------------------------------------------
echo roscd a3c
roscd a3c
echo ------------------------------------------------------------------
echo CUDA_VISIBLE_DEVICES= python job.py $1
CUDA_VISIBLE_DEVICES= python job.py $1
echo ------------------------------------------------------------------
