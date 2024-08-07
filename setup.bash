#!/bin/bash

export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/home/nearthlab/nearthlab/ros2_ws/build/livox_laser_simulation:/usr/include/gazebo-11:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/nearthlab/nearthlab/ros2_ws/build/livox_laser_simulation:/usr/include/gazebo-11
#export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH::/home/nearthlab/nearthlab/ros2_ws/src/livox_laser_simulation