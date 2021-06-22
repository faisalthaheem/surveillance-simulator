#!/bin/sh
export SDF_PATH=$PWD/..
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/.gazebo/models:$PWD/models
roslaunch surveillance_simulator gazebo.launch