#!/bin/sh
lxc exec -t gazebo sh /home/ubuntu/work/simulator/shell-scripts/ros2-install-to-gazebo.sh
lxc exec -t gazebo -- apt install -y gazebo9
