#!/bin/bash
echo "Creating container"
lxc launch ubuntu:20.04 --profile default --profile x11 gazebo

echo "Mounting project"
lxc exec gazebo -- "mkdir -p /home/ubuntu/work/simulator"
lxc config device add gazebo projectroot disk source=$PWD/../ path=/home/ubuntu/work/simulator
lxc exec gazebo -- "ls /home/ubuntu/work/simulator"

