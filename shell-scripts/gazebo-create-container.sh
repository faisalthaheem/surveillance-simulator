#!/bin/bash
echo "Creating container"
lxc launch ubuntu:20.04 --profile default --profile x11 gazebo

echo "Mounting project"
printf "uid $(id -u) 1000\ngid $(id -g) 1000" | lxc config set gazebo raw.idmap -
lxc exec gazebo -- "mkdir -p /home/ubuntu/work/simulator"
lxc config device add gazebo projectroot disk source=$PWD/../ path=/home/ubuntu/work/simulator
lxc exec gazebo -- "ls /home/ubuntu/work/simulator"

