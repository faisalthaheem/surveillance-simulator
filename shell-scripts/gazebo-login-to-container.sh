#!/bin/sh

echo "logging in as ubuntu"
lxc exec gazebo -- sudo --user ubuntu --login
