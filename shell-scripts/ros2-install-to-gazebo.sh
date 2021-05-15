#!/bin/sh
# http://wiki.ros.org/noetic/Installation/Ubuntu
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -

sudo apt update
sudo apt -y install ros-noetic-desktop-full

touch /etc/profile.d/ros-noetic-env.sh && chmod +x /etc/profile.d/ros-noetic-env.sh
echo "source /opt/ros/noetic/setup.bash" >> /etc/profile.d/ros-noetic-env.sh
echo "source /home/ubuntu/work/simulator/catkin_ws/devel/setup.bash" >> /etc/profile.d/ros-noetic-env.sh

source /etc/profile.d/ros-noetic-env.sh

sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install -y python3-rosdep

sudo rosdep init
rosdep update
