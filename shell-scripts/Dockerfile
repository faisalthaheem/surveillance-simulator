FROM ubuntu:20.04

#!/bin/sh
# http://wiki.ros.org/noetic/Installation/Ubuntu
RUN apt update && export DEBIAN_FRONTEND=noninteractive && \
    apt install -y lsb-core

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add -

RUN apt update && export DEBIAN_FRONTEND=noninteractive && \
    apt -y install ros-noetic-desktop-full

RUN touch /etc/profile.d/ros-noetic-env.sh && chmod +x /etc/profile.d/ros-noetic-env.sh && \
    echo "source /opt/ros/noetic/setup.bash" >> /etc/profile.d/ros-noetic-env.sh && \
    echo "source /home/dev/work/simulator/catkin_ws/devel/setup.bash" >> /etc/profile.d/ros-noetic-env.sh

RUN apt update && export DEBIAN_FRONTEND=noninteractive && \
    apt install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential

RUN rosdep init && rosdep update

# RUN export uid=1000 gid=1000 && \
#     mkdir -p /home/dev/work && \
#     echo "dev:x:${uid}:${gid}:dev,,,:/home/dev:/bin/bash" >> /etc/passwd && \
#     echo "dev:x:${uid}:" >> /etc/group && \
#     echo "dev ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/dev && \
#     chmod 0440 /etc/sudoers.d/dev && \
#     chown ${uid}:${gid} -R /home/dev

# USER dev
# ENV HOME /home/dev

CMD /usr/bin/bash
