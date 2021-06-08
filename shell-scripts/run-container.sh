#!/bin/bash

docker run -it \
    -e DISPLAY=$DISPLAY \
    -e "TERM=xterm-256color" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $PWD/..:/home/dev/work/simulator \
    $1 \
    ros-dev:11 bash
