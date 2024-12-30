#!/bin/bash

docker run -it --rm --gpus all --network host --privileged \
    -v /dev:/dev \
    -v ~/ieee_simulation:/ieee_simulation \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    ros-humble-cuda
