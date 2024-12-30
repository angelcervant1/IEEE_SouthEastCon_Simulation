#!/bin/bash

docker run -it --rm --gpus all --network host --privileged \
    -v /dev:/dev \
    -v ~/IEEE_SouthEastCon_Simulation:/IEEE_SouthEastCon_Simulation\
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    ros-humble-cuda
