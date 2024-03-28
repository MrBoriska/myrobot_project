#!/bin/bash

xhost +local:docker

docker run -e DISPLAY=$DISPLAY \
           -v /tmp/.X11-unix:/tmp/.X11-unix: \
           -v /dev/dri:/dev/dri \
           -v $(pwd):/myrobot_project \
           -it \
           --name myrobot_project \
           myrobot_project