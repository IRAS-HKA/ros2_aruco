#!/bin/sh
uid=$(eval "id -u")
gid=$(eval "id -g")

echo "Run Container"
xhost + local:root
docker run --name aruco  -it --privileged --net host -e DISPLAY=$DISPLAY --rm ros2_aruco/ros:foxy
# -v $PWD:/home/docker/ros2_ws/src/ros2_aruco
