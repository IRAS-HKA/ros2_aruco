#!/bin/sh
uid=$(eval "id -u")
gid=$(eval "id -g")
docker build --build-arg UID="$uid" --build-arg GID="$gid" -t ros2_aruco/ros:foxy .

echo "Run Container"
xhost + local:root
docker run --name aruco -v $PWD:/home/docker/ros2_ws/src/ros2_aruco -it --privileged --net host -e DISPLAY=$DISPLAY --rm ros2_aruco/ros:foxy

