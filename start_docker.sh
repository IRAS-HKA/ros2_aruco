#!/bin/sh
uid=$(eval "id -u")
gid=$(eval "id -g")
docker build --build-arg UID="$uid" --build-arg GID="$gid" -t aruco/ros:foxy .

echo "Run Container"
xhost + local:root
docker run --name aruco -v /dev:/dev --group-add=dialout -it --privileged --net host -e DISPLAY=$DISPLAY --rm aruco/ros:foxy

# zum mounten, pfad noch anpassen
# -v $PWD/ros:/home/nuc/ros2_aruco/ros2_aruco


# docker run --name common_cmake_python -it --privileged  -v $PWD/ros:/home/docker/ros2_ws/src --net host -e DISPLAY=$DISPLAY --rm common_python_package/ros:foxy

#--name ptu_driver_cmake_python
