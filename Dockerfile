##############################################################################
##                                 Base Image                               ##
##############################################################################
ARG ROS_DISTRO=foxy
FROM ros:$ROS_DISTRO-ros-base
ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
RUN rosdep update --rosdistro $ROS_DISTRO

##############################################################################
##                                 Global Dependecies                       ##
##############################################################################
RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-$ROS_DISTRO-cv-bridge \
    python3-pip \
    ros-$ROS_DISTRO-ros2bag \
    ros-$ROS_DISTRO-rosbag2 \
    ros-$ROS_DISTRO-rosbag2-storage \
    ros-$ROS_DISTRO-rosbag2-storage-dbgsym \
    ros-$ROS_DISTRO-rosbag2-storage-default-plugins \
    ros-$ROS_DISTRO-rosbag2-storage-default-plugins-dbgsym \
    ros-$ROS_DISTRO-rosbag2-test-common \
    ros-$ROS_DISTRO-rosbag2-tests \
    ros-$ROS_DISTRO-rosbag2-converter-default-plugins \
    ros-$ROS_DISTRO-rosbag2-converter-default-plugins-dbgsym \
    ros-$ROS_DISTRO-rosbag2-transport \
    ros-$ROS_DISTRO-rosbag2-transport-dbgsym \
    ros-$ROS_DISTRO-tf2-tools \
    ros-$ROS_DISTRO-tf-transformations \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN pip3 install \
    opencv-contrib-python==4.6.0.66 \
    transforms3d

##############################################################################
##                                 Create User                              ##
##############################################################################
ARG USER=docker
ARG PASSWORD=petra
ARG UID=1000
ARG GID=1000
ARG DOMAIN_ID=8
ENV UID=$UID
ENV GID=$GID
ENV USER=$USER
ENV ROS_DOMAIN_ID=$DOMAIN_ID
RUN groupadd -g "$GID" "$USER"  && \
    useradd -m -u "$UID" -g "$GID" --shell $(which bash) "$USER" -G sudo && \
    echo "$USER:$PASSWORD" | chpasswd && \
    echo "%sudo ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/sudogrp
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc

COPY dds_profile.xml /home/$USER
RUN chown $USER:$USER /home/$USER/dds_profile.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/home/$USER/dds_profile.xml

USER $USER 
RUN mkdir -p /home/$USER/ros2_ws/src

##############################################################################
##                                 User Dependecies                         ##
##############################################################################
WORKDIR /home/$USER/ros2_ws/src
COPY . ./ros2_aruco

##############################################################################
##                                 Build ROS and run                        ##
##############################################################################
WORKDIR /home/$USER/ros2_ws
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --symlink-install
RUN echo "source /home/$USER/ros2_ws/install/setup.bash" >> /home/$USER/.bashrc

RUN sudo sed --in-place --expression \
    '$isource "/home/$USER/ros2_ws/install/setup.bash"' \
    /ros_entrypoint.sh

# CMD /bin/bash
CMD ["ros2", "launch", "ros2_aruco", "aruco.launch.py"]

