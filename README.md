# ROS2 ArUco

ROS2 Wrapper for OpenCV Aruco Marker Tracking

Based on https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco

This node locates Aruco AR markers in images and publishes their ids and poses.

## Interface:

- `aruco_node` (ROS2 Foxy python)

    Node for marker detection

- `aruco_generate_marker.py` (python)

    Python script for generating ArUco marker

### PTU Node
---

#### Topics

Subscriptions:
* `/camera/image_raw` (`sensor_msgs.msg.Image`)
* `/camera/camera_info` (`sensor_msgs.msg.CameraInfo`)

Published Topics:
* `/aruco_poses` (`geometry_msgs.msg.PoseArray`) - Poses of all detected markers (suitable for rviz visualization)
* `/aruco_markers` (`ros2_aruco_interfaces.msg.ArucoMarkers`) - Provides an array of all poses along with the corresponding marker ids

#### Parameters

* `marker_size` - size of the markers in meters (default .0625)
* `aruco_dictionary_id` - dictionary that was used to generate markers (default `DICT_5X5_250`)
* `image_topic` - image topic to subscribe to (default `/camera/image_raw`)
* `camera_info_topic` - Camera info topic to subscribe to (default `/camera/camera_info`)
* `camera_frame` - Camera optical frame to use (default to the frame id provided by the camera info message.)

## How to run:

### Dependencies

- opencv-contrib-python

- cv_bridge

### Installation

    pip install opencv-contrib-python # or pip3

    sudo apt-get install ros-$ROS_DISTRO-cv-bridge 

Build packages with ros

    colcon build --symlink-install

### Start the nodes

    ros2 launch ros2_aruco aruco.launch.py

## TF2
```
ros2 run tf2_tools view_frames.py
ros2 run tf2_ros tf2_echo [reference_frame] [target_frame]
```

## Generating Marker Images

```
ros2 run ros2_aruco aruco_generate_marker
```

Pass the `-h` flag for usage information: 

```
usage: aruco_generate_marker [-h] [--id ID] [--size SIZE] [--dictionary]

Generate a .png image of a specified maker.

optional arguments:
  -h, --help     show this help message and exit
  --id ID        Marker id to generate (default: 1)
  --size SIZE    Side length in pixels (default: 200)
  --dictionary   Dictionary to use. Valid options include: DICT_4X4_100,
                 DICT_4X4_1000, DICT_4X4_250, DICT_4X4_50, DICT_5X5_100,
                 DICT_5X5_1000, DICT_5X5_250, DICT_5X5_50, DICT_6X6_100,
                 DICT_6X6_1000, DICT_6X6_250, DICT_6X6_50, DICT_7X7_100,
                 DICT_7X7_1000, DICT_7X7_250, DICT_7X7_50, DICT_ARUCO_ORIGINAL
                 (default: DICT_5X5_250)
```
