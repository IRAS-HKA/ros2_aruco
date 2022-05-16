import os
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    config = os.path.join(
        get_package_prefix('ros2_aruco'),
        '..',
        '..',
        'src',
        'ros2_aruco',
        'ros2_aruco',
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='ros2_aruco',
            executable='aruco_node',
            name='aruco_node',
            output='screen',
            parameters=[config]
        )
    ])
