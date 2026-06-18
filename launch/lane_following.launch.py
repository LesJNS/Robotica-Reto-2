#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('capytown_esan')
    hsv = os.path.join(pkg, 'config', 'hsv_params.yaml')
    pid = os.path.join(pkg, 'config', 'pid_params.yaml')

    return LaunchDescription([

        # 1. Cámara → /camera/image_raw
        Node(
            package='capytown_esan',
            executable='cam_pub',
            name='cam_pub',
            parameters=[{'device': 0, 'width': 640, 'height': 480, 'fps': 30}],
            output='screen',
        ),

        # 2. Detector de carril: /camera/image_raw → /lane_error
        Node(
            package='capytown_esan',
            executable='lane_detector',
            name='lane_detector',
            parameters=[hsv],
            output='screen',
        ),

        # 3. Controlador PID: /lane_error → /cmd_vel
        Node(
            package='capytown_esan',
            executable='lane_controller',
            name='lane_controller',
            parameters=[pid],
            output='screen',
        ),

        # 4. Web viewer en http://<IP>:5800/
        Node(
            package='capytown_esan',
            executable='camera_stream',
            name='camera_stream',
            output='screen',
        ),
    ])
