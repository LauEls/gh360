#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    camera_startup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('realsense2_camera'), 'launch'), '/rs_launch.py'])
    )

    aruco_recognition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ros2_aruco'), 'launch'), '/aruco_recognition.launch.py'])
    )

    camera_frame_cmd = Node(
        package='gh360_examples',
        executable='camera_frame',
        name='camera_frame'
    )

    door_handle_frame_cmd = Node(
        package='gh360_examples',
        executable='door_handle_pose',
        name='door_handle_pose'
    )
    
    return LaunchDescription([
        camera_startup,
        aruco_recognition,
        camera_frame_cmd,
        door_handle_frame_cmd,
    ])