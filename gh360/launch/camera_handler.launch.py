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

    camera_frame_cmd = Node(
        package='gh360',
        executable='camera_frame',
        name='camera_frame'
    )

    
    
    return LaunchDescription([
        camera_startup,
        camera_frame_cmd,
    ])