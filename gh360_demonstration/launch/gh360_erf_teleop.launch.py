#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    demo_node = Node(
        package='gh360_demonstration',
        executable='gh360_erf_teleop',
        name='gh360_erf_teleop',
        namespace='gh360_control',
        output='screen'
    )

    spacemouse_node = Node(
        package='gh360_demonstration',
        executable='spacemouse',
        name='spacemouse',
        namespace='gh360_control',
        #parameters=[spacemouse_config_file],
        output='screen')
    
    return LaunchDescription([
        # controller_nodes,
        demo_node,
        spacemouse_node,
    ])