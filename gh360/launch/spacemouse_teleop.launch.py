#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro


def generate_launch_description():
    # package_name = 'gh360'
    # robot_name = 'gh360'
    # model_file_path = os.path.join(get_package_share_directory(package_name), 'urdf', robot_name+'.urdf')
    # robot_description_raw = xacro.process_file(model_file_path).toxml()

    # inverse_jacobian_cmd = Node(
    #     package='gh360',
    #     executable='inverse_jacobian',
    #     name='inverse_jacobian',
    #     # emulate_tty=True,
    #     parameters=[{
    #     'robot_description': robot_description_raw,
    #     'tcp_link_name': 'eef',
    #     'joint_states_topic': '/gh360_joint_states'}]
    # )

    teleop_cmd = Node(
        package='gh360',
        executable='teleop',
        name='teleop',    
    )
    
    space_mouse_cmd = Node(
            package='gh360_examples',
            executable='spacemouse',
            name='spacemouse'
    )

    reset_robot_cmd = Node(
        package='gh360_examples',
        executable='reset_robot',
        name='reset_robot'
    )

    # eef_pos_in_world_cmd = Node(
    #     package='gh360_examples',
    #     executable='eef_pos_in_world',
    #     name='eef_pos_in_world'
    # )

    # door_env_obs_cmd = Node(
    #     package='gh360_demonstration',
    #     executable='door_env_obs',
    #     name='door_env_obs'
    # )


    
    return LaunchDescription([
        # inverse_jacobian_cmd,
        teleop_cmd,
        space_mouse_cmd,
        reset_robot_cmd,
        # door_env_obs_cmd,
        # eef_pos_in_world_cmd
    ])