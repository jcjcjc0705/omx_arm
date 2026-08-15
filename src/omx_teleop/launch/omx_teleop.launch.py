#!/usr/bin/env python3
"""Mirror node only. Bring both arms up first with omx_bringup.

    ros2 launch omx_bringup omx_f.launch.py
    ros2 launch omx_bringup omx_l.launch.py
    ros2 launch omx_teleop omx_teleop.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('align_tolerance', default_value='0.15',
                              description='max leader/follower gap before arming, rad'),
        DeclareLaunchArgument('gripper_joint', default_value='gripper_joint_1',
                              description='empty string disables the startup open'),
        DeclareLaunchArgument('gripper_open', default_value='0.7',
                              description='follower gripper angle commanded once at startup, '
                                          'rad. Mirrors the leader trigger rest position '
                                          'through signs[-1], so -0.7 leader -> +0.7 follower.'),
    ]

    mirror = Node(
        package='omx_teleop',
        executable='leader_follower_mirror',
        name='leader_follower_mirror',
        output='screen',
        parameters=[{
            'leader_states_topic': '/omx_leader/joint_states',
            'follower_states_topic': '/omx_follower/joint_states',
            'follower_controller': '/omx_follower/position_controller',
            'joint_names': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5',
                            'gripper_joint_1'],
            'signs': [1.0, 1.0, 1.0, 1.0, 1.0, -1.0],
            'offsets': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'align_tolerance': ParameterValue(
                LaunchConfiguration('align_tolerance'), value_type=float),
            'gripper_joint': ParameterValue(
                LaunchConfiguration('gripper_joint'), value_type=str),
            'gripper_open': ParameterValue(
                LaunchConfiguration('gripper_open'), value_type=float),
        }],
    )

    return LaunchDescription(declared_arguments + [mirror])
