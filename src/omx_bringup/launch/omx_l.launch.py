#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('use_mock_hardware', default_value='false',
                              description='true = mock ; false = real hardware'),
        DeclareLaunchArgument('port_name', default_value='/dev/ttyACM1',
                              description='Real hardware serial port'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Whether to start RViz'),
    ]

    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    port_name = LaunchConfiguration('port_name')

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                FindExecutable(name='xacro'), ' ',
                PathJoinSubstitution([
                    FindPackageShare('omx_description'), 'urdf', 'omx_l.urdf.xacro']),
                ' use_mock_hardware:=', use_mock_hardware,
                ' port_name:=', port_name,
            ]),
            value_type=str,
        )
    }

    controllers = PathJoinSubstitution(
        [FindPackageShare('omx_bringup'), 'config', 'omx_l_controllers.yaml'])
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('omx_description'), 'rviz', 'view_robot.rviz'])

    control_node = Node(
        package='controller_manager', executable='ros2_control_node',
        parameters=[controllers], output='both',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[robot_description], output='both',
    )

    jsb_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster'], output='screen',
    )

    gripper_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['gripper_controller'], output='screen',
    )

    delay_gripper = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[gripper_spawner]))

    rviz_node = Node(
        package='rviz2', executable='rviz2', arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')), output='log',
    )

    return LaunchDescription(declared_arguments + [
        control_node, robot_state_publisher, jsb_spawner, delay_gripper, rviz_node,
    ])
