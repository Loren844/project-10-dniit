#!/usr/bin/env python3
"""
launch/display.launch.py
Visualisation du robot SCARA dans RViz2 avec sliders de joint.
Usage :
  ros2 launch scara_description display.launch.py
  ros2 launch scara_description display.launch.py use_gui:=false
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('scara_description')
    xacro_file = os.path.join(pkg, 'urdf', 'scara_robot.urdf.xacro')

    use_gui = LaunchConfiguration('use_gui')
    declare_use_gui = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Activer les sliders joint_state_publisher_gui'
    )

    robot_description = {
        'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    }

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    joint_state_pub_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(use_gui),
    )

    rviz_config = os.path.join(pkg, 'config', 'display.rviz')
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.isfile(rviz_config) else [],
    )

    return LaunchDescription([
        declare_use_gui,
        robot_state_pub,
        joint_state_pub_gui,
        rviz2,
    ])
