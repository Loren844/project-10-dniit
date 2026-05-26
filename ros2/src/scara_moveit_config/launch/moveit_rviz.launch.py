#!/usr/bin/env python3
"""
launch/moveit_rviz.launch.py
Lance robot_state_publisher + move_group + RViz2 (config MoveIt2).
Usage :
  ros2 launch scara_moveit_config moveit_rviz.launch.py
  ros2 launch scara_moveit_config moveit_rviz.launch.py use_sim_time:=true
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def load_yaml(package_name, file_path):
    pkg = get_package_share_directory(package_name)
    with open(os.path.join(pkg, file_path), 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    desc_pkg = get_package_share_directory('scara_description')
    mvt_pkg  = get_package_share_directory('scara_moveit_config')
    xacro_file = os.path.join(desc_pkg, 'urdf', 'scara_robot.urdf.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_sim = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Utiliser le temps simulé (Gazebo)')

    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    srdf_file = os.path.join(mvt_pkg, 'config', 'scara_robot.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    kinematics_yaml   = load_yaml('scara_moveit_config', 'config/kinematics.yaml')
    joint_limits_yaml = load_yaml('scara_moveit_config', 'config/joint_limits.yaml')

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time},
        ],
    )

    # move_group depuis son propre launch
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mvt_pkg, 'launch', 'move_group.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            {'robot_description_planning': joint_limits_yaml},
            {'use_sim_time': use_sim_time},
        ],
        arguments=['-d', os.path.join(mvt_pkg, 'config', 'moveit.rviz')]
            if os.path.isfile(os.path.join(mvt_pkg, 'config', 'moveit.rviz')) else [],
    )

    return LaunchDescription([
        declare_sim,
        robot_state_pub,
        move_group_launch,
        rviz2,
    ])
