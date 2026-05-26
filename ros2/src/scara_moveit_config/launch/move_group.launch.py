#!/usr/bin/env python3
"""
launch/move_group.launch.py
Lance le serveur MoveIt2 (move_group) pour le SCARA.
Usage :
  ros2 launch scara_moveit_config move_group.launch.py
  ros2 launch scara_moveit_config move_group.launch.py use_sim_time:=true
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def load_yaml(package_name, file_path):
    pkg_path = get_package_share_directory(package_name)
    full_path = os.path.join(pkg_path, file_path)
    with open(full_path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    desc_pkg  = get_package_share_directory('scara_description')
    mvt_pkg   = get_package_share_directory('scara_moveit_config')
    xacro_file = os.path.join(desc_pkg, 'urdf', 'scara_robot.urdf.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_sim = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Utiliser le temps simulé (Gazebo)')

    # ── Description du robot ──────────────────────────────────────────────
    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    # ── SRDF ─────────────────────────────────────────────────────────────
    srdf_file = os.path.join(mvt_pkg, 'config', 'scara_robot.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    # ── Paramètres cinématiques ───────────────────────────────────────────
    kinematics_yaml = load_yaml('scara_moveit_config', 'config/kinematics.yaml')

    # ── Limites articulaires ──────────────────────────────────────────────
    joint_limits_yaml = load_yaml('scara_moveit_config', 'config/joint_limits.yaml')
    joint_limits = {'robot_description_planning': joint_limits_yaml}

    # ── Contrôleurs MoveIt2 ───────────────────────────────────────────────
    moveit_controllers = load_yaml('scara_moveit_config', 'config/moveit_controllers.yaml')

    # ── Paramètres de planification OMPL ─────────────────────────────────
    ompl_planning_yaml = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': (
                'default_planner_request_adapters/AddTimeOptimalParameterization '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/FixStartStatePathConstraints'
            ),
            'start_state_max_bounds_error': 0.1,
        }
    }

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits,
            ompl_planning_yaml,
            moveit_controllers,
            {'use_sim_time': use_sim_time},
            {'publish_robot_description_semantic': True},
            {'allow_trajectory_execution': True},
            {'publish_planning_scene': True},
            {'publish_geometry_updates': True},
            {'publish_state_updates': True},
            {'publish_transforms_updates': True},
            {'monitor_dynamics': False},
        ],
    )

    return LaunchDescription([
        declare_sim,
        move_group_node,
    ])
