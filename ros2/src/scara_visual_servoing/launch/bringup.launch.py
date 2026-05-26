#!/usr/bin/env python3
"""
launch/bringup.launch.py
Lance le nœud VS + la cible simulée + robot_state_publisher (sans Gazebo).
Idéal pour tester la boucle de contrôle sur un robot réel ou en simulation Python.

Usage :
  ros2 launch scara_visual_servoing bringup.launch.py
  ros2 launch scara_visual_servoing bringup.launch.py mode:=live
  ros2 launch scara_visual_servoing bringup.launch.py mode:=sim demo_mode:=conveyor
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_pkg = get_package_share_directory('scara_description')
    xacro_file = os.path.join(desc_pkg, 'urdf', 'scara_robot.urdf.xacro')

    # ── Arguments ─────────────────────────────────────────────────────────
    mode      = LaunchConfiguration('mode')
    demo_mode = LaunchConfiguration('demo_mode')
    target_x  = LaunchConfiguration('target_x')
    target_y  = LaunchConfiguration('target_y')
    target_z  = LaunchConfiguration('target_z')
    gain      = LaunchConfiguration('gain')
    dt        = LaunchConfiguration('dt')

    args = [
        DeclareLaunchArgument('mode',      default_value='sim',
            description='"sim" | "live" | "realsense"'),
        DeclareLaunchArgument('demo_mode', default_value='fixed',
            description='"fixed" | "conveyor" | "sequence"'),
        DeclareLaunchArgument('target_x',  default_value='0.35'),
        DeclareLaunchArgument('target_y',  default_value='0.15'),
        DeclareLaunchArgument('target_z',  default_value='0.05'),
        DeclareLaunchArgument('gain',      default_value='0.5'),
        DeclareLaunchArgument('dt',        default_value='0.033'),
    ]

    # ── robot_state_publisher ─────────────────────────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', xacro_file]), value_type=str),
        }],
    )

    # ── joint_state_publisher (simulation sans Gazebo) ────────────────────
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'rate': 50,
        }],
    )

    # ── Nœud VS principal ─────────────────────────────────────────────────
    vs_node = Node(
        package='scara_visual_servoing',
        executable='vs_node',
        name='scara_vs_node',
        output='screen',
        parameters=[{
            'mode':          ParameterValue(mode,     value_type=str),
            'dt':            ParameterValue(dt,       value_type=float),
            'gain':          ParameterValue(gain,     value_type=float),
            'adaptive_gain': True,
            'target_x':      ParameterValue(target_x, value_type=float),
            'target_y':      ParameterValue(target_y, value_type=float),
            'target_z':      ParameterValue(target_z, value_type=float),
        }],
    )

    # ── Nœud cible simulée ────────────────────────────────────────────────
    sim_target = Node(
        package='scara_visual_servoing',
        executable='sim_target_node',
        name='sim_target_node',
        output='screen',
        parameters=[{
            'demo_mode': ParameterValue(demo_mode, value_type=str),
            'target_x':  ParameterValue(target_x,  value_type=float),
            'target_y':  ParameterValue(target_y,  value_type=float),
            'target_z':  ParameterValue(target_z,  value_type=float),
        }],
    )

    return LaunchDescription(args + [rsp, jsp, vs_node, sim_target])
