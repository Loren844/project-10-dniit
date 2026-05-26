#!/usr/bin/env python3
"""
launch/gazebo_sim.launch.py
Simulation complete : Gazebo Sim (Ignition/Garden) + ros2_control + MoveIt2 + VS.

Architecture de lancement :
  1. gz_sim         -- moteur physique + GUI Gazebo Sim
  2. robot_state_publisher
  3. spawn (create) -- import du robot dans Gazebo Sim
  4. gz_bridge      -- clock /clock
  5. joint_state_broadcaster
  6. joint_trajectory_controller
  7. move_group     -- MoveIt2 (optionnel)
  8. vs_node        -- asservissement visuel
  9. sim_target_node

Prerequis :
  sudo apt install ros-humble-ros-gz ros-humble-gz-ros2-control

Usage :
  ros2 launch scara_visual_servoing gazebo_sim.launch.py
  ros2 launch scara_visual_servoing gazebo_sim.launch.py headless:=true
  ros2 launch scara_visual_servoing gazebo_sim.launch.py with_moveit:=true demo_mode:=conveyor
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    ExecuteProcess, TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc_pkg  = get_package_share_directory('scara_description')
    mvt_pkg   = get_package_share_directory('scara_moveit_config')
    xacro_file = os.path.join(desc_pkg, 'urdf', 'scara_robot.urdf.xacro')
    controllers_yaml = os.path.join(mvt_pkg, 'config', 'ros2_controllers.yaml')

    # ── Arguments ─────────────────────────────────────────────────────────
    headless    = LaunchConfiguration('headless')
    with_moveit = LaunchConfiguration('with_moveit')
    demo_mode   = LaunchConfiguration('demo_mode')
    target_x    = LaunchConfiguration('target_x')
    target_y    = LaunchConfiguration('target_y')
    target_z    = LaunchConfiguration('target_z')
    gain        = LaunchConfiguration('gain')

    args = [
        DeclareLaunchArgument('headless',    default_value='false',
            description='Lancer Gazebo sans interface graphique'),
        DeclareLaunchArgument('with_moveit', default_value='false',
            description='Démarrer le serveur MoveIt2 move_group'),
        DeclareLaunchArgument('demo_mode',   default_value='fixed',
            description='"fixed" | "conveyor" | "sequence"'),
        DeclareLaunchArgument('target_x',    default_value='0.35'),
        DeclareLaunchArgument('target_y',    default_value='0.15'),
        DeclareLaunchArgument('target_z',    default_value='0.05'),
        DeclareLaunchArgument('gain',        default_value='1.2'),
    ]

    # ── Description robot (xacro + controllers_yaml en arg) ─────────────
    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file,
                     ' controllers_yaml:=', controllers_yaml]),
            value_type=str),
    }

    # ── Gazebo Sim avec GUI ───────────────────────────────────────────────
    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': '-r empty.sdf',
            'on_exit_shutdown': 'true',
        }.items(),
        condition=UnlessCondition(headless),
    )

    # ── Gazebo Sim headless ───────────────────────────────────────────────
    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': '-s -r empty.sdf',
            'on_exit_shutdown': 'true',
        }.items(),
        condition=IfCondition(headless),
    )

    # ── robot_state_publisher ─────────────────────────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # Suppression des anciennes entites (world name 'default' dans empty.sdf)
    _del_cmd = (
        'GZ=$(command -v ign 2>/dev/null && echo ign || echo gz); '
        'MSG=$([ "$GZ" = "ign" ] && echo ignition || echo gz); '
        'for name in scara_robot conveyor_belt deposit_zone grasp_object; do '
        '$GZ service -s /world/default/remove --reqtype ${MSG}.msgs.Entity '
        '--reptype ${MSG}.msgs.Boolean --timeout 500 '
        '--req "name: \"$name\" type: MODEL" 2>/dev/null; '
        'done; true'
    )
    delete_old = TimerAction(
        period=2.0,
        actions=[ExecuteProcess(cmd=['bash', '-c', _del_cmd], output='log')],
    )

    # SDF des objets a spawner dans Gazebo
    # Tapis roulant le long de l'axe Y, a x=0.55 m (face au robot bras tendu en +X)
    _SDF_CONVEYOR = (
        '<sdf version="1.7"><model name="conveyor_belt"><static>true</static>'
        '<pose>0.55 0.0 0.025 0 0 0</pose><link name="link">'
        '<visual name="v"><geometry><box>'
        '<size>0.12 1.0 0.05</size></box></geometry>'
        '<material><ambient>0.15 0.65 0.15 1</ambient>'
        '<diffuse>0.2 0.75 0.2 1</diffuse>'
        '<specular>0.1 0.1 0.1 1</specular></material></visual>'
        '<collision name="c"><geometry><box>'
        '<size>0.12 1.0 0.05</size></box></geometry></collision>'
        '</link></model></sdf>'
    )
    # Zone de depot : cylindre bleu a (0, -0.55) — robot tourne de 90 deg
    _SDF_DEPOSIT = (
        '<sdf version="1.7"><model name="deposit_zone"><static>true</static>'
        '<pose>0.0 -0.55 0.02 0 0 0</pose><link name="link">'
        '<visual name="v"><geometry><cylinder>'
        '<radius>0.12</radius><length>0.04</length></cylinder></geometry>'
        '<material><ambient>0.1 0.1 0.85 1</ambient>'
        '<diffuse>0.15 0.15 1.0 1</diffuse>'
        '<specular>0.1 0.1 0.1 1</specular></material></visual>'
        '<collision name="c"><geometry><cylinder>'
        '<radius>0.12</radius><length>0.04</length></cylinder></geometry></collision>'
        '</link></model></sdf>'
    )
    # Objet a saisir : sphere rouge DYNAMIQUE avec collision.
    # Historique des tentatives :
    #   static=true    -> set_pose ignore (limitation documentee Ignition Fortress)
    #   kinematic=true -> set_pose ignore (DART ecrase le changement ECM chaque pas)
    #   dynamic + <gravity>false</gravity> -> sphere invisible : DART (moteur physique
    #     par defaut d'Ignition Fortress) NE supporte PAS le flag gravite par-lien.
    #     La sphere tombe immediatement malgre la balise.
    # Solution finale : corps dynamique NORMAL (gravite activee) + <collision> :
    #   - La sphere repose sur le tapis par contact physique (ne tombe pas)
    #   - set_pose fonctionne sur les corps dynamiques (DART teleporte le corps)
    #   - Spawn a z=0.10 (1cm au-dessus du tapis) pour eviter la penetration initiale
    _SDF_OBJECT = (
        '<sdf version="1.7"><model name="grasp_object">'
        '<pose>0.55 -0.45 0.10 0 0 0</pose>'
        '<link name="link">'
        '<inertial><mass>0.1</mass>'
        '<inertia><ixx>0.000064</ixx><ixy>0</ixy><ixz>0</ixz>'
        '<iyy>0.000064</iyy><iyz>0</iyz><izz>0.000064</izz></inertia>'
        '</inertial>'
        '<visual name="v"><geometry><sphere>'
        '<radius>0.04</radius></sphere></geometry>'
        '<material><ambient>0.9 0.1 0.05 1</ambient>'
        '<diffuse>1.0 0.15 0.1 1</diffuse>'
        '<specular>0.2 0.1 0.1 1</specular></material></visual>'
        '<collision name="c"><geometry><sphere>'
        '<radius>0.04</radius></sphere></geometry></collision>'
        '</link></model></sdf>'
    )

    # Positions explicites (-x/-y/-z) : en Ignition Fortress, ros_gz_sim create
    # peut ignorer le tag <pose> interne au SDF ; les flags CLI sont prioritaires.
    spawn_conveyor = TimerAction(period=5.5, actions=[Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'conveyor_belt', '-string', _SDF_CONVEYOR,
                   '-x', '0.55', '-y', '0.0', '-z', '0.025'],
        output='screen')])
    spawn_deposit = TimerAction(period=6.0, actions=[Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'deposit_zone', '-string', _SDF_DEPOSIT,
                   '-x', '0.0', '-y', '-0.55', '-z', '0.02'],
        output='screen')])
    spawn_object = TimerAction(period=6.5, actions=[Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'grasp_object', '-string', _SDF_OBJECT,
                   '-x', '0.55', '-y', '-0.45', '-z', '0.10'],
        output='screen')])

    # Bridge /clock Gazebo -> ROS ───────────────────────────────────────
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'
        ],
        output='screen',
    )

    # ── Spawn du robot dans Gazebo Sim ────────────────────────────────────
    # Attendre 4s : Gazebo + robot_state_publisher doivent etre prets
    spawn = TimerAction(
        period=4.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'scara_robot',
                '-topic', '/robot_description',
                '-x', '0', '-y', '0', '-z', '0',
            ],
            output='screen',
        )],
    )

    # ── Contrôleurs ros2_control ────────────────────────────────────────
    # Utiliser Node(spawner) plutôt que ExecuteProcess(bash) : le Node hérite
    # du même environnement ROS que le processus de lancement (AMENT_PREFIX_PATH,
    # LD_LIBRARY_PATH corrects), ce qui permet au controller_manager de trouver
    # les plugins joint_state_broadcaster et joint_trajectory_controller.
    spawn_jsb = TimerAction(
        period=20.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                '-t', 'joint_state_broadcaster/JointStateBroadcaster',
                '--controller-manager-timeout', '30',
            ],
            output='screen',
        )],
    )

    spawn_jtc = TimerAction(
        period=26.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_trajectory_controller',
                '-t', 'joint_trajectory_controller/JointTrajectoryController',
                '--controller-manager-timeout', '30',
            ],
            output='screen',
        )],
    )

    # ── MoveIt2 (optionnel) ───────────────────────────────────────────────
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mvt_pkg, 'launch', 'move_group.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=IfCondition(with_moveit),
    )

    # ── VS node ───────────────────────────────────────────────────────────
    vs_node = Node(
        package='scara_visual_servoing',
        executable='vs_node',
        name='scara_vs_node',
        output='screen',
        parameters=[{
            'mode':          'sim',
            'use_sim_time':  True,
            'gain':          ParameterValue(gain,     value_type=float),
            'adaptive_gain': True,
            'target_x':      ParameterValue(target_x, value_type=float),
            'target_y':      ParameterValue(target_y, value_type=float),
            'target_z':      ParameterValue(target_z, value_type=float),
        }],
    )

    # ── Cible simulee ─────────────────────────────────────────────────────
    # Demarre APRES l'activation du JTC (t=26s) pour que le robot soit operationnel
    # quand le tapis commence a bouger. Ainsi le robot traque l'objet EN MOUVEMENT.
    sim_target = TimerAction(period=28.0, actions=[Node(
        package='scara_visual_servoing',
        executable='sim_target_node',
        name='sim_target_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'demo_mode': ParameterValue(demo_mode, value_type=str),
            'target_x':  ParameterValue(target_x,  value_type=float),
            'target_y':  ParameterValue(target_y,  value_type=float),
            'target_z':  ParameterValue(target_z,  value_type=float),
        }],
    )])

    return LaunchDescription(args + [
        gz_sim_gui,
        gz_sim_headless,
        delete_old,
        rsp,
        gz_bridge,
        spawn,
        spawn_conveyor,
        spawn_deposit,
        spawn_object,
        spawn_jsb,
        spawn_jtc,
        move_group_launch,
        vs_node,
        sim_target,
    ])
