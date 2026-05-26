#!/usr/bin/env python3
"""
gazebo_bridge.py — Nœud ROS2 : pont Gazebo / état articulaire initial
═══════════════════════════════════════════════════════════════════════
Publie la position initiale vers le joint_trajectory_controller
au démarrage de la simulation Gazebo, évitant l'effondrement du robot
au lancement.
"""
from __future__ import annotations

import time
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class GazeboBridge(Node):
    """Envoie la commande de position initiale au robot simule."""

    # DH order: [theta1=joint_1theta, d2=joint_1z, theta3=joint3, theta4=joint4]
    JOINT_NAMES = ['joint_1theta', 'joint_1z', 'joint3', 'joint4']
    HOME_POSE   = [0.0, 0.35, 0.3, 0.0]   # matches URDF initial values (hors singularite)

    def __init__(self):
        super().__init__('gazebo_bridge')

        self.declare_parameter('delay_s',    2.0)   # délai avant envoi (s)
        self.declare_parameter('repeat',     5)     # nombre d'envois

        self._delay  = self.get_parameter('delay_s').value
        self._repeat = self.get_parameter('repeat').value
        self._count  = 0

        self._pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10,
        )

        # Timer unique déclenché après delay_s
        self._timer = self.create_timer(self._delay, self._send_home)
        self.get_logger().info(
            f'GazeboBridge : envoi de la position initiale dans {self._delay:.1f} s')

    def _send_home(self):
        if self._count >= self._repeat:
            self._timer.cancel()
            self.get_logger().info('GazeboBridge : position initiale atteinte.')
            return

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names  = self.JOINT_NAMES

        pt = JointTrajectoryPoint()
        pt.positions      = self.HOME_POSE
        pt.velocities     = [0.0] * 4
        pt.time_from_start = Duration(sec=2, nanosec=0)
        msg.points = [pt]

        self._pub.publish(msg)
        self._count += 1
        self.get_logger().info(
            f'Commande initiale publiée ({self._count}/{self._repeat})')


def main(args=None):
    rclpy.init(args=args)
    node = GazeboBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
