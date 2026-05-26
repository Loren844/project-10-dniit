#!/usr/bin/env python3
"""
sim_target_node.py - Simulation tapis roulant + pick-and-place
==============================================================
Scenario :
  Un objet apparait sur un tapis roulant (y=+0.15, z=0.05) et se deplace
  en -X. Le robot SCARA le saisit dans la zone de travail, puis le depose
  sur le point de depot (y=-0.35).

Topics publies :
  /vs/target_pose   (PoseStamped) - position objet sur tapis (zone de saisie)
  /vs/deposit_pose  (PoseStamped) - point de depot fixe
  /vs/target_marker  (Marker)    - sphere rouge = objet
  /vs/deposit_marker (Marker)    - cylindre bleu = zone depot
  /vs/conveyor_marker (Marker)   - boite verte = tapis roulant

Topics abonnes :
  /vs/status (String) - "PICKED" | "DEPOSITED" pour sync etat

Parametres :
  conveyor_speed  : float  vitesse tapis (m/s)  defaut 0.04
  publish_rate    : float  frequence Hz          defaut 20.0
"""
from __future__ import annotations

import math
import os
import shutil
import subprocess
import threading
import rclpy
from rclpy.node import Node
import numpy as np

# Bindings Python ignition.transport : même partition/transport que Gazebo -> set_pose fiable
# Fallback subprocess si non disponible (Ubuntu 22.04 + Fortress les installe normalement)
try:
    import ignition.transport as _ign_tr
    from ignition.msgs.pose_pb2 import Pose as _IgPose
    from ignition.msgs.boolean_pb2 import Boolean as _IgBool
    _IGN_NODE = _ign_tr.Node()
    _USE_IGN_PYTHON = True
except Exception as _ign_import_exc:
    _IGN_NODE = None
    _USE_IGN_PYTHON = False
    _IgPose = None
    _IgBool = None
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import String, ColorRGBA
from builtin_interfaces.msg import Duration


# Workspace SCARA: a2=0.4, a3=0.3 -> r reachable in [0.5, 0.7] m (|theta3|<=90 deg)
# Tapis roulant le long de l'axe Y, a position X fixe (face au robot bras tendu en +X)
# Objet se deplace de y=-0.45 (bas) vers y=+0.45 (haut)
CONVEYOR_X  = 0.55   # tapis a x=0.55 m (r=0.55 m, dans le workspace)
CONVEYOR_Z  = 0.14   # Le robot s'arrêtera 5 cm plus haut (juste au-dessus de la boule)
OBJ_START_Y = -0.45  # objet entre en scene depuis le bas (-Y)
OBJ_END_Y   = 0.45   # objet sort par le haut (+Y, reset)
PICK_ZONE_Y = 0.0    # zone de saisie : y=0 (face au robot, r=0.55 m)
PICK_PAUSE  = 3.0    # secondes de pause du tapis quand l'objet est en zone de saisie

DEPOSIT_X   = 0.0
DEPOSIT_Y   = -0.55
DEPOSIT_Z   = 0.14   # Pareil pour ne pas écraser le socle bleu de dépôt

OBJ_VISUAL_Z = 0.09
GZ_WORLD_NAME = 'empty'

if shutil.which('ign'):
    _GZ_CMD = 'ign'
    _GZ_MSG_NS = 'ignition'
else:
    _GZ_CMD = 'gz'
    _GZ_MSG_NS = 'gz'


class SimTargetNode(Node):

    # Etats internes du noeud
    STATE_CONVEYOR  = 'CONVEYOR'   # tapis en mouvement
    STATE_PAUSED    = 'PAUSED'     # objet arrete, robot en approche
    STATE_PICKED    = 'PICKED'     # robot a saisi, objet "disparu"
    STATE_DEPOSITED = 'DEPOSITED'  # objet depose, reset en cours

    def __init__(self):
        super().__init__('sim_target_node')

        self.declare_parameter('conveyor_speed', 0.04)
        self.declare_parameter('publish_rate',   20.0)

        self._speed = self.get_parameter('conveyor_speed').value
        rate        = self.get_parameter('publish_rate').value

        # Publications
        self._pub_target   = self.create_publisher(PoseStamped, '/vs/target_pose',    10)
        self._pub_deposit  = self.create_publisher(PoseStamped, '/vs/deposit_pose',   10)
        self._pub_obj_mk   = self.create_publisher(Marker,      '/vs/target_marker',  10)
        self._pub_dep_mk   = self.create_publisher(Marker,      '/vs/deposit_marker', 10)
        self._pub_conv_mk  = self.create_publisher(Marker,      '/vs/conveyor_marker',10)

        # Abonnements
        self.create_subscription(String, '/vs/status', self._cb_status, 10)
        self.create_subscription(PoseStamped, '/vs/tcp_pose', self._cb_tcp_pose, 10)

        # Etat
        self._state     = self.STATE_CONVEYOR
        self._obj_y     = OBJ_START_Y
        self._pause_t0  = 0.0
        # Position TCP du bras (pour sphere pendant CARRYING)
        self._tcp_pose  = np.array([CONVEYOR_X, PICK_ZONE_Y, OBJ_VISUAL_Z])

        # Deplace la sphere Gazebo via gz service (thread daemon)
        self._gz_busy   = False
        self._gz_tick   = 0
        self._gz_err_count = 0
        # Demarre a la position initiale (appelé avant spawn -> echec silencieux OK)
        self._gz_move_object(CONVEYOR_X, OBJ_START_Y, OBJ_VISUAL_Z)

        dt = 1.0 / rate
        self._t_last    = self.get_clock().now().nanoseconds * 1e-9
        self._timer = self.create_timer(dt, self._update)

        gz_method = 'ignition.transport Python' if _USE_IGN_PYTHON else f'subprocess ({_GZ_CMD})'
        self.get_logger().info(
            f'SimTargetNode: tapis Y@x={CONVEYOR_X}m, vitesse={self._speed} m/s, '
            f'pick=(x={CONVEYOR_X},y={PICK_ZONE_Y}), depot=({DEPOSIT_X},{DEPOSIT_Y},{DEPOSIT_Z}) '
            f'| set_pose via: {gz_method}'
        )

    # -------------------------------------------------------------------------

    def _gz_move_object(self, x: float, y: float, z: float):
        """Deplace la sphere grasp_object dans Gazebo (thread daemon).

        Strategie :
          1. bindings Python ignition.transport (meme partition que Gazebo -> fiable)
          2. subprocess 'ign service' en fallback (timeout 2000 ms)
        """
        if self._gz_busy:
            return
        self._gz_busy = True
        logger = self.get_logger()

        def _run():
            success = False
            try:
                # -- Approche 1 : bindings Python (in-process, meme transport que Gazebo)
                if _USE_IGN_PYTHON and _IGN_NODE is not None:
                    try:
                        req_py = _IgPose()
                        req_py.name = 'grasp_object'
                        req_py.position.x = float(x)
                        req_py.position.y = float(y)
                        req_py.position.z = float(z)
                        req_py.orientation.w = 1.0
                        ok, rep = _IGN_NODE.request(
                            f'/world/{GZ_WORLD_NAME}/set_pose',
                            req_py, _IgPose, _IgBool, 500
                        )
                        success = ok and rep.data
                    except Exception as e_py:
                        logger.warn(
                            f'ign-python set_pose error (1er appel?): {e_py}'
                        )

                # -- Approche 2 : subprocess (fallback si Python KO)
                if not success:
                    req_str = (
                        f'name: "grasp_object" '
                        f'position {{x: {x:.4f} y: {y:.4f} z: {z:.4f}}} '
                        f'orientation {{w: 1.0}}'
                    )
                    result = subprocess.run(
                        [_GZ_CMD, 'service',
                         '-s', f'/world/{GZ_WORLD_NAME}/set_pose',
                         '--reqtype', f'{_GZ_MSG_NS}.msgs.Pose',
                         '--reptype', f'{_GZ_MSG_NS}.msgs.Boolean',
                         '--timeout', '2000',
                         '--req', req_str],
                        timeout=4.0, capture_output=True, check=False,
                        env=os.environ.copy(),
                    )
                    stdout = result.stdout.decode(errors='ignore')
                    success = result.returncode == 0 and 'data: true' in stdout
                    if not success:
                        self._gz_err_count += 1
                        if self._gz_err_count <= 5:
                            err = result.stderr.decode(errors='ignore')[:80]
                            logger.warn(
                                f'set_pose #{self._gz_err_count} FAILED '
                                f'rc={result.returncode} '
                                f'out=[{stdout[:80].strip()}] {err.strip()}'
                            )
            except Exception as exc:
                self._gz_err_count += 1
                if self._gz_err_count <= 5:
                    logger.warn(f'set_pose exception: {exc}')
            finally:
                self._gz_busy = False

        threading.Thread(target=_run, daemon=True).start()

    # -------------------------------------------------------------------------

    def _cb_tcp_pose(self, msg: PoseStamped):
        """Stocke la position courante du TCP du bras (pour phase CARRYING)."""
        p = msg.pose.position
        self._tcp_pose = np.array([p.x, p.y, p.z])

    def _cb_status(self, msg: String):
        status = msg.data
        if 'PICKED' in status and self._state in (self.STATE_PAUSED, self.STATE_CONVEYOR):
            self._state = self.STATE_PICKED
            self.get_logger().info('Status PICKED recu - objet en transit')
        elif 'DEPOSITED' in status and self._state == self.STATE_PICKED:
            self._state = self.STATE_DEPOSITED
            self._deposit_t0 = self.get_clock().now().nanoseconds * 1e-9 # CHRONO DEMARRE ICI
            self.get_logger().info('Status DEPOSITED recu - depot en cours')
    # -------------------------------------------------------------------------

    def _update(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        dt  = now - self._t_last
        self._t_last = now

        stamp = self.get_clock().now().to_msg()

        # -- Tapis roulant --------------------------------------------------
        if self._state == self.STATE_CONVEYOR:
            self._obj_y += self._speed * dt
            
            # Entrer en zone de saisie -> pause tapis
            if abs(self._obj_y - PICK_ZONE_Y) < 0.05:
                self._state   = self.STATE_PAUSED
                self._pause_t0 = now
                self.get_logger().info(
                    f'Objet en zone de saisie (y={self._obj_y:.3f}) - pause {PICK_PAUSE}s')

            # Objet sorti du cote haut -> reset
            elif self._obj_y > OBJ_END_Y:
                self._obj_y = OBJ_START_Y
                self.get_logger().info('Objet remis en debut de tapis')

        elif self._state == self.STATE_DEPOSITED:
            # Attendre 2.5 secondes que le bras s'éloigne avant de faire disparaître la boule
            if now - getattr(self, '_deposit_t0', 0.0) > 2.5:
                self._obj_y = OBJ_START_Y
                self._state = self.STATE_CONVEYOR
                self.get_logger().info('Nouveau cycle: objet repart sur le tapis')

        elif self._state == self.STATE_DEPOSITED:
            # Reset: nouvel objet sur le tapis
            self._obj_y = OBJ_START_Y
            self._state = self.STATE_CONVEYOR
            self.get_logger().info('Nouveau cycle: objet repart sur le tapis')

        # -- Publication pose cible (seulement si objet visible) -----------
        if self._state in (self.STATE_CONVEYOR, self.STATE_PAUSED):
            pose = self._make_posestamped(stamp, CONVEYOR_X, self._obj_y, CONVEYOR_Z)
            self._pub_target.publish(pose)
            self._publish_object_marker(stamp, CONVEYOR_X, self._obj_y, OBJ_VISUAL_Z)
        # -- Point de depot (toujours publie) --------------------------------
        dep = self._make_posestamped(stamp, DEPOSIT_X, DEPOSIT_Y, DEPOSIT_Z)
        self._pub_deposit.publish(dep)
        self._publish_deposit_marker(stamp)
        self._publish_conveyor_marker(stamp)

        # -- Mise a jour position sphere Gazebo
        self._gz_tick += 1
        
        if self._state == self.STATE_PICKED:
            # EFFET AIMANT : On téléporte à 10 Hz pour contrer la gravité
            if self._gz_tick % 5 == 0:
                self._gz_move_object(
                    float(self._tcp_pose[0]),
                    float(self._tcp_pose[1]),
                    float(self._tcp_pose[2] - 0.03), # La boule s'accroche 3cm sous la pince
                )
        else:
            # Économie de CPU (1 Hz) quand la boule est sur le tapis ou au sol
            if self._gz_tick % 20 == 0:
                if self._state in (self.STATE_CONVEYOR, self.STATE_PAUSED):
                    self._gz_move_object(CONVEYOR_X, self._obj_y, OBJ_VISUAL_Z)
                elif self._state == self.STATE_DEPOSITED:
                    self._gz_move_object(DEPOSIT_X, DEPOSIT_Y, OBJ_VISUAL_Z)

    # -------------------------------------------------------------------------

    def _make_posestamped(self, stamp, x, y, z) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp    = stamp
        msg.header.frame_id = 'base_link'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 1.0
        return msg

    def _publish_object_marker(self, stamp, x, y, z):
        mk = Marker()
        mk.header.stamp    = stamp
        mk.header.frame_id = 'base_link'
        mk.ns = 'vs_object'; mk.id = 0
        mk.type   = Marker.SPHERE
        mk.action = Marker.ADD
        mk.pose.position.x = float(x)
        mk.pose.position.y = float(y)
        mk.pose.position.z = float(z)
        mk.pose.orientation.w = 1.0
        mk.scale.x = mk.scale.y = mk.scale.z = 0.06
        mk.color = ColorRGBA(r=0.95, g=0.20, b=0.05, a=0.95)
        mk.lifetime = Duration(sec=1, nanosec=0)
        self._pub_obj_mk.publish(mk)

    def _publish_deposit_marker(self, stamp):
        mk = Marker()
        mk.header.stamp    = stamp
        mk.header.frame_id = 'base_link'
        mk.ns = 'vs_deposit'; mk.id = 1
        mk.type   = Marker.CYLINDER
        mk.action = Marker.ADD
        mk.pose.position.x = DEPOSIT_X
        mk.pose.position.y = DEPOSIT_Y
        mk.pose.position.z = DEPOSIT_Z - 0.01
        mk.pose.orientation.w = 1.0
        mk.scale.x = mk.scale.y = 0.10
        mk.scale.z = 0.02
        mk.color = ColorRGBA(r=0.05, g=0.40, b=0.95, a=0.80)
        mk.lifetime = Duration(sec=2, nanosec=0)
        self._pub_dep_mk.publish(mk)

    def _publish_conveyor_marker(self, stamp):
        # Tapis roulant: boite verte allongee le long de Y
        mk = Marker()
        mk.header.stamp    = stamp
        mk.header.frame_id = 'base_link'
        mk.ns = 'vs_conveyor'; mk.id = 2
        mk.type   = Marker.CUBE
        mk.action = Marker.ADD
        mk.pose.position.x = CONVEYOR_X
        mk.pose.position.y = (OBJ_START_Y + OBJ_END_Y) / 2.0
        mk.pose.position.z = CONVEYOR_Z / 2.0
        mk.pose.orientation.w = 1.0
        mk.scale.x = 0.12
        mk.scale.y = abs(OBJ_START_Y - OBJ_END_Y)
        mk.scale.z = CONVEYOR_Z
        mk.color = ColorRGBA(r=0.15, g=0.60, b=0.15, a=0.50)
        mk.lifetime = Duration(sec=2, nanosec=0)
        self._pub_conv_mk.publish(mk)


# =============================================================================
def main(args=None):
    rclpy.init(args=args)
    node = SimTargetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
