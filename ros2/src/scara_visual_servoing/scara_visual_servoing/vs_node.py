#!/usr/bin/env python3
"""
vs_node.py — Nœud ROS2 d'asservissement visuel PBVS pour SCARA 4-DOF
═══════════════════════════════════════════════════════════════════════
Encapsule le pipeline Phase 3 (vs_controller + visual_error + gripper)
dans un nœud ROS2 Humble.

Modes de fonctionnement
───────────────────────
  sim       : cible virtuelle publiée par sim_target_node (sans caméra)
  live      : flux webcam / ArUco (nécessite cv_bridge)
  realsense : flux RealSense D435 via topic ROS2 (topic /camera/color/image_raw)

Topics abonnés
──────────────
  /joint_states           (sensor_msgs/JointState)   — état courant du robot
  /vs/target_pose         (geometry_msgs/PoseStamped) — cible simulée (mode sim)
  /camera/color/image_raw (sensor_msgs/Image)         — flux caméra (mode live)

Topics publiés
──────────────
  /joint_trajectory_controller/joint_trajectory
                          (trajectory_msgs/JointTrajectory) — commande position
  /vs/status              (std_msgs/String)                 — état de la boucle VS
  /vs/error               (geometry_msgs/Vector3)           — norme erreur (m)

Paramètres
──────────
  mode          : str  "sim" | "live" | "realsense"  (défaut : "sim")
  dt            : float  période de contrôle (s)     (défaut : 0.033 ≈ 30 Hz)
  gain          : float  gain λ initial VS            (défaut : 0.5)
  adaptive_gain : bool   gain adaptatif Chaumette     (défaut : true)
  marker_size   : float  taille marqueur ArUco (m)    (défaut : 0.08)
  cam_index     : int    index caméra USB             (défaut : 0)
  target_x      : float  cible simulée X (m)          (défaut : 0.35)
  target_y      : float  cible simulée Y (m)          (défaut : 0.05)
  target_z      : float  cible simulée Z (m)          (défaut : -0.12)
"""
from __future__ import annotations

import os
import sys
import math
import time
from pathlib import Path

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg       import JointState, Image
from trajectory_msgs.msg   import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg     import PoseStamped, Vector3
from std_msgs.msg          import String
from builtin_interfaces.msg import Duration, Time

# ── Chemin vers Phase 3 ────────────────────────────────────────────────────
_THIS_DIR   = Path(__file__).resolve().parent
_PHASE3_DIR = _THIS_DIR.parents[3] / 'python' / 'phase3'
_PHASE2_DIR = _THIS_DIR.parents[3] / 'python' / 'phase2'

for p in [str(_PHASE3_DIR), str(_PHASE2_DIR)]:
    if p not in sys.path:
        sys.path.insert(0, p)

from vs_controller  import VSController, ScaraParams, ik_solutions, scara_jacobian
from visual_error   import compute_error, VisualError
from gripper_controller import PickPlaceSequencer, PickPlaceState


# ═══════════════════════════════════════════════════════════════════════════
# Cinématique directe (FK) — SCARA 4-DOF
# ═══════════════════════════════════════════════════════════════════════════

def scara_fk(q: np.ndarray, params: ScaraParams) -> tuple[np.ndarray, np.ndarray]:
    """
    Cinématique directe analytique.

    Retourne (t [3], R [3×3]) :
      t = [px, py, pz]
      R = Rot_Z(θ1+θ3+θ4)   convention SCARA, cohérente avec la Jacobienne
          (dω/dθ1 = dω/dθ3 = dω/dθ4 = [0,0,1] ⇒ Ω = θ1+θ3+θ4)

    Vérification :
      px = a2·cos(θ1) + a3·cos(θ1+θ3)
      py = a2·sin(θ1) + a3·sin(θ1+θ3)
      pz = d2 − d3 − d4
    """
    t1, d2, t3, t4 = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    a2, a3 = params.a2, params.a3
    d3, d4 = params.d3, params.d4

    px = a2 * math.cos(t1) + a3 * math.cos(t1 + t3)
    py = a2 * math.sin(t1) + a3 * math.sin(t1 + t3)
    pz = d2 - d3 - d4

    theta = t1 + t3 + t4
    c, s = math.cos(theta), math.sin(theta)
    R = np.array([[c, -s, 0.0],
                  [s,  c, 0.0],
                  [0.0, 0.0, 1.0]])   # Rot_Z(θ_total)

    return np.array([px, py, pz]), R


# ═══════════════════════════════════════════════════════════════════════════
# Nœud principal
# ═══════════════════════════════════════════════════════════════════════════

class ScaraVSNode(Node):
    """Nœud ROS2 d'asservissement visuel PBVS pour le SCARA 4-DOF."""

    # DH order: [theta1=joint_1theta, d2=joint_1z, theta3=joint3, theta4=joint4]
    JOINT_NAMES = ['joint_1theta', 'joint_1z', 'joint3', 'joint4']

    def __init__(self):
        super().__init__('scara_vs_node')

        # ── Déclaration des paramètres ────────────────────────────────────
        self.declare_parameter('mode',          'sim')
        self.declare_parameter('dt',            0.033)
        self.declare_parameter('gain',          0.5)
        self.declare_parameter('adaptive_gain', True)
        self.declare_parameter('marker_size',   0.08)
        self.declare_parameter('cam_index',     0)
        self.declare_parameter('target_x',      0.35)
        self.declare_parameter('target_y',      0.15)
        self.declare_parameter('target_z',      0.05)

        mode          = self.get_parameter('mode').value
        self.dt       = self.get_parameter('dt').value
        gain          = self.get_parameter('gain').value
        adaptive      = self.get_parameter('adaptive_gain').value

        # ── Objets Phase 3 ────────────────────────────────────────────────
        # Parametres URDF: a2=0.4, a3=0.3, d3=0.1, d4=0.15
        self.params = ScaraParams(
            a2=0.4, a3=0.3, d3=0.1, d4=0.15,
            q_min=np.array([-2.356, 0.0,  -1.571, -3.142]),
            q_max=np.array([ 2.356, 0.4,   1.571,  3.142]),
            dq_max=np.array([2.0,   0.2,   2.0,    2.0]),  # joint_1z max 0.2 m/s
        )
        self.controller = VSController(
            params=self.params,
            gain=gain,
            adaptive=adaptive,
        )
        self.sequencer  = PickPlaceSequencer(self.params)

        # ── État courant (position de repos par défaut) ───────────────────
        # q = [theta1, d2, theta3, theta4]
        # home: joint_1z=0.40 (initial URDF), joint3=0.3 -> hors singularite
        self.q = np.array([0.0, 0.40, 0.3, 0.0])
        self._joint_state_received = False

        # ── Pose cible (mode sim : lue sur topic ou paramètre) ────────────
        self.t_des: np.ndarray | None = None
        self.R_des: np.ndarray | None = None
        self._load_default_target()

        # Pick-and-place state machine
        # Phases: TRACKING -> WAIT_PICK -> CARRYING -> WAIT_DEP -> TRACKING
        self._phase   = 'TRACKING'
        self._hold_t0 = 0.0
        # Depot face au robot, a 90 deg du tapis (mis a jour par /vs/deposit_pose)
        self._deposit_pose = np.array([0.0, -0.55, 0.05])

        # ── QoS ──────────────────────────────────────────────────────────
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10,
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10,
        )

        # ── Abonnements ───────────────────────────────────────────────────
        # joint_state_broadcaster publie en BEST_EFFORT (sensor data QoS)
        self.create_subscription(
            JointState,
            '/joint_states',
            self._cb_joint_states,
            qos_best_effort,
        )
        self.create_subscription(
            PoseStamped,
            '/vs/target_pose',
            self._cb_target_pose,
            qos_reliable,
        )
        self.create_subscription(
            PoseStamped,
            '/vs/deposit_pose',
            self._cb_deposit_pose,
            qos_reliable,
        )

        if mode in ('live', 'realsense'):
            topic = ('/camera/color/image_raw'
                     if mode == 'realsense'
                     else '/camera/image_raw')
            try:
                from cv_bridge import CvBridge
                self._bridge = CvBridge()
                self.create_subscription(
                    Image, topic, self._cb_image, qos_best_effort)
                self.get_logger().info(f'Mode caméra activé — topic : {topic}')
            except ImportError:
                self.get_logger().warn(
                    'cv_bridge non disponible — basculement en mode sim.')
                mode = 'sim'
        else:
            self._bridge = None

        # ── Publications ─────────────────────────────────────────────────
        self._pub_traj = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10,
        )
        self._pub_status = self.create_publisher(String, '/vs/status', 10)
        self._pub_error  = self.create_publisher(Vector3, '/vs/error',  10)
        self._pub_tcp    = self.create_publisher(PoseStamped, '/vs/tcp_pose', 10)

        # ── Timer de contrôle ─────────────────────────────────────────────
        self._timer = self.create_timer(self.dt, self._control_loop)

        # ── Compteurs et logs ─────────────────────────────────────────────
        self._loop_count = 0
        self._converged  = False

        self.get_logger().info(
            f'ScaraVSNode démarré — mode={mode}, dt={self.dt:.3f}s, '
            f'gain={gain}, adaptive={adaptive}'
        )

    # ──────────────────────────────────────────────────────────────────────
    # Initialisations
    # ──────────────────────────────────────────────────────────────────────

    def _load_default_target(self):
        """Charge la cible depuis les paramètres (mode sim)."""
        tx = self.get_parameter('target_x').value
        ty = self.get_parameter('target_y').value  # conveyor Y (0.15)
        tz = self.get_parameter('target_z').value  # pick height (0.05)
        self.t_des = np.array([tx, ty, tz])
        # Orientation désirée : outil pointant vers le bas, rotation nulle
        self.R_des = np.eye(3)
        self.get_logger().info(
            f'Cible simulée : [{tx:.3f}, {ty:.3f}, {tz:.3f}] m')

    # ──────────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────────

    def _cb_joint_states(self, msg: JointState):
        """Met à jour l'état articulaire courant depuis /joint_states."""
        pos_map = dict(zip(msg.name, msg.position))
        for i, name in enumerate(self.JOINT_NAMES):
            if name in pos_map:
                self.q[i] = pos_map[name]
        self._joint_state_received = True

    def _cb_target_pose(self, msg: PoseStamped):
        """Met à jour la pose cible depuis /vs/target_pose."""
        # Ignorer si le robot n'est pas en phase de suivi
        if self._phase not in ('TRACKING',):
            return
        p = msg.pose.position
        new_t = np.array([p.x, p.y, p.z])
        quat = msg.pose.orientation
        new_R = _quat_to_rot(quat.x, quat.y, quat.z, quat.w)
        # Réinitialiser VS seulement si la cible a sauté > 50 mm
        # (ex: reset tapis apres depot). Ne pas réinitialiser pour les
        # micro-déplacements du tapis (2 mm/tick a 20 Hz) — cela empecherait
        # la machine d'etats de passer en WAIT_PICK.
        if self._converged and self.t_des is not None:
            jump = float(np.linalg.norm(new_t - self.t_des))
            if jump > 0.05:  # > 50 mm : vrai changement de cible
                self._converged = False
                self.get_logger().info(
                    f'Nouvelle cible (+{jump*1000:.0f}mm) — réinitialisation VS')
        self.t_des = new_t
        self.R_des = new_R

    def _cb_deposit_pose(self, msg):
        """Met a jour le point de depot depuis /vs/deposit_pose."""
        p = msg.pose.position
        self._deposit_pose = np.array([p.x, p.y, p.z])

    def _cb_image(self, msg: Image):
        """
        Traitement image caméra (mode live/realsense).
        Détecte le marqueur ArUco et met à jour la pose cible.
        """
        if self._bridge is None:
            return
        try:
            import cv2
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            pose = _detect_aruco_pose(
                frame,
                marker_size=self.get_parameter('marker_size').value,
            )
            if pose is not None:
                t_marker, R_marker = pose
                # Pose cible = pose du marqueur (ici on vise directement le marqueur)
                self.t_des = t_marker
                self.R_des = R_marker
        except Exception as exc:
            self.get_logger().warn(f'Erreur traitement image : {exc}')

    # ──────────────────────────────────────────────────────────────────────
    # Boucle de contrôle
    # ──────────────────────────────────────────────────────────────────────

    def _control_loop(self):
        """Boucle PBVS — appelée à la fréquence 1/dt."""
        self._loop_count += 1

        # Attendre au moins un état articulaire réel
        if not self._joint_state_received:
            if self._loop_count % 30 == 0:
                self.get_logger().warn(
                    'En attente de /joint_states...')
            return

        if self.t_des is None or self.R_des is None:
            return

        # ── Cinématique directe (toujours calculée pour pub TCP) ──────────
        t_cur, R_cur = scara_fk(self.q, self.params)
        # Publier position TCP : permet a sim_target_node de faire suivre la sphere
        _tcp_msg = PoseStamped()
        _tcp_msg.header.stamp = self.get_clock().now().to_msg()
        _tcp_msg.header.frame_id = 'base_link'
        _tcp_msg.pose.position.x = float(t_cur[0])
        _tcp_msg.pose.position.y = float(t_cur[1])
        _tcp_msg.pose.position.z = float(t_cur[2])
        _tcp_msg.pose.orientation.w = 1.0
        self._pub_tcp.publish(_tcp_msg)

        # Pick-and-place state machine
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._converged:
            if self._phase == 'TRACKING':
                self._phase = 'WAIT_PICK'
                self._hold_t0 = now
                self._pub_status.publish(String(data='PICKED'))
                self.get_logger().info('Objet saisi - maintien 1s')
            elif self._phase == 'WAIT_PICK':
                if now - self._hold_t0 >= 1.0:
                    self._phase = 'CARRYING'
                    self._converged = False
                    self.t_des = self._deposit_pose.copy()
                    self.R_des = np.eye(3)
                    self.get_logger().info('Transport vers depot: '
                        f'({self._deposit_pose[0]:.2f}, '
                        f'{self._deposit_pose[1]:.2f}, '
                        f'{self._deposit_pose[2]:.2f})')
            elif self._phase == 'CARRYING':
                self._phase = 'WAIT_DEP'
                self._hold_t0 = now
                self._pub_status.publish(String(data='DEPOSITED'))
                self.get_logger().info('Objet depose - pause 1.5s')
            elif self._phase == 'WAIT_DEP':
                if now - self._hold_t0 >= 1.5:
                    self._phase = 'TRACKING'
                    self._converged = False
                    self.get_logger().info('Cycle termine - attente prochain objet')
            return


        # ── Erreur de pose (t_cur, R_cur calculés en haut de la boucle) ──
        try:
            error: VisualError = compute_error(t_cur, R_cur, self.t_des, self.R_des)
        except Exception as exc:
            self.get_logger().error(f'compute_error : {exc}')
            return

        # ── Publication des métriques ─────────────────────────────────────
        self._pub_error.publish(
            Vector3(x=float(error.norm_t_mm),
                    y=float(error.norm_r_deg),
                    z=0.0)
        )

        # ── Convergence ───────────────────────────────────────────────────
        if error.converged:
            self._converged = True
            status_msg = f'CONVERGED phase={self._phase}'
            self._pub_status.publish(String(data=status_msg))
            self.get_logger().info(
                f'Convergence: |et|={error.norm_t_mm:.1f}mm '
                f'phase={self._phase}')
            return

        # ── Contrôleur VS ─────────────────────────────────────────────────
        try:
            cmd = self.controller.update(error, self.q, self.dt)
        except Exception as exc:
            self.get_logger().error(f'VSController.update : {exc}')
            self._pub_status.publish(String(data='CONTROLLER_ERROR'))
            return

        if cmd.singular:
            self._pub_status.publish(String(data='SINGULAR'))
            self.get_logger().warn(
                f'Singularité détectée — σ_min={cmd.sigma_min:.4f}')

        # ── Intégration Euler : q_next = q + dq·dt ───────────────────────
        q_next = self.q + cmd.dq * self.dt

        # ── Saturation des limites articulaires ───────────────────────────
        q_next[0] = np.clip(q_next[0],
                            self.params.q_min[0], self.params.q_max[0])
        q_next[1] = np.clip(q_next[1],
                            self.params.q_min[1], self.params.q_max[1])
        q_next[2] = np.clip(q_next[2],
                            self.params.q_min[2], self.params.q_max[2])
        q_next[3] = np.clip(q_next[3],
                            self.params.q_min[3], self.params.q_max[3])

        # ── Publication de la trajectoire ─────────────────────────────────
        self._publish_trajectory(q_next)

        # ── Statut ────────────────────────────────────────────────────────
        status = 'APPROACH' if error.norm_t_mm < 50.0 else 'TRACKING'
        if cmd.saturated:
            status += '_SAT'
        self._pub_status.publish(String(data=status))

    # ──────────────────────────────────────────────────────────────────────
    # Publication
    # ──────────────────────────────────────────────────────────────────────

    def _publish_trajectory(self, q_target: np.ndarray):
        """
        Publie une trajectoire à un seul point (position cible dans dt secondes).
        Le joint_trajectory_controller exécutera le mouvement.
        """
        msg = JointTrajectory()
        # stamp=0 : le JTC interprete la trajectoire comme "demarrer maintenant"
        # (evite l'erreur "ends in the past" causee par la latence ROS/sim)
        msg.header.stamp = Time(sec=0, nanosec=0)
        msg.joint_names  = self.JOINT_NAMES

        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q_target]
        pt.velocities = [0.0] * 4
        # Fenetre d'execution : 2x dt (66ms) suffit avec stamp=0
        # Ne pas depasser 100ms sinon la boucle VS devient trop lente
        duration_s = max(self.dt * 1.2, 0.040)  # fenetre d'execution plus courte = robot plus reactif
        pt.time_from_start = Duration(sec=0, nanosec=int(duration_s * 1e9))

        msg.points = [pt]
        self._pub_traj.publish(msg)


# ═══════════════════════════════════════════════════════════════════════════
# Helpers
# ═══════════════════════════════════════════════════════════════════════════

def _quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Quaternion (x,y,z,w) → matrice de rotation 3×3."""
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if n < 1e-10:
        return np.eye(3)
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n
    return np.array([
        [1-2*(qy*qy+qz*qz),   2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [  2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz),   2*(qy*qz-qx*qw)],
        [  2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)],
    ])


def _detect_aruco_pose(
    frame: 'np.ndarray',
    marker_size: float = 0.08,
) -> 'tuple[np.ndarray, np.ndarray] | None':
    """
    Détecte le premier marqueur ArUco dans frame.
    Retourne (t_marker [3], R_marker [3×3]) dans le repère caméra,
    ou None si non détecté.

    Note : calibration caméra factice (à remplacer par les paramètres réels).
    """
    try:
        import cv2
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        params     = cv2.aruco.DetectorParameters()
        detector   = cv2.aruco.ArucoDetector(aruco_dict, params)
        corners, ids, _ = detector.detectMarkers(frame)

        if ids is None or len(ids) == 0:
            return None

        h, w = frame.shape[:2]
        fx = fy = 0.8 * max(h, w)        # approximation focale
        cx, cy = w / 2.0, h / 2.0
        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
        dist = np.zeros(5)

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, marker_size, K, dist)

        rvec = rvecs[0][0]
        tvec = tvecs[0][0]
        R, _ = cv2.Rodrigues(rvec)
        return tvec, R

    except Exception:
        return None


# ═══════════════════════════════════════════════════════════════════════════
# Point d'entrée
# ═══════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = ScaraVSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
