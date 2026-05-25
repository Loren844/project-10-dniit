"""
robot_transform.py
Transformation de la pose estimée dans le repère caméra → repère robot.

Contexte : la caméra est fixe au-dessus du tapis roulant (eye-to-hand).
La matrice T_cam_to_robot (4×4) est déterminée une seule fois par calibration
main-œil (hand-eye calibration) ou mesure géométrique directe.

Modules :
  - RobotTransform  : encapsule T_cam_to_robot et fournit transform()
  - hand_eye_calibrate() : estime T_cam_to_robot depuis des paires de poses
  - save/load_transform  : sauvegarde/chargement dans .npz

Usage :
    from robot_transform import RobotTransform, load_transform
    from pose_estimation  import Pose6D

    # Charger la transformation
    tf = load_transform("calibration_data/cam_to_robot.npz")

    # Convertir une pose caméra en pose robot
    pose_robot = tf.transform(pose_cam)
    print(pose_robot.position_m)   # [x, y, z] en mètres, repère robot
"""

from __future__ import annotations

import os
from dataclasses import dataclass

import numpy as np

from pose_estimation import Pose6D


# ---------------------------------------------------------------------------
@dataclass
class RobotTransform:
    """
    Encapsule la matrice extrinsèque T_cam_to_robot.

    T_cam_to_robot exprime le repère caméra dans le repère robot :
        P_robot = T_cam_to_robot @ P_cam

    Attributs
    ---------
    T : np.ndarray (4, 4)
        Matrice homogène caméra → robot
    """
    T: np.ndarray   # (4, 4)

    def transform(self, pose_cam: Pose6D) -> Pose6D:
        """
        Convertit une Pose6D du repère caméra vers le repère robot.

        Paramètres
        ----------
        pose_cam : Pose6D dans le repère caméra

        Retourne
        --------
        Pose6D dans le repère robot
        """
        T_obj_cam   = pose_cam.T_cam               # 4×4
        T_obj_robot = self.T @ T_obj_cam            # 4×4

        R_robot = T_obj_robot[:3, :3]
        t_robot = T_obj_robot[:3,  3]
        return Pose6D(label=pose_cam.label, R_cam=R_robot, t_cam=t_robot)

    def transform_point(self, point_cam: np.ndarray) -> np.ndarray:
        """
        Transforme un point 3D [x, y, z] du repère caméra vers le repère robot.
        """
        p_h = np.append(point_cam, 1.0)
        return (self.T @ p_h)[:3]

    def inverse(self) -> "RobotTransform":
        """Retourne la transformation inverse (robot → caméra)."""
        T_inv = np.eye(4)
        R = self.T[:3, :3]
        t = self.T[:3, 3]
        T_inv[:3, :3] = R.T
        T_inv[:3,  3] = -R.T @ t
        return RobotTransform(T=T_inv)


# ---------------------------------------------------------------------------
# --- Création d'une transformation géométrique simple ---------------------
# ---------------------------------------------------------------------------

def make_transform_from_geometry(
    translation_m: np.ndarray,
    rotation_deg: np.ndarray,
    rotation_order: str = "ZYX",
) -> RobotTransform:
    """
    Construit T_cam_to_robot depuis une translation et des angles d'Euler.

    Utile quand la position de la caméra est connue géométriquement
    (mesure directe, dessin CAO).

    Paramètres
    ----------
    translation_m  : [tx, ty, tz] en mètres (position caméra dans repère robot)
    rotation_deg   : [rz, ry, rx] en degrés (angles d'Euler ZYX)
    rotation_order : 'ZYX' (défaut, Tait-Bryan aéronautique)

    Retourne
    --------
    RobotTransform
    """
    t = np.array(translation_m, dtype=float)
    rz, ry, rx = np.radians(rotation_deg)

    # Matrices de rotation élémentaires
    Rx = np.array([[1,      0,       0     ],
                   [0,  np.cos(rx), -np.sin(rx)],
                   [0,  np.sin(rx),  np.cos(rx)]])
    Ry = np.array([[ np.cos(ry), 0, np.sin(ry)],
                   [0,           1, 0          ],
                   [-np.sin(ry), 0, np.cos(ry)]])
    Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                   [np.sin(rz),  np.cos(rz), 0],
                   [0,           0,          1]])

    R = Rz @ Ry @ Rx   # Rotation composée ZYX

    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = t
    return RobotTransform(T=T)


# ---------------------------------------------------------------------------
# --- Calibration main-œil (eye-to-hand) ------------------------------------
# ---------------------------------------------------------------------------

def hand_eye_calibrate(
    R_gripper2base_list: list[np.ndarray],
    t_gripper2base_list: list[np.ndarray],
    R_target2cam_list:   list[np.ndarray],
    t_target2cam_list:   list[np.ndarray],
    method: int = None,
) -> RobotTransform:
    """
    Calibration main-œil (eye-to-hand) via cv2.calibrateHandEye.

    Protocole :
    1. Placer un marqueur ArUco de taille connue dans l'espace de travail.
    2. Déplacer le robot en N configurations (N ≥ 5).
    3. À chaque configuration, enregistrer :
       - la pose du robot (gripper dans repère base)   → R_gripper2base, t_gripper2base
       - la pose du marqueur dans la caméra            → R_target2cam,   t_target2cam
         (obtenue via estimate_pose_aruco)

    Paramètres
    ----------
    R_gripper2base_list : N × (3,3) rotations robot
    t_gripper2base_list : N × (3,)  translations robot (mètres)
    R_target2cam_list   : N × (3,3) rotations marqueur→caméra
    t_target2cam_list   : N × (3,)  translations marqueur→caméra (mètres)
    method              : méthode cv2 (défaut : TSAI)

    Retourne
    --------
    RobotTransform (T_cam_to_robot)
    """
    import cv2
    if method is None:
        method = cv2.CALIB_HAND_EYE_TSAI

    R_cam2robot, t_cam2robot = cv2.calibrateHandEye(
        R_gripper2base_list,
        t_gripper2base_list,
        R_target2cam_list,
        t_target2cam_list,
        method=method,
    )
    T = np.eye(4)
    T[:3, :3] = R_cam2robot
    T[:3,  3] = t_cam2robot.ravel()
    return RobotTransform(T=T)


# ---------------------------------------------------------------------------
# --- Sauvegarde / chargement -----------------------------------------------
# ---------------------------------------------------------------------------

def save_transform(tf: RobotTransform, path: str):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    np.savez(path, T=tf.T)
    print(f"Transformation sauvegardée : {path}")


def load_transform(path: str) -> RobotTransform:
    data = np.load(path)
    return RobotTransform(T=data["T"])


def print_transform_report(tf: RobotTransform):
    import cv2
    T = tf.T
    t_mm = T[:3, 3] * 1000
    rvec, _ = cv2.Rodrigues(T[:3, :3])
    euler_deg = np.degrees(rvec.ravel())
    print("\n" + "=" * 55)
    print("  TRANSFORMATION CAMÉRA → ROBOT")
    print("=" * 55)
    print(f"  Translation : ({t_mm[0]:.1f}, {t_mm[1]:.1f}, {t_mm[2]:.1f}) mm")
    print(f"  Rotation    : ({euler_deg[0]:.2f}, {euler_deg[1]:.2f}, {euler_deg[2]:.2f})°")
    print("  Matrice T :")
    for row in T:
        print("   ", "  ".join(f"{v:8.4f}" for v in row))
    print("=" * 55)


# ---------------------------------------------------------------------------
# --- Test rapide ------------------------------------------------------------
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    # Exemple : caméra positionnée à 800 mm au-dessus du plan de travail,
    # décalée de 200 mm vers X, face vers le bas (rotation de -90° autour de X)
    tf = make_transform_from_geometry(
        translation_m = [0.2, 0.0, 0.8],
        rotation_deg  = [0.0, 0.0, -90.0],
    )
    print_transform_report(tf)

    # Test aller-retour
    p_cam   = np.array([0.1, -0.05, 0.7])
    p_robot = tf.transform_point(p_cam)
    p_back  = tf.inverse().transform_point(p_robot)
    print(f"\nPoint caméra  : {p_cam * 1000} mm")
    print(f"Point robot   : {p_robot * 1000} mm")
    print(f"Aller-retour  : {p_back * 1000} mm  (erreur = {np.linalg.norm(p_back - p_cam)*1e6:.3f} µm)")
