"""
pose_estimation.py
Estimation de la pose 6D (position + orientation) d'un objet dans le repère caméra.

Deux méthodes selon la source de détection :

  1. ArUco  → solvePnP exact (4 coins 2D + modèle 3D connu du marqueur)
  2. YOLO   → solvePnP approximatif sur le centre + depth (caméra RGB-D)
              ou estimation 2D (plan de travail Z=const) sans profondeur

Sortie : Pose6D
    .R_cam   np.ndarray (3, 3)   Rotation  objet → repère caméra
    .t_cam   np.ndarray (3,)     Translation objet → repère caméra (mètres)
    .T_cam   np.ndarray (4, 4)   Matrice homogène complète

Usage :
    from camera_calibration import load_calibration
    from detect_objects import detect_aruco
    from pose_estimation import estimate_pose_aruco

    cam = load_calibration("calibration_data/camera_params.npz")
    detections = detect_aruco(frame)
    poses = estimate_pose_aruco(detections, cam, marker_size_m=0.05)
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from detect_objects import Detection


# ---------------------------------------------------------------------------
@dataclass
class Pose6D:
    label:  str
    R_cam:  np.ndarray   # (3, 3)
    t_cam:  np.ndarray   # (3,)

    @property
    def T_cam(self) -> np.ndarray:
        """Matrice homogène 4×4 objet → caméra."""
        T = np.eye(4)
        T[:3, :3] = self.R_cam
        T[:3,  3] = self.t_cam
        return T

    @property
    def position_m(self) -> np.ndarray:
        """Position (x, y, z) en mètres dans le repère caméra."""
        return self.t_cam.copy()

    @property
    def euler_deg(self) -> np.ndarray:
        """Angles d'Euler ZYX en degrés (repère caméra)."""
        rvec, _ = cv2.Rodrigues(self.R_cam)
        return np.degrees(rvec.ravel())

    def __str__(self) -> str:
        p = self.position_m * 1000  # → mm
        e = self.euler_deg
        return (
            f"Pose6D [{self.label}] "
            f"pos=({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f}) mm  "
            f"euler=({e[0]:.1f}, {e[1]:.1f}, {e[2]:.1f})°"
        )


# ---------------------------------------------------------------------------
# --- Pose depuis marqueurs ArUco -------------------------------------------
# ---------------------------------------------------------------------------

def estimate_pose_aruco(
    detections: list[Detection],
    cam: dict,
    marker_size_m: float = 0.05,
) -> list[Pose6D]:
    """
    Estime la pose de chaque marqueur ArUco via solvePnP.

    Les 4 coins du marqueur dans son repère local (z = 0, origine au centre) :
        (-s/2,  s/2, 0)  top-left
        ( s/2,  s/2, 0)  top-right
        ( s/2, -s/2, 0)  bottom-right
        (-s/2, -s/2, 0)  bottom-left

    Paramètres
    ----------
    detections    : liste de Detection (méthode='aruco')
    cam           : dict issu de load_calibration()
    marker_size_m : côté du marqueur en mètres

    Retourne
    --------
    liste de Pose6D
    """
    K    = cam["K"].astype(np.float64)
    dist = cam["dist"].astype(np.float64)
    s    = marker_size_m / 2.0

    # Coins 3D dans le repère marqueur (z = 0)
    obj_pts = np.array([
        [-s,  s, 0],
        [ s,  s, 0],
        [ s, -s, 0],
        [-s, -s, 0],
    ], dtype=np.float64)

    poses: list[Pose6D] = []
    for d in detections:
        if d.method != "aruco" or d.corners is None:
            continue
        img_pts = d.corners.astype(np.float64)  # (4, 2)

        success, rvec, tvec = cv2.solvePnP(
            obj_pts, img_pts, K, dist,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        if not success:
            continue

        R, _ = cv2.Rodrigues(rvec)
        poses.append(Pose6D(
            label = d.label,
            R_cam = R,
            t_cam = tvec.ravel(),
        ))
    return poses


# ---------------------------------------------------------------------------
# --- Pose depuis YOLO + profondeur (RGB-D) ----------------------------------
# ---------------------------------------------------------------------------

def estimate_pose_yolo_rgbd(
    detections: list[Detection],
    cam: dict,
    depth_frame: np.ndarray,
    depth_scale: float = 0.001,
) -> list[Pose6D]:
    """
    Estime la pose 3D d'objets YOLO en utilisant la carte de profondeur (RealSense).

    La position estimée est le centre de masse de la région de profondeur
    dans la bounding box (médiane pour filtrer le bruit).

    Paramètres
    ----------
    detections  : liste de Detection (méthode='yolo')
    cam         : dict de calibration
    depth_frame : carte de profondeur (H × W) uint16 — valeurs en unités capteur
    depth_scale : facteur de conversion unité → mètres (RealSense D435 : 0.001)

    Retourne
    --------
    liste de Pose6D
    """
    K    = cam["K"].astype(np.float64)
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]

    poses: list[Pose6D] = []
    for d in detections:
        if d.method != "yolo":
            continue
        bx, by, bw, bh = d.bbox
        # Région de profondeur (centre 50 % de la bbox pour éviter le fond)
        margin_x = int(bw * 0.25)
        margin_y = int(bh * 0.25)
        roi = depth_frame[by + margin_y: by + bh - margin_y,
                          bx + margin_x: bx + bw - margin_x]
        valid = roi[roi > 0]
        if valid.size == 0:
            continue
        z = float(np.median(valid)) * depth_scale   # en mètres

        pcx, pcy = d.center_px
        x = (pcx - cx) * z / fx
        y = (pcy - cy) * z / fy

        # Orientation : identité (face caméra), à affiner selon le contexte
        poses.append(Pose6D(
            label = d.label,
            R_cam = np.eye(3),
            t_cam = np.array([x, y, z]),
        ))
    return poses


# ---------------------------------------------------------------------------
# --- Pose depuis YOLO sans profondeur (plan de travail Z = const) ----------
# ---------------------------------------------------------------------------

def estimate_pose_yolo_flat(
    detections: list[Detection],
    cam: dict,
    work_plane_z_m: float = 0.0,
) -> list[Pose6D]:
    """
    Estime la position 2D d'un objet YOLO en supposant qu'il se trouve
    sur un plan horizontal à hauteur connue (plan du tapis roulant).

    Position 3D calculée par reprojection inverse :
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy
    avec Z = work_plane_z_m (distance caméra → plan de travail).

    Paramètres
    ----------
    detections      : liste de Detection (méthode='yolo')
    cam             : dict de calibration
    work_plane_z_m  : distance caméra → plan de travail en mètres

    Retourne
    --------
    liste de Pose6D (R = identité, orientation non estimée)
    """
    K  = cam["K"].astype(np.float64)
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    Z = work_plane_z_m

    poses: list[Pose6D] = []
    for d in detections:
        if d.method != "yolo":
            continue
        pcx, pcy = d.center_px
        x = (pcx - cx) * Z / fx
        y = (pcy - cy) * Z / fy
        poses.append(Pose6D(
            label = d.label,
            R_cam = np.eye(3),
            t_cam = np.array([x, y, Z]),
        ))
    return poses


# ---------------------------------------------------------------------------
# --- Visualisation ----------------------------------------------------------
# ---------------------------------------------------------------------------

def draw_pose_axes(
    frame: np.ndarray,
    poses: list[Pose6D],
    cam: dict,
    axis_length_m: float = 0.03,
) -> np.ndarray:
    """
    Dessine les axes XYZ de la pose sur l'image (repère caméra → image).

    Axe X : rouge, Y : vert, Z : bleu
    """
    K    = cam["K"].astype(np.float64)
    dist = cam["dist"].astype(np.float64)
    out  = frame.copy()

    axis_pts = np.float32([
        [0, 0, 0],
        [axis_length_m, 0, 0],
        [0, axis_length_m, 0],
        [0, 0, axis_length_m],
    ])

    for p in poses:
        rvec, _ = cv2.Rodrigues(p.R_cam)
        img_pts, _ = cv2.projectPoints(axis_pts, rvec, p.t_cam, K, dist)
        img_pts = img_pts.reshape(-1, 2).astype(int)
        origin = tuple(img_pts[0])
        cv2.line(out, origin, tuple(img_pts[1]), (0, 0, 255),   3)  # X rouge
        cv2.line(out, origin, tuple(img_pts[2]), (0, 255, 0),   3)  # Y vert
        cv2.line(out, origin, tuple(img_pts[3]), (255, 0, 0),   3)  # Z bleu
        pos_mm = p.position_m * 1000
        cv2.putText(out,
                    f"[{p.label}] ({pos_mm[0]:.0f},{pos_mm[1]:.0f},{pos_mm[2]:.0f})mm",
                    (origin[0] + 5, origin[1] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    return out


# ---------------------------------------------------------------------------
# --- Test rapide ------------------------------------------------------------
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import argparse
    from camera_calibration import load_calibration
    from detect_objects import detect_aruco, draw_detections

    parser = argparse.ArgumentParser(description="Test estimation de pose ArUco")
    parser.add_argument("--image",  required=True, help="Image de test")
    parser.add_argument("--calib",  default="calibration_data/camera_params.npz")
    parser.add_argument("--size",   type=float, default=0.05, help="Taille marqueur (m)")
    args = parser.parse_args()

    frame = cv2.imread(args.image)
    if frame is None:
        print(f"Impossible de lire : {args.image}")
        raise SystemExit(1)

    cam        = load_calibration(args.calib)
    detections = detect_aruco(frame)
    poses      = estimate_pose_aruco(detections, cam, marker_size_m=args.size)

    print(f"{len(poses)} pose(s) estimée(s) :")
    for p in poses:
        print(" ", p)

    out = draw_detections(frame, detections)
    out = draw_pose_axes(out, poses, cam, axis_length_m=args.size * 0.8)
    cv2.imshow("Pose estimation", out)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
