"""
realsense_capture.py
Interface pour la caméra Intel RealSense D435 (ou D415 / D455).

Fournit deux classes :

  RealSenseCapture  — flux RGB + profondeur alignés, intrinsèques auto-chargés
  MockRealSense     — caméra virtuelle pour tester sans matériel (webcam + depth=0)

Dépendance : pip install pyrealsense2

Usage :
    from realsense_capture import RealSenseCapture, MockRealSense

    # RealSense réelle
    with RealSenseCapture(width=1280, height=720, fps=30) as cam:
        calibration = cam.get_calibration()   # → dict {K, dist, ...}
        while True:
            color, depth = cam.read()         # color BGR, depth uint16 (mm)
            if color is None: break
            # ... pipeline détection

    # Test sans matériel (webcam)
    with MockRealSense(cam_index=1) as cam:
        color, depth = cam.read()
"""

from __future__ import annotations

import warnings
from typing import Optional

import cv2
import numpy as np


# ---------------------------------------------------------------------------
# --- RealSense réelle -------------------------------------------------------
# ---------------------------------------------------------------------------

class RealSenseCapture:
    """
    Flux RGB + profondeur depuis une Intel RealSense D4xx.

    Les deux flux sont alignés (depth-to-color) : chaque pixel depth
    correspond exactement au pixel RGB de même coordonnée.

    Paramètres
    ----------
    width, height : résolution (défaut 1280×720)
    fps           : fréquence (défaut 30)
    align_to_color: si True (défaut), aligne le depth sur le repère couleur
    """

    def __init__(
        self,
        width:          int  = 1280,
        height:         int  = 720,
        fps:            int  = 30,
        align_to_color: bool = True,
    ):
        try:
            import pyrealsense2 as rs  # type: ignore
        except ImportError:
            raise ImportError(
                "Le module 'pyrealsense2' est requis.\n"
                "Installer avec : pip install pyrealsense2"
            )

        self._rs           = rs
        self._pipeline     = rs.pipeline()
        self._config       = rs.config()
        self._align        = None
        self._profile      = None
        self._align_to_color = align_to_color
        self.width         = width
        self.height        = height
        self.fps           = fps

        self._config.enable_stream(rs.stream.color, width, height,
                                   rs.format.bgr8, fps)
        self._config.enable_stream(rs.stream.depth, width, height,
                                   rs.format.z16, fps)

    # -----------------------------------------------------------------------
    def __enter__(self) -> "RealSenseCapture":
        self.start()
        return self

    def __exit__(self, *_):
        self.stop()

    # -----------------------------------------------------------------------
    def start(self):
        """Démarre le pipeline RealSense."""
        self._profile = self._pipeline.start(self._config)
        if self._align_to_color:
            self._align = self._rs.align(self._rs.stream.color)

        # Laisser l'auto-exposition converger (~30 frames)
        for _ in range(30):
            self._pipeline.wait_for_frames()

        print(f"RealSense démarrée : {self.width}×{self.height} @ {self.fps} fps")

    def stop(self):
        """Arrête le pipeline."""
        self._pipeline.stop()

    # -----------------------------------------------------------------------
    def read(self) -> tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Lit un frame aligné (color + depth).

        Retourne
        --------
        color : np.ndarray (H, W, 3) BGR  — ou None si timeout
        depth : np.ndarray (H, W)  uint16 — valeur en mm (0 = invalide)
        """
        try:
            frames = self._pipeline.wait_for_frames(timeout_ms=5000)
        except RuntimeError:
            return None, None

        if self._align:
            frames = self._align.process(frames)

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return None, None

        color = np.asanyarray(color_frame.get_data())   # (H, W, 3) BGR
        depth = np.asanyarray(depth_frame.get_data())   # (H, W) uint16, mm

        return color, depth

    # -----------------------------------------------------------------------
    def get_depth_scale(self) -> float:
        """Facteur de conversion unité depth → mètres (typiquement 0.001)."""
        sensor = self._profile.get_device().first_depth_sensor()
        return sensor.get_depth_scale()

    # -----------------------------------------------------------------------
    def get_calibration(self) -> dict:
        """
        Retourne les paramètres intrinsèques de la caméra couleur
        au format attendu par pose_estimation.py.

        Ne nécessite PAS de calibration manuelle — les paramètres sont
        stockés dans le firmware de la caméra.

        Retourne
        --------
        dict {K, dist, rms, img_width, img_height}
        """
        stream = self._profile.get_stream(self._rs.stream.color)
        intr   = stream.as_video_stream_profile().get_intrinsics()

        K = np.array([
            [intr.fx,  0,       intr.ppx],
            [0,        intr.fy, intr.ppy],
            [0,        0,       1       ],
        ], dtype=np.float64)

        # Coefficients de distorsion Brown-Conrady → format OpenCV (k1,k2,p1,p2,k3)
        c = intr.coeffs
        dist = np.array([[c[0], c[1], c[2], c[3], c[4]]], dtype=np.float64)

        print("\n  Calibration RealSense (firmware) :")
        print(f"    fx={intr.fx:.2f}  fy={intr.fy:.2f}")
        print(f"    cx={intr.ppx:.2f}  cy={intr.ppy:.2f}")
        print(f"    distorsion : {dist.ravel()}")

        return {
            "K":          K,
            "dist":       dist,
            "rms":        0.0,       # calibration firmware = sans erreur de reprojection mesurée
            "img_width":  intr.width,
            "img_height": intr.height,
        }

    # -----------------------------------------------------------------------
    def get_3d_point(self, depth: np.ndarray, px: float, py: float,
                     calibration: dict) -> Optional[np.ndarray]:
        """
        Convertit un point 2D + sa profondeur en coordonnées 3D (mètres).

        Utilise la déprojection via les intrinsèques :
            X = (u - cx) * Z / fx
            Y = (v - cy) * Z / fy

        Paramètres
        ----------
        depth       : carte de profondeur (H, W) uint16 mm
        px, py      : coordonnées pixel du point
        calibration : dict issu de get_calibration()

        Retourne
        --------
        np.ndarray (3,) [X, Y, Z] en mètres, ou None si depth invalide
        """
        ix, iy = int(round(px)), int(round(py))
        h, w   = depth.shape

        if not (0 <= ix < w and 0 <= iy < h):
            return None

        z_mm = float(depth[iy, ix])
        if z_mm <= 0:
            # Profondeur invalide : prendre la médiane d'un patch 5×5
            patch = depth[max(0,iy-2):iy+3, max(0,ix-2):ix+3]
            valid = patch[patch > 0]
            if valid.size == 0:
                return None
            z_mm = float(np.median(valid))

        z = z_mm * 0.001   # mm → m
        K = calibration["K"]
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]

        X = (px - cx) * z / fx
        Y = (py - cy) * z / fy
        return np.array([X, Y, z])

    # -----------------------------------------------------------------------
    @staticmethod
    def is_available() -> bool:
        """Vérifie si pyrealsense2 est installé ET si une caméra est connectée."""
        try:
            import pyrealsense2 as rs  # type: ignore
            ctx     = rs.context()
            devices = ctx.query_devices()
            return len(devices) > 0
        except (ImportError, Exception):
            return False

    @staticmethod
    def list_devices():
        """Affiche les caméras RealSense connectées."""
        try:
            import pyrealsense2 as rs  # type: ignore
            ctx     = rs.context()
            devices = ctx.query_devices()
            if len(devices) == 0:
                print("  Aucune caméra RealSense détectée.")
                return
            for i, dev in enumerate(devices):
                name   = dev.get_info(rs.camera_info.name)
                serial = dev.get_info(rs.camera_info.serial_number)
                fw     = dev.get_info(rs.camera_info.firmware_version)
                print(f"  [{i}] {name}  SN={serial}  FW={fw}")
        except ImportError:
            print("  pyrealsense2 non installé.")


# ---------------------------------------------------------------------------
# --- Fallback : webcam classique (test sans matériel) ----------------------
# ---------------------------------------------------------------------------

class MockRealSense:
    """
    Émule une RealSense avec une webcam ordinaire.
    La carte de profondeur est constante (work_plane_z_mm).

    Utile pour tester le pipeline complet sans hardware RealSense.
    """

    def __init__(
        self,
        cam_index:       int   = 0,
        work_plane_z_mm: float = 800.0,
    ):
        self._cap      = cv2.VideoCapture(cam_index)
        self._z_mm     = work_plane_z_mm
        self.width     = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height    = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.fps       = int(self._cap.get(cv2.CAP_PROP_FPS)) or 30
        warnings.warn(
            "MockRealSense : profondeur constante "
            f"(Z={work_plane_z_mm:.0f} mm). Résultats approximatifs.",
            UserWarning, stacklevel=2
        )

    def __enter__(self) -> "MockRealSense":
        return self

    def __exit__(self, *_):
        self.stop()

    def start(self):
        pass

    def stop(self):
        self._cap.release()

    def read(self) -> tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        ret, color = self._cap.read()
        if not ret:
            return None, None
        # Profondeur uniforme au plan de travail
        depth = np.full((color.shape[0], color.shape[1]),
                        int(self._z_mm), dtype=np.uint16)
        return color, depth

    def get_depth_scale(self) -> float:
        return 0.001

    def get_calibration(self) -> dict:
        """Calibration approximative basée sur la résolution."""
        f  = max(self.width, self.height) * 0.85
        K  = np.array([[f, 0, self.width/2],
                       [0, f, self.height/2],
                       [0, 0, 1]], dtype=np.float64)
        warnings.warn(
            "Calibration MockRealSense approximative — "
            "utiliser camera_calibration.py pour une calibration réelle.",
            UserWarning, stacklevel=2
        )
        return {"K": K, "dist": np.zeros((1, 5)),
                "rms": -1.0, "img_width": self.width, "img_height": self.height}

    def get_3d_point(self, depth: np.ndarray, px: float, py: float,
                     calibration: dict) -> Optional[np.ndarray]:
        z  = self._z_mm * 0.001
        K  = calibration["K"]
        fx, fy = K[0,0], K[1,1]
        cx, cy = K[0,2], K[1,2]
        return np.array([(px - cx)*z/fx, (py - cy)*z/fy, z])


# ---------------------------------------------------------------------------
# --- Utilitaire : choisir automatiquement la meilleure caméra disponible ---
# ---------------------------------------------------------------------------

def auto_camera(
    cam_index:       int   = 0,
    work_plane_z_mm: float = 800.0,
    realsense_res:   tuple = (1280, 720),
    realsense_fps:   int   = 30,
):
    """
    Retourne une RealSenseCapture si disponible, sinon une MockRealSense.

    Usage :
        with auto_camera() as cam:
            calib = cam.get_calibration()
            color, depth = cam.read()
    """
    if RealSenseCapture.is_available():
        print("  RealSense D4xx détectée — utilisation du flux RGB-D.")
        return RealSenseCapture(*realsense_res, realsense_fps)
    else:
        print(f"  Aucune RealSense — fallback webcam (index {cam_index}).")
        return MockRealSense(cam_index, work_plane_z_mm)


# ---------------------------------------------------------------------------
# --- Test rapide ------------------------------------------------------------
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Test RealSense / MockRealSense")
    parser.add_argument("--cam",  type=int,   default=0)
    parser.add_argument("--z",    type=float, default=800.0, help="Profondeur mock (mm)")
    parser.add_argument("--list", action="store_true", help="Lister les caméras RealSense")
    args = parser.parse_args()

    if args.list:
        print("Caméras RealSense connectées :")
        RealSenseCapture.list_devices()
        raise SystemExit(0)

    print(f"RealSense disponible : {RealSenseCapture.is_available()}")

    with auto_camera(cam_index=args.cam, work_plane_z_mm=args.z) as cam:
        calib = cam.get_calibration()
        print(f"\nRésolution : {cam.width}×{cam.height} @ {cam.fps} fps")
        print(f"K =\n{calib['K']}")

        print("\nAppuyer sur 'q' pour quitter.")
        while True:
            color, depth = cam.read()
            if color is None:
                break

            # Afficher la profondeur en fausse couleur
            depth_vis = cv2.convertScaleAbs(depth, alpha=0.05)
            depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

            # Afficher la profondeur du centre
            cx, cy = cam.width // 2, cam.height // 2
            p3d = cam.get_3d_point(depth, cx, cy, calib)
            if p3d is not None:
                cv2.putText(color,
                            f"Centre : ({p3d[0]*1000:.0f}, {p3d[1]*1000:.0f}, {p3d[2]*1000:.0f}) mm",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            cv2.imshow("RealSense — RGB", color)
            cv2.imshow("RealSense — Depth", depth_color)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cv2.destroyAllWindows()
