"""
hand_eye_collect.py — Collecte interactive pour la calibration caméra→robot.

Protocole :
  1. Fixe un marqueur ArUco sur l'effecteur du robot.
  2. Lance ce script.
  3. À chaque pose robot, entre manuellement les angles PLC puis appuie sur ESPACE
     pour capturer la pose ArUco vue par la caméra.
  4. Répète pour 8–15 positions très différentes (angle, distance, inclinaison).
  5. Une fois terminé, appuie sur 'c' pour calculer et enregistrer cam_to_robot.npz.

Usage :
    cd python/phase2
    python hand_eye_collect.py --cam 0 --marker-id 0 --marker-size 0.06

Arguments :
    --cam          INT    Index caméra (défaut 0)
    --marker-id    INT    ID du marqueur ArUco sur l'effecteur (défaut 0)
    --marker-size  FLOAT  Taille du marqueur en mètres (défaut 0.06)
    --calib        FILE   Chemin cam_to_robot.npz (défaut calibration_data/cam_to_robot.npz)
    --output       FILE   Fichier de sortie (défaut calibration_data/cam_to_robot.npz)
"""

from __future__ import annotations

import argparse
import os
import sys

import cv2
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

from camera_calibration import load_calibration
from robot_transform import hand_eye_calibrate, save_transform, print_transform_report


# ── Cinématique directe SCARA (même paramètres que vs_controller.py) ────────

A2   = 0.300   # m — bras 1
A3   = 0.160   # m — bras 2
D3   = 0.150   # m — offset Z lien 3
D4   = 0.059   # m — offset effecteur


def fk_scara(theta1_deg: float, theta3_deg: float, d2_mm: float,
             theta4_deg: float = 0.0) -> np.ndarray:
    """
    Retourne la matrice homogène T_gripper_in_base (4×4).
    Entrées : angles en degrés, Z en mm.
    """
    t1 = np.radians(theta1_deg)
    t3 = np.radians(theta3_deg)
    d2 = d2_mm / 1000.0

    px = A2 * np.cos(t1) + A3 * np.cos(t1 + t3)
    py = A2 * np.sin(t1) + A3 * np.sin(t1 + t3)
    pz = d2 - D3 - D4

    phi = t1 + t3 + np.radians(theta4_deg)
    c, s = np.cos(phi), np.sin(phi)

    T = np.eye(4)
    T[:3, :3] = np.array([[c, -s, 0],
                           [s,  c, 0],
                           [0,  0, 1]])
    T[:3, 3] = [px, py, pz]
    return T


# ── Détection ArUco ─────────────────────────────────────────────────────────

def detect_and_pose(frame: np.ndarray, cam_params: dict,
                    marker_size_m: float, target_id: int):
    """
    Détecte le marqueur ArUco et retourne (R_target2cam, t_target2cam) ou None.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
    aruco_params = cv2.aruco.DetectorParameters()
    detector     = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        return None

    for i, mid in enumerate(ids.ravel()):
        if mid != target_id:
            continue
        half = marker_size_m / 2.0
        obj_pts = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0],
        ], dtype=np.float64)

        img_pts = corners[i].reshape(4, 2).astype(np.float64)
        K    = cam_params["K"]
        dist = cam_params["dist"]
        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, dist,
                                      flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return None
        R, _ = cv2.Rodrigues(rvec)
        return R, tvec.ravel()

    return None


# ── Interface ────────────────────────────────────────────────────────────────

def ask_robot_pose() -> tuple[float, float, float, float] | None:
    """Demande à l'opérateur les coordonnées cartésiennes XYZ + angle poignet lus sur la PLC."""
    print("\n╔══════════════════════════════════════════════╗")
    print("║  Lire sur la PLC les valeurs actuelles :     ║")
    print("╚══════════════════════════════════════════════╝")
    try:
        x   = float(input("  X (mm)              : "))
        y   = float(input("  Y (mm)              : "))
        z   = float(input("  Z (mm)              : "))
        rz  = float(input("  θ poignet (degrés)  : "))
        return x, y, z, rz
    except (ValueError, KeyboardInterrupt):
        return None


def draw_overlay(frame: np.ndarray, pose_result, sample_count: int):
    h, w = frame.shape[:2]
    color = (0, 255, 0) if pose_result else (0, 60, 200)
    label = (f"✓ Marqueur détecté — {sample_count} captures"
             if pose_result else "Marqueur non visible")
    cv2.rectangle(frame, (0, h - 40), (w, h), (20, 20, 20), -1)
    cv2.putText(frame, label, (10, h - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
    cv2.putText(frame, f"[ESPACE] Capturer  [c] Calibrer  [q] Quitter",
                (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
    cv2.putText(frame, f"Captures: {sample_count}", (w - 180, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 200, 255), 2)


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cam",         type=int,   default=0)
    ap.add_argument("--marker-id",   type=int,   default=0)
    ap.add_argument("--marker-size", type=float, default=0.06)
    ap.add_argument("--calib",   default=os.path.join(_HERE, "calibration_data", "cam_to_robot.npz"))
    ap.add_argument("--output",  default=os.path.join(_HERE, "calibration_data", "cam_to_robot.npz"))
    ap.add_argument("--samples", default=os.path.join(_HERE, "calibration_data", "handeye_samples.npz"))
    args = ap.parse_args()

    if not os.path.exists(args.calib):
        print(f"[ERREUR] cam_to_robot.npz introuvable: {args.calib}")
        print("→ Fais d'abord la calibration intrinsèque depuis l'interface GUI.")
        sys.exit(1)

    cam_params = load_calibration(args.calib)
    print(f"[OK] Calibration caméra chargée (RMS={cam_params.get('rms', -1):.3f})")

    cap = cv2.VideoCapture(args.cam)
    if not cap.isOpened():
        print(f"[ERREUR] Caméra {args.cam} introuvable.")
        sys.exit(1)

    R_gripper2base = []
    t_gripper2base = []
    R_target2cam   = []
    t_target2cam   = []

    # Recharge les captures précédentes si elles existent
    if os.path.exists(args.samples):
        data = np.load(args.samples, allow_pickle=True)
        R_gripper2base = list(data["R_g2b"])
        t_gripper2base = list(data["t_g2b"])
        R_target2cam   = list(data["R_t2c"])
        t_target2cam   = list(data["t_t2c"])
        print(f"[OK] {len(R_gripper2base)} captures rechargées depuis {args.samples}")

    MIN_SAMPLES = 8
    print(f"\n[INFO] Cible: marqueur ArUco ID={args.marker_id}, taille={args.marker_size*1000:.0f} mm")
    print(f"[INFO] Minimum requis: {MIN_SAMPLES} captures.")
    print("\n→ Bouge le robot vers une première position, puis appuie sur ESPACE.\n")

    last_pose = None

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        last_pose = detect_and_pose(frame, cam_params,
                                    args.marker_size, args.marker_id)

        if last_pose:
            R_t2c, t_t2c = last_pose
            cv2.aruco.drawDetectedMarkers(frame, *([],))  # skip, déjà dans detect
            # dessine axes
            K, dist = cam_params["K"], cam_params["dist"]
            rvec, _ = cv2.Rodrigues(R_t2c)
            cv2.drawFrameAxes(frame, K, dist, rvec, t_t2c, args.marker_size * 0.5)

        draw_overlay(frame, last_pose, len(R_gripper2base))
        cv2.imshow("Hand-Eye Calibration", frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break

        elif key == ord(' '):
            if last_pose is None:
                print("[WARN] Marqueur non visible — approche le robot ou le marqueur.")
                continue

            robot_pose = ask_robot_pose()
            if robot_pose is None:
                continue

            x_mm, y_mm, z_mm, rz_deg = robot_pose
            angle = np.radians(rz_deg)
            c, s = np.cos(angle), np.sin(angle)
            R_g2b = np.array([[c, -s, 0],
                               [s,  c, 0],
                               [0,  0, 1]])
            t_g2b = np.array([x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0])

            R_t2c, t_t2c = last_pose
            R_gripper2base.append(R_g2b)
            t_gripper2base.append(t_g2b.reshape(3, 1))
            R_target2cam.append(R_t2c)
            t_target2cam.append(t_t2c.reshape(3, 1))

            n = len(R_gripper2base)
            # Sauvegarde automatique après chaque capture
            os.makedirs(os.path.dirname(args.samples), exist_ok=True)
            np.savez(args.samples,
                     R_g2b=np.array(R_gripper2base), t_g2b=np.array(t_gripper2base),
                     R_t2c=np.array(R_target2cam),   t_t2c=np.array(t_target2cam))
            print(f"  [✓] Capture {n} enregistrée (X={x_mm:.1f}mm Y={y_mm:.1f}mm Z={z_mm:.1f}mm θ={rz_deg:.1f}°)")
            if n >= MIN_SAMPLES:
                print(f"  → {n} captures disponibles. Appuie sur 'c' pour calibrer.")

        elif key == ord('c'):
            n = len(R_gripper2base)
            if n < MIN_SAMPLES:
                print(f"[WARN] Seulement {n}/{MIN_SAMPLES} captures. Continue à collecter.")
                continue

            print(f"\n[INFO] Calcul de la calibration hand-eye sur {n} poses...")
            try:
                tf = hand_eye_calibrate(
                    R_gripper2base, t_gripper2base,
                    R_target2cam,   t_target2cam,
                )
                os.makedirs(os.path.dirname(args.output), exist_ok=True)
                save_transform(tf, args.output)
                print_transform_report(tf)
                print(f"\n[✓] Fichier enregistré : {args.output}")
                print("[✓] Relance l'application GUI pour utiliser la nouvelle calibration.")
            except Exception as e:
                import traceback
                print(f"[ERREUR] Calibration échouée: {e}")
                traceback.print_exc()
                continue

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
