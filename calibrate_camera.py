#!/usr/bin/env python3
"""
calibrate_camera.py — Calibration caméra → robot pour SCARA

Étape 1 : Calibration intrinsèque (damier)
  → sauvegarde python/phase2/calibration_data/camera_params.npz

Étape 2 : Calibration extrinsèque (correspondances pixel ↔ position robot)
  → sauvegarde python/phase2/calibration_data/cam_to_robot.npz

Usage :
  source env/bin/activate
  python calibrate_camera.py --cam 0
  python calibrate_camera.py --cam 0 --ip 192.168.0.10   # lecture PLC auto
  python calibrate_camera.py --cam 0 --skip-intrinsic     # si déjà calibré

Contrôles Étape 1 (damier) :
  [C] ou [ESPACE]  Capturer une pose
  [Q]              Calibrer et passer à l'étape 2
  [ESC]            Annuler

Contrôles Étape 2 (correspondances) :
  Clic gauche      Marquer le bout de l'effecteur
  [U]              Annuler le dernier point
  [Q]              Calculer la transformation et sauvegarder
  [ESC]            Annuler
"""

import argparse
import math
import os
import sys
import numpy as np
import cv2

try:
    import snap7
    from snap7.util import get_real
    _SNAP7 = True
except ImportError:
    _SNAP7 = False

# ── Paramètres robot (identiques à gui_robot.py) ────────────────────────────
L1, L2 = 150.0, 180.0  # mm
J1_SCALE = 25000.0 / math.radians(90)
J2_SCALE = 7500.0 / math.radians(90)

HERE = os.path.dirname(os.path.abspath(__file__))
CALIB_DIR = os.path.join(HERE, 'python', 'phase2', 'calibration_data')


def fk_xy_mm(j1_enc: float, j2_enc: float) -> tuple:
    """Cinématique directe : encodeurs J1, J2 → (X, Y) en mm."""
    t1 = j1_enc / J1_SCALE
    t2 = j2_enc / J2_SCALE
    x = L1 * math.cos(t1) + L2 * math.cos(t1 + t2)
    y = L1 * math.sin(t1) + L2 * math.sin(t1 + t2)
    return x, y


def read_plc_encoders(plc):
    """Lit les positions encodeur depuis la PLC. Retour : (J1, Z, J2, J4)."""
    data = plc.read_area(0x83, 0, 100, 16)
    return [get_real(data, i * 4) for i in range(4)]


# ═════════════════════════════════════════════════════════════════════════════
# ÉTAPE 1 : Calibration intrinsèque (damier)
# ═════════════════════════════════════════════════════════════════════════════

def step_intrinsic(cam_idx, board_cols, board_rows, square_mm, min_captures=12):
    board = (board_cols, board_rows)
    objp = np.zeros((board_cols * board_rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_cols, 0:board_rows].T.reshape(-1, 2) * square_mm

    obj_pts, img_pts = [], []
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

    cap = cv2.VideoCapture(cam_idx)
    if not cap.isOpened():
        print(f"ERREUR : caméra {cam_idx} introuvable")
        return None

    win = "Etape 1 — Calibration intrinseque (damier)"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)

    print("\n" + "=" * 60)
    print("  ÉTAPE 1 — CALIBRATION INTRINSÈQUE (DAMIER)")
    print("=" * 60)
    print(f"  Damier : {board_cols} x {board_rows} coins, cases de {square_mm} mm")
    print(f"  Minimum : {min_captures} captures variées (angles, distances)")
    print()
    print("  [C] ou [ESPACE] : capturer")
    print("  [Q] : calibrer et continuer")
    print("  [ESC] : annuler")
    print()

    flash_until = 0.0
    img_size = None

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        if img_size is None:
            img_size = (frame.shape[1], frame.shape[0])

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        enhanced = clahe.apply(gray)
        found, corners = cv2.findChessboardCorners(
            enhanced, board,
            cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)

        disp = frame.copy()
        if found:
            cv2.drawChessboardCorners(disp, board, corners, True)
            cv2.putText(disp, f"Damier {board_cols}x{board_rows} detecte — appuyez C",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(disp, f"Cherche damier {board_cols}x{board_rows}...",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        now = cv2.getTickCount() / cv2.getTickFrequency()
        if now < flash_until:
            overlay = disp.copy()
            cv2.rectangle(overlay, (0, 0), (disp.shape[1], disp.shape[0]),
                          (0, 255, 0), -1)
            cv2.addWeighted(overlay, 0.15, disp, 0.85, 0, disp)

        n = len(obj_pts)
        color = (0, 255, 0) if n >= min_captures else (0, 165, 255)
        cv2.putText(disp, f"Captures : {n}/{min_captures}",
                    (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        if n >= min_captures:
            cv2.putText(disp, "Pret ! Appuyez Q pour calibrer",
                        (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow(win, disp)
        key = cv2.waitKey(30) & 0xFF

        if key == 27:
            cap.release()
            cv2.destroyWindow(win)
            return None

        if key in (ord('c'), ord('C'), 32):
            if found:
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                obj_pts.append(objp.copy())
                img_pts.append(corners2)
                flash_until = now + 0.3
                print(f"  ✓ Capture {len(obj_pts)}")
            else:
                print("  ✗ Damier non détecté")

        if key in (ord('q'), ord('Q')):
            if n < 5:
                print(f"  ✗ Au moins 5 captures nécessaires ({n} actuellement)")
            else:
                break

    cap.release()
    cv2.destroyWindow(win)

    if len(obj_pts) < 5:
        print("ERREUR : pas assez de captures")
        return None

    print("\n  Calcul en cours...")
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_pts, img_pts, img_size, None, None)

    total_err = 0.0
    for i in range(len(obj_pts)):
        reproj, _ = cv2.projectPoints(obj_pts[i], rvecs[i], tvecs[i], K, dist)
        total_err += cv2.norm(img_pts[i], reproj, cv2.NORM_L2) / len(reproj)
    mean_err = total_err / len(obj_pts)

    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    print(f"\n  {'=' * 55}")
    print(f"  RÉSULTATS CALIBRATION INTRINSÈQUE")
    print(f"  {'=' * 55}")
    print(f"  Résolution  : {img_size[0]} x {img_size[1]} px")
    print(f"  Focale       : fx = {fx:.1f}  fy = {fy:.1f} px")
    print(f"  Centre       : cx = {cx:.1f}  cy = {cy:.1f} px")
    print(f"  Distorsion   : {np.array2string(dist.ravel()[:5], precision=4)}")
    print(f"  RMS          : {rms:.4f} px")
    quality = ("Excellente ✓" if rms < 0.5
               else "Bonne ✓" if rms < 1.0
               else "Médiocre ⚠ — refaites avec plus de poses variées")
    print(f"  Qualité      : {quality}")
    print(f"  {'=' * 55}")

    os.makedirs(CALIB_DIR, exist_ok=True)
    path = os.path.join(CALIB_DIR, 'camera_params.npz')
    np.savez(path, K=K, dist=dist, rms=np.float64(rms),
             img_width=np.int32(img_size[0]), img_height=np.int32(img_size[1]))
    print(f"\n  Sauvegardé : {path}")
    return K, dist, rms, img_size


# ═════════════════════════════════════════════════════════════════════════════
# ÉTAPE 2 : Calibration extrinsèque (correspondances pixel ↔ robot)
# ═════════════════════════════════════════════════════════════════════════════

def step_extrinsic(cam_idx, K, dist, plc=None, min_points=6):
    pixel_pts = []
    robot_pts = []
    pending_click = [None]

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            pending_click[0] = (x, y)

    cap = cv2.VideoCapture(cam_idx)
    if not cap.isOpened():
        print("ERREUR : caméra introuvable")
        return None

    win = "Etape 2 — Cliquez sur l'effecteur"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(win, on_mouse)

    print("\n" + "=" * 60)
    print("  ÉTAPE 2 — CALIBRATION EXTRINSÈQUE (pixel ↔ robot)")
    print("=" * 60)
    if plc:
        print("  Mode : PLC automatique ✓")
        print("  → Déplacez le robot avec le GUI")
        print("  → Cliquez sur le bout de l'effecteur dans l'image")
        print("  → Position lue automatiquement depuis la PLC")
    else:
        print("  Mode : saisie manuelle")
        print("  → Déplacez le robot avec le GUI")
        print("  → Cliquez sur le bout de l'effecteur dans l'image")
        print("  → Entrez X et Y (mm) affichés dans le GUI")
    print()
    print(f"  Couvrez l'espace de travail avec {min_points}+ points bien répartis")
    print("  [U] : annuler le dernier point")
    print("  [Q] : calculer la transformation")
    print("  [ESC] : annuler")
    print()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        disp = frame.copy()

        # Dessiner les points déjà collectés
        for i, (px, rb) in enumerate(zip(pixel_pts, robot_pts)):
            cv2.circle(disp, (int(px[0]), int(px[1])), 8, (0, 255, 0), 2)
            cv2.circle(disp, (int(px[0]), int(px[1])), 2, (0, 255, 0), -1)
            label = f"P{i + 1} ({rb[0]:.0f}, {rb[1]:.0f})"
            cv2.putText(disp, label, (int(px[0]) + 12, int(px[1]) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

        # Réticule du clic en attente
        if pending_click[0] is not None:
            cx, cy = pending_click[0]
            cv2.drawMarker(disp, (cx, cy), (0, 255, 255),
                           cv2.MARKER_CROSS, 30, 2)

        n = len(pixel_pts)
        color = (0, 255, 0) if n >= min_points else (0, 165, 255)
        cv2.putText(disp, f"Points : {n}/{min_points}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        if n >= min_points:
            cv2.putText(disp, "Pret ! Appuyez Q pour calibrer",
                        (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow(win, disp)
        key = cv2.waitKey(30) & 0xFF

        if key == 27:
            cap.release()
            cv2.destroyWindow(win)
            return None

        if key in (ord('u'), ord('U')):
            if pixel_pts:
                pixel_pts.pop()
                robot_pts.pop()
                pending_click[0] = None
                print(f"  ↩ Point retiré ({len(pixel_pts)} restants)")

        # Traiter le clic
        if pending_click[0] is not None:
            u, v = pending_click[0]
            pending_click[0] = None

            if plc:
                try:
                    plc_pos = read_plc_encoders(plc)
                    j1_enc, j2_enc = plc_pos[0], plc_pos[2]
                    x_mm, y_mm = fk_xy_mm(j1_enc, j2_enc)
                    pixel_pts.append((float(u), float(v)))
                    robot_pts.append((x_mm, y_mm))
                    print(f"  ✓ P{len(pixel_pts)}: pixel=({u}, {v})  "
                          f"→ robot=({x_mm:.1f}, {y_mm:.1f}) mm")
                except Exception as e:
                    print(f"  ✗ Erreur PLC : {e}")
            else:
                frozen = disp.copy()
                cv2.drawMarker(frozen, (u, v), (0, 0, 255),
                               cv2.MARKER_CROSS, 30, 2)
                cv2.putText(frozen, "Entrez X, Y dans le terminal...",
                            (10, frozen.shape[0] - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.imshow(win, frozen)
                cv2.waitKey(1)

                print(f"\n  Clic détecté : pixel ({u}, {v})")
                print("  Entrez les coordonnées robot affichées dans le GUI :")
                try:
                    x_str = input("    X (mm) = ")
                    y_str = input("    Y (mm) = ")
                    x_mm = float(x_str)
                    y_mm = float(y_str)
                    pixel_pts.append((float(u), float(v)))
                    robot_pts.append((x_mm, y_mm))
                    print(f"  ✓ P{len(pixel_pts)}: robot=({x_mm:.1f}, {y_mm:.1f}) mm\n")
                except (ValueError, EOFError):
                    print("  ✗ Valeur invalide, point ignoré\n")

        if key in (ord('q'), ord('Q')):
            if n < 4:
                print(f"  ✗ Minimum 4 points nécessaires ({n} actuellement)")
            else:
                break

    cap.release()
    cv2.destroyWindow(win)

    if len(pixel_pts) < 4:
        print("ERREUR : pas assez de points")
        return None

    # ── Calcul de la transformation ──────────────────────────────────────
    robot_2d = np.array(robot_pts, dtype=np.float64)   # (N, 2) en mm
    img_2d = np.array(pixel_pts, dtype=np.float64)     # (N, 2) en pixels

    print("\n  Calcul de la transformation...")

    # ── Homographie pixel → robot (robuste, indépendant de K) ────────────
    src = img_2d.reshape(-1, 1, 2).astype(np.float64)
    dst = robot_2d.reshape(-1, 1, 2).astype(np.float64)
    H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 5.0)
    if H is None:
        print("ERREUR : findHomography a échoué")
        return None

    inliers = mask.ravel().astype(bool)
    n_in = int(np.sum(inliers))
    n_out = len(inliers) - n_in
    if n_out:
        print(f"  RANSAC : {n_in} inliers, {n_out} outlier(s) rejeté(s)")

    # ── Validation homographie (test direct pixel → robot) ───────────────
    world_errors_mm = []
    print(f"\n  {'=' * 60}")
    print(f"  RÉSULTATS CALIBRATION EXTRINSÈQUE")
    print(f"  {'=' * 60}")
    print(f"  Points utilisés : {n_in}/{len(pixel_pts)}")

    for i, (px, rb) in enumerate(zip(pixel_pts, robot_pts)):
        p = H @ np.array([px[0], px[1], 1.0])
        p /= p[2]
        calc_x, calc_y = p[0], p[1]
        err_mm = math.hypot(calc_x - rb[0], calc_y - rb[1])
        world_errors_mm.append(err_mm)
        status = "  " if inliers[i] else "⚠ "
        print(f"  {status}P{i + 1}: vrai=({rb[0]:7.1f}, {rb[1]:7.1f})  "
              f"calc=({calc_x:7.1f}, {calc_y:7.1f}) mm  Δ={err_mm:.1f} mm")

    inlier_errors = [e for e, m in zip(world_errors_mm, inliers) if m]
    mean_mm = np.mean(inlier_errors) if inlier_errors else float('nan')
    max_mm = np.max(inlier_errors) if inlier_errors else float('nan')

    print(f"\n  Err. position (inliers) : {mean_mm:.1f} mm (moy)  {max_mm:.1f} mm (max)")

    # ── Décomposer H en T_cam_to_robot pour compatibilité pipeline ───────
    H_inv = np.linalg.inv(H)
    K_inv = np.linalg.inv(K)
    M = K_inv @ H_inv
    lam = (np.linalg.norm(M[:, 0]) + np.linalg.norm(M[:, 1])) / 2.0
    M /= lam
    r1, r2, t_vec = M[:, 0], M[:, 1], M[:, 2]
    r3 = np.cross(r1, r2)
    R_approx = np.column_stack([r1, r2, r3])
    # Projeter sur SO(3) via SVD
    U, _, Vt = np.linalg.svd(R_approx)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        R = -R
        t_vec = -t_vec

    T_robot_to_cam = np.eye(4)
    T_robot_to_cam[:3, :3] = R
    T_robot_to_cam[:3, 3] = t_vec / 1000.0  # mm → m
    T_cam_to_robot = np.linalg.inv(T_robot_to_cam)

    print(f"  Hauteur caméra    : {abs(t_vec[2]):.0f} mm")

    quality = ("Excellente ✓" if mean_mm < 3.0
               else "Bonne ✓" if mean_mm < 8.0
               else "Médiocre ⚠ — vérifiez les coordonnées et ajoutez des points")
    print(f"  Qualité           : {quality}")
    print(f"  {'=' * 60}")

    # ── Sauvegarde ───────────────────────────────────────────────────────
    os.makedirs(CALIB_DIR, exist_ok=True)
    path = os.path.join(CALIB_DIR, 'cam_to_robot.npz')
    np.savez(path, T=T_cam_to_robot, H=H)
    print(f"\n  Sauvegardé : {path}")
    print("  (T = matrice 4×4 cam→robot, H = homographie pixel→robot mm)")
    print("\n  Calibration terminée ✓")
    return T_cam_to_robot


# ═════════════════════════════════════════════════════════════════════════════
# Auto-estimation des intrinsèques (quand pas de damier)
# ═════════════════════════════════════════════════════════════════════════════

def estimate_intrinsics(cam_idx):
    """Estime K depuis la résolution caméra (FOV ~60°, distorsion nulle)."""
    cap = cv2.VideoCapture(cam_idx)
    ret, frame = cap.read()
    cap.release()
    if not ret:
        raise RuntimeError(f"Caméra {cam_idx} introuvable")
    h, w = frame.shape[:2]
    f = float(w)
    K = np.array([[f, 0, w / 2.0],
                  [0, f, h / 2.0],
                  [0, 0, 1.0]], dtype=np.float64)
    dist = np.zeros((1, 5), dtype=np.float64)
    print(f"\n  Intrinsèques auto-estimées (résolution {w}×{h}, FOV ~60°)")
    print(f"  fx={f:.0f}  fy={f:.0f}  cx={w/2:.0f}  cy={h/2:.0f}  dist=0")
    print("  → Pour de meilleurs résultats, refaites avec un damier plus tard")
    os.makedirs(CALIB_DIR, exist_ok=True)
    path = os.path.join(CALIB_DIR, 'camera_params.npz')
    np.savez(path, K=K, dist=dist, rms=np.float64(-1),
             img_width=np.int32(w), img_height=np.int32(h))
    print(f"  Sauvegardé : {path}")
    return K, dist, (w, h)


# ═════════════════════════════════════════════════════════════════════════════
# Main
# ═════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="Calibration caméra → robot SCARA (2 étapes)")
    parser.add_argument("--cam", type=int, default=0,
                        help="Index caméra (défaut 0)")
    parser.add_argument("--ip", type=str, default=None,
                        help="IP PLC pour lecture auto des positions robot")
    parser.add_argument("--cols", type=int, default=9,
                        help="Coins internes damier — colonnes (défaut 9)")
    parser.add_argument("--rows", type=int, default=6,
                        help="Coins internes damier — lignes (défaut 6)")
    parser.add_argument("--square", type=float, default=25.0,
                        help="Taille d'une case du damier en mm (défaut 25)")
    parser.add_argument("--skip-intrinsic", action="store_true",
                        help="Sauter l'étape 1 (charge le fichier existant ou auto-estime)")
    args = parser.parse_args()

    print("\n" + "=" * 60)
    print("  CALIBRATION CAMÉRA → ROBOT SCARA")
    print("=" * 60)

    # ── PLC (optionnel) ──────────────────────────────────────────────────
    plc = None
    if args.ip:
        if not _SNAP7:
            print("  ⚠ snap7 non installé → mode saisie manuelle")
        else:
            try:
                plc = snap7.client.Client()
                plc.connect(args.ip, 0, 1)
                print(f"  ✓ PLC connectée ({args.ip})")
            except Exception as e:
                print(f"  ⚠ Connexion PLC impossible ({e}) → mode saisie manuelle")
                plc = None

    # ── Étape 1 : intrinsèque ────────────────────────────────────────────
    K, dist = None, None
    params_path = os.path.join(CALIB_DIR, 'camera_params.npz')

    if args.skip_intrinsic:
        if os.path.exists(params_path):
            data = np.load(params_path)
            K, dist = data["K"], data["dist"]
            rms = float(data["rms"])
            print(f"\n  Intrinsèques chargées : {params_path}")
            print(f"  fx={K[0, 0]:.1f}  fy={K[1, 1]:.1f}  "
                  f"cx={K[0, 2]:.1f}  cy={K[1, 2]:.1f}  RMS={rms:.3f}")
        else:
            K, dist, _ = estimate_intrinsics(args.cam)
    else:
        result = step_intrinsic(args.cam, args.cols, args.rows, args.square)
        if result is None:
            print("\n  Pas de damier → auto-estimation des intrinsèques")
            K, dist, _ = estimate_intrinsics(args.cam)
        else:
            K, dist = result[0], result[1]

    # ── Étape 2 : extrinsèque ────────────────────────────────────────────
    T = step_extrinsic(args.cam, K, dist, plc=plc)

    if plc:
        plc.disconnect()

    if T is None:
        print("\nCalibration annulée.")
        sys.exit(1)


if __name__ == "__main__":
    main()
