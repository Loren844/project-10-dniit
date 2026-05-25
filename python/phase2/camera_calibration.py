"""
camera_calibration.py
Calibration intrinsèque de la caméra par damier (chessboard).

Méthode : OpenCV findChessboardCorners + calibrateCamera
Sortie   : camera_params.npz  →  K (3×3), dist (1×5), rms (px)

Usage :
    # Calibration depuis un dossier d'images
    python camera_calibration.py --images calibration_data/ --rows 6 --cols 9 --size 25

    # Calibration en direct (webcam)
    python camera_calibration.py --live --rows 6 --cols 9 --size 25

Arguments :
    --images  DIR   Dossier contenant les images de calibration (*.jpg, *.png)
    --live          Capture depuis la webcam (appuyer sur 'c' pour capturer, 'q' pour quitter)
    --rows    INT   Nombre de coins internes (lignes du damier, défaut 6)
    --cols    INT   Nombre de coins internes (colonnes du damier, défaut 9)
    --size    FLOAT Taille d'une case du damier en mm (défaut 25)
    --output  FILE  Fichier de sortie (défaut calibration_data/camera_params.npz)
"""

import argparse
import glob
import os
import sys

import cv2
import numpy as np


# ---------------------------------------------------------------------------
def calibrate_from_images(image_paths: list[str], board_size: tuple, square_mm: float):
    """
    Calibration depuis une liste de fichiers image.

    Paramètres
    ----------
    image_paths : list[str]
        Chemins vers les images de calibration.
    board_size : (int, int)
        (colonnes, lignes) de coins internes du damier.
    square_mm : float
        Taille d'une case en mm.

    Retourne
    --------
    K     : np.ndarray (3, 3)  Matrice intrinsèque
    dist  : np.ndarray (1, 5)  Coefficients de distorsion (k1, k2, p1, p2, k3)
    rms   : float              Erreur de reprojection RMS en pixels
    img_size : (int, int)      (largeur, hauteur) des images
    """
    cols, rows = board_size

    # Points 3D dans le repère damier (z = 0)
    obj_pts_template = np.zeros((rows * cols, 3), dtype=np.float32)
    obj_pts_template[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    obj_pts_template *= square_mm

    obj_points = []   # Points 3D (monde)
    img_points = []   # Points 2D (image)
    img_size   = None

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-4)

    n_ok = 0
    for path in image_paths:
        img  = cv2.imread(path)
        if img is None:
            print(f"  [WARN] Impossible de lire : {path}")
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if img_size is None:
            img_size = (gray.shape[1], gray.shape[0])

        found, corners = cv2.findChessboardCorners(gray, (cols, rows), None)
        if not found:
            print(f"  [SKIP] Damier non détecté : {os.path.basename(path)}")
            continue

        corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        obj_points.append(obj_pts_template)
        img_points.append(corners_refined)
        n_ok += 1
        print(f"  [OK]   {os.path.basename(path)}")

    if n_ok < 5:
        raise RuntimeError(
            f"Calibration insuffisante : seulement {n_ok} image(s) valide(s). "
            "Au moins 5 sont nécessaires."
        )

    print(f"\n  {n_ok}/{len(image_paths)} images utilisées pour la calibration.")

    rms, K, dist, _, _ = cv2.calibrateCamera(
        obj_points, img_points, img_size, None, None
    )
    return K, dist, rms, img_size


# ---------------------------------------------------------------------------
def calibrate_live(board_size: tuple, square_mm: float, cam_index: int = 0,
                   n_captures: int = 20):
    """
    Calibration interactive depuis la webcam.
    Appuyer sur 'c' pour capturer, 'q' pour terminer et calibrer.
    """
    cols, rows = board_size
    obj_pts_template = np.zeros((rows * cols, 3), dtype=np.float32)
    obj_pts_template[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    obj_pts_template *= square_mm

    obj_points = []
    img_points = []
    img_size   = None
    criteria   = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-4)

    cap = cv2.VideoCapture(cam_index)
    if not cap.isOpened():
        raise RuntimeError(f"Impossible d'ouvrir la caméra (index {cam_index}).")

    print("  Appuyer sur 'c' pour capturer, 'q' pour calibrer et quitter.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, (cols, rows), None)

        display = frame.copy()
        if found:
            cv2.drawChessboardCorners(display, (cols, rows), corners, found)
            cv2.putText(display, "Damier détecté — appuyer sur 'c'",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            cv2.putText(display, "Damier non détecté",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        cv2.putText(display, f"Captures : {len(img_points)}/{n_captures}",
                    (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.imshow("Calibration — appuyer c/q", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or len(img_points) >= n_captures:
            break
        if key == ord('c') and found:
            if img_size is None:
                img_size = (gray.shape[1], gray.shape[0])
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            obj_points.append(obj_pts_template)
            img_points.append(corners_refined)
            print(f"  Capture {len(img_points)}/{n_captures}")

    cap.release()
    cv2.destroyAllWindows()

    if len(img_points) < 5:
        raise RuntimeError(
            f"Calibration insuffisante : {len(img_points)} capture(s). Au moins 5 sont nécessaires."
        )

    rms, K, dist, _, _ = cv2.calibrateCamera(
        obj_points, img_points, img_size, None, None
    )
    return K, dist, rms, img_size


# ---------------------------------------------------------------------------
def save_calibration(K: np.ndarray, dist: np.ndarray, rms: float,
                     img_size: tuple, output_path: str):
    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    np.savez(output_path, K=K, dist=dist, rms=rms,
             img_width=img_size[0], img_height=img_size[1])
    print(f"\n  Calibration sauvegardée : {output_path}")


def load_calibration(path: str) -> dict:
    """Charge camera_params.npz et retourne un dict {K, dist, rms, img_width, img_height}."""
    data = np.load(path)
    return {
        "K":          data["K"],
        "dist":       data["dist"],
        "rms":        float(data["rms"]),
        "img_width":  int(data["img_width"]),
        "img_height": int(data["img_height"]),
    }


def print_calibration_report(K: np.ndarray, dist: np.ndarray, rms: float,
                              img_size: tuple):
    w, h = img_size
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    print("\n" + "=" * 55)
    print("  RÉSULTATS DE CALIBRATION")
    print("=" * 55)
    print(f"  Résolution image       : {w} × {h} px")
    print(f"  Focale  fx = {fx:.2f} px   fy = {fy:.2f} px")
    print(f"  Centre  cx = {cx:.2f} px   cy = {cy:.2f} px")
    print(f"  Distorsion  : {dist.ravel()}")
    print(f"  Erreur RMS  : {rms:.4f} px", end="  ")
    if rms < 0.5:
        print("✓ (excellente)")
    elif rms < 1.0:
        print("✓ (acceptable)")
    else:
        print("⚠ (élevée — recalibrer)")
    print("=" * 55)


# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="Calibration intrinsèque par damier")
    src = parser.add_mutually_exclusive_group(required=True)
    src.add_argument("--images", metavar="DIR",
                     help="Dossier d'images de calibration")
    src.add_argument("--live",   action="store_true",
                     help="Calibration depuis la webcam")
    parser.add_argument("--rows",   type=int,   default=6,   help="Coins internes (lignes)")
    parser.add_argument("--cols",   type=int,   default=9,   help="Coins internes (colonnes)")
    parser.add_argument("--size",   type=float, default=25.0, help="Taille case (mm)")
    parser.add_argument("--output", default="calibration_data/camera_params.npz")
    parser.add_argument("--cam",    type=int,   default=0,   help="Index caméra (live)")
    parser.add_argument("--n",      type=int,   default=20,  help="Nb captures (live)")
    args = parser.parse_args()

    board = (args.cols, args.rows)

    print("=" * 55)
    print("  CALIBRATION CAMÉRA — DAMIER")
    print("=" * 55)
    print(f"  Damier   : {args.cols} × {args.rows} coins internes")
    print(f"  Case     : {args.size} mm\n")

    if args.images:
        patterns = ["*.jpg", "*.jpeg", "*.png", "*.bmp"]
        paths = []
        for pat in patterns:
            paths.extend(sorted(glob.glob(os.path.join(args.images, pat))))
        if not paths:
            print(f"Aucune image trouvée dans {args.images}", file=sys.stderr)
            sys.exit(1)
        print(f"  {len(paths)} image(s) trouvée(s)\n")
        K, dist, rms, img_size = calibrate_from_images(paths, board, args.size)
    else:
        K, dist, rms, img_size = calibrate_live(board, args.size, args.cam, args.n)

    print_calibration_report(K, dist, rms, img_size)
    save_calibration(K, dist, rms, img_size, args.output)


if __name__ == "__main__":
    main()
