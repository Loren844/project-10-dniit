"""
main_phase2.py
Pipeline complet Phase 2 — Reconnaissance d'objets et estimation de pose.

Étapes :
  1. Vérification des dépendances et du fichier de calibration
  2. Chargement de la calibration caméra + transformation caméra→robot
  3. Détection (ArUco ou YOLO) sur image, vidéo ou flux live
  4. Estimation de la pose 6D dans le repère caméra
  5. Transformation vers le repère robot
  6. Affichage console + visualisation

Usage :
    # Test sur image avec ArUco (mode le plus simple)
    python main_phase2.py --image test_images/scene.jpg --method aruco

    # Flux webcam avec ArUco
    python main_phase2.py --live --method aruco

    # Image avec YOLO (nécessite ultralytics + modèle entraîné)
    python main_phase2.py --image test_images/scene.jpg --method yolo --model yolov8n.pt

    # Générer une image de test avec des marqueurs ArUco
    python main_phase2.py --generate-test

Options principales :
    --calib   FILE   Calibration caméra  (défaut : calibration_data/camera_params.npz)
    --tf      FILE   Transformation cam→robot (défaut : calibration_data/cam_to_robot.npz)
    --size    FLOAT  Taille marqueur ArUco en m (défaut : 0.05)
    --z       FLOAT  Hauteur plan de travail en m (mode YOLO flat, défaut : 0.80)
"""

from __future__ import annotations

import argparse
import os
import sys

import cv2
import numpy as np


# ---------------------------------------------------------------------------
# --- Vérification des dépendances ------------------------------------------
# ---------------------------------------------------------------------------

def check_dependencies():
    missing = []
    try:
        import cv2  # noqa: F401
    except ImportError:
        missing.append("opencv-python  (pip install opencv-python)")
    try:
        import cv2.aruco  # noqa: F401
    except (ImportError, AttributeError):
        missing.append("opencv-contrib-python  (pip install opencv-contrib-python)")
    try:
        import numpy  # noqa: F401
    except ImportError:
        missing.append("numpy  (pip install numpy)")

    if missing:
        print("Dépendances manquantes :")
        for m in missing:
            print(f"  - {m}")
        sys.exit(1)


# ---------------------------------------------------------------------------
# --- Génération d'images de test -------------------------------------------
# ---------------------------------------------------------------------------

def generate_test_image(output_dir: str = "test_images", dict_name: str = "5x5_50"):
    """Génère une image de test avec 3 marqueurs ArUco sur fond blanc."""
    os.makedirs(output_dir, exist_ok=True)

    aruco_dict_ids = {
        "5x5_50": cv2.aruco.DICT_5X5_50,
        "4x4_50": cv2.aruco.DICT_4X4_50,
    }
    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_ids.get(dict_name, cv2.aruco.DICT_5X5_50))

    img = np.ones((600, 900, 3), dtype=np.uint8) * 240
    positions = [(50, 50), (350, 200), (600, 50)]
    size_px = 150

    for i, (x, y) in enumerate(positions):
        marker_img = cv2.aruco.generateImageMarker(aruco_dict, i, size_px)
        marker_bgr = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2BGR)
        img[y:y+size_px, x:x+size_px] = marker_bgr
        cv2.putText(img, f"ID {i}", (x, y + size_px + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (50, 50, 200), 2)

    # Simuler un objet sur tapis roulant
    cv2.rectangle(img, (200, 350), (700, 550), (180, 220, 180), -1)
    cv2.rectangle(img, (200, 350), (700, 550), (80, 80, 80), 2)
    cv2.putText(img, "Tapis roulant (objet simulé)", (210, 460),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (50, 50, 50), 2)

    path = os.path.join(output_dir, "test_aruco_scene.jpg")
    cv2.imwrite(path, img)
    print(f"Image de test générée : {path}")
    return path


# ---------------------------------------------------------------------------
# --- Pipeline principal ----------------------------------------------------
# ---------------------------------------------------------------------------

def make_dummy_calibration() -> dict:
    """
    Calibration factice pour les tests sans fichier réel.
    Paramètres typiques d'une webcam 1080p.
    """
    import warnings
    warnings.warn(
        "Aucun fichier de calibration trouvé — utilisation de paramètres factices.\n"
        "Les mesures de distance seront approximatives.\n"
        "Lancer camera_calibration.py pour une calibration réelle.",
        UserWarning, stacklevel=2
    )
    W, H = 1280, 720
    f = 900.0   # focale approximative en pixels
    K = np.array([[f, 0, W/2],
                  [0, f, H/2],
                  [0, 0, 1  ]], dtype=np.float64)
    return {"K": K, "dist": np.zeros((1, 5)), "rms": -1.0,
            "img_width": W, "img_height": H}


def make_default_transform() -> "RobotTransform":
    """
    Transformation géométrique par défaut :
    caméra centrée à 800 mm au-dessus de la base robot, orientée vers le bas.
    À remplacer par la calibration main-œil réelle.
    """
    from robot_transform import make_transform_from_geometry
    import warnings
    warnings.warn(
        "Transformation caméra→robot par défaut (géométrique).\n"
        "Lancer la procédure hand_eye_calibrate() pour une transformation précise.",
        UserWarning, stacklevel=2
    )
    return make_transform_from_geometry(
        translation_m=[0.0, 0.0, 0.8],
        rotation_deg=[0.0, 0.0, -90.0],
    )


def run_pipeline_on_frame(
    frame: np.ndarray,
    cam: dict,
    tf,
    method: str,
    yolo_model,
    conf_threshold: float,
    marker_size_m: float,
    work_plane_z_m: float,
    depth_frame: np.ndarray = None,
    mot: "MultiObjectTracker" = None,
) -> tuple[list, list, list, np.ndarray]:
    """
    Exécute détection + estimation de pose + Kalman sur un frame.

    Retourne (detections, poses_robot, tracked_objects, frame_annotated)
    """
    from detect_objects   import detect_aruco, detect_yolo, draw_detections
    from pose_estimation  import (estimate_pose_aruco, estimate_pose_yolo_flat,
                                   estimate_pose_yolo_rgbd, draw_pose_axes)
    from kalman_tracker   import MultiObjectTracker

    # --- Détection ---
    if method == "aruco":
        detections = detect_aruco(frame)
    else:
        detections = detect_yolo(frame, model_path=yolo_model,
                                 conf_threshold=conf_threshold)

    # --- Estimation de pose ---
    if method == "aruco":
        poses_cam = estimate_pose_aruco(detections, cam, marker_size_m=marker_size_m)
    elif depth_frame is not None:
        poses_cam = estimate_pose_yolo_rgbd(detections, cam, depth_frame)
    else:
        poses_cam = estimate_pose_yolo_flat(detections, cam, work_plane_z_m)

    # --- Transformation repère robot ---
    poses_robot = [tf.transform(p) for p in poses_cam]

    # --- Filtre de Kalman (suivi + prédiction) ---
    tracked_objects = []
    if mot is not None:
        measurements = [(p.label, p.position_m) for p in poses_robot]
        tracked_objects = mot.update(measurements)

    # --- Annotation visuelle ---
    annotated = draw_detections(frame, detections)
    annotated = draw_pose_axes(annotated, poses_cam, cam,
                                axis_length_m=marker_size_m * 0.8)

    # Dessiner les prédictions Kalman (croix orange)
    K_mat = cam["K"].astype(np.float64)
    dist  = cam["dist"].astype(np.float64)
    for obj in tracked_objects:
        # Projeter la position prédite (repère robot → repère caméra via tf inverse)
        p_pred_cam = tf.inverse().transform_point(obj.predicted_m)
        if p_pred_cam[2] > 0:
            import cv2 as _cv2
            pts, _ = _cv2.projectPoints(
                p_pred_cam.reshape(1, 3), np.zeros(3), np.zeros(3), K_mat, dist
            )
            px, py = int(pts[0, 0, 0]), int(pts[0, 0, 1])
            _cv2.drawMarker(annotated, (px, py), (0, 128, 255),
                            _cv2.MARKER_DIAMOND, 18, 3)
            _cv2.putText(annotated, f"pred T{obj.track_id}",
                         (px + 10, py - 5), _cv2.FONT_HERSHEY_SIMPLEX,
                         0.5, (0, 128, 255), 1)

    return detections, poses_robot, tracked_objects, annotated


def print_poses(poses_robot: list, tracked: list = None, step: int = None):
    prefix = f"[Frame {step}] " if step is not None else ""
    for p in poses_robot:
        pos_mm = p.position_m * 1000
        print(f"  {prefix}[{p.label}]  "
              f"X={pos_mm[0]:+7.1f}mm  Y={pos_mm[1]:+7.1f}mm  Z={pos_mm[2]:+7.1f}mm")
    if tracked:
        for obj in tracked:
            print(f"    ↳ {obj}")


# ---------------------------------------------------------------------------
# --- CLI -------------------------------------------------------------------
# ---------------------------------------------------------------------------

def parse_args():
    parser = argparse.ArgumentParser(
        description="Phase 2 — Reconnaissance d'objets et estimation de pose"
    )
    src = parser.add_mutually_exclusive_group()
    src.add_argument("--image",         metavar="FILE",  help="Image de test")
    src.add_argument("--video",         metavar="FILE",  help="Fichier vidéo")
    src.add_argument("--live",          action="store_true", help="Webcam live")
    src.add_argument("--generate-test", action="store_true",
                     help="Générer une image de test et quitter")
    src.add_argument("--list-cams",   action="store_true",
                     help="Lister les caméras disponibles et quitter")
    src.add_argument("--realsense",   action="store_true",
                     help="Forcer l'utilisation de la RealSense D4xx (RGB-D)")

    parser.add_argument("--method",   choices=["aruco", "yolo"], default="aruco")
    parser.add_argument("--calib",    default="calibration_data/camera_params.npz")
    parser.add_argument("--tf",       default="calibration_data/cam_to_robot.npz")
    parser.add_argument("--size",     type=float, default=0.05,  help="Taille marqueur (m)")
    parser.add_argument("--z",        type=float, default=0.80,  help="Hauteur plan travail (m)")
    parser.add_argument("--model",    default="yolov8n.pt",      help="Modèle YOLO")
    parser.add_argument("--conf",     type=float, default=0.5)
    parser.add_argument("--cam",      type=int,   default=0)
    parser.add_argument("--dict",     default="5x5_50",          help="Dictionnaire ArUco")
    return parser.parse_args()


def main():
    check_dependencies()
    args = parse_args()

    # --generate-test : créer l'image et quitter
    if args.generate_test:
        path = generate_test_image()
        print(f"\nTester avec : python main_phase2.py --image {path} --method aruco")
        return

    # --list-cams : identifier l'index de chaque caméra
    if args.list_cams:
        print("Recherche des caméras disponibles...")
        found = []
        for i in range(6):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                print(f"  [index {i}]  {w}×{h}")
                found.append(i)
                cap.release()
        if not found:
            print("  Aucune caméra détectée.")
        else:
            print(f"\nUtiliser --cam INDEX  (ex: --cam {found[0]})")
            print("Sur macOS : index 0 = Continuity Camera (téléphone), index 1 = FaceTime intégrée")
        print("\nCaméras Intel RealSense :")
        from realsense_capture import RealSenseCapture
        RealSenseCapture.list_devices()
        return

    # Charger ou créer calibration + transformation
    from camera_calibration import load_calibration
    from robot_transform    import load_transform

    if os.path.exists(args.calib):
        cam = load_calibration(args.calib)
        print(f"Calibration chargée : {args.calib}  (RMS={cam['rms']:.3f} px)")
    else:
        print(f"[WARN] Calibration non trouvée ({args.calib}) — paramètres par défaut.")
        cam = make_dummy_calibration()

    if os.path.exists(args.tf):
        tf = load_transform(args.tf)
        print(f"Transformation chargée : {args.tf}")
    else:
        print(f"[WARN] Transformation non trouvée ({args.tf}) — géométrie par défaut.")
        tf = make_default_transform()

    yolo_model = args.model if args.method == "yolo" else None

    print(f"\nMéthode de détection : {args.method.upper()}")
    print("-" * 50)

    from kalman_tracker import MultiObjectTracker

    # --- Image statique (pas de Kalman — un seul frame) ---
    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            print(f"Impossible de lire : {args.image}", file=sys.stderr)
            sys.exit(1)
        detections, poses_robot, _, annotated = run_pipeline_on_frame(
            frame, cam, tf, args.method, yolo_model,
            args.conf, args.size, args.z
        )
        print(f"{len(poses_robot)} objet(s) détecté(s) :")
        print_poses(poses_robot)
        cv2.imshow("Phase 2 — Détection + Pose", annotated)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    # --- Vidéo ou flux live (Kalman actif) ---
    elif args.video or args.live or args.realsense:
        from realsense_capture import auto_camera, RealSenseCapture

        if args.realsense or (not args.video and RealSenseCapture.is_available()):
            # RealSense disponible → RGB-D complet, calibration depuis firmware
            camera_ctx = auto_camera(cam_index=args.cam,
                                     work_plane_z_mm=args.z * 1000)
            # Remplacer la calibration par celle du firmware RealSense
            print("Calibration : firmware RealSense (intrinsèques automatiques)")
            use_realsense = True
        else:
            camera_ctx = None
            use_realsense = False

        source = args.video if args.video else args.cam
        if not use_realsense:
            cap = cv2.VideoCapture(source)
            if not cap.isOpened():
                print(f"Impossible d'ouvrir la source vidéo : {source}", file=sys.stderr)
                sys.exit(1)
            fps = cap.get(cv2.CAP_PROP_FPS)
        else:
            fps = 30.0

        dt  = 1.0 / fps if fps > 0 else 0.033
        mot = MultiObjectTracker(dt=dt, sigma_accel=1.0, sigma_pos=0.005,
                                 max_missed=5, latency_s=0.150)

        print(f"Filtre de Kalman activé  (dt={dt*1000:.1f}ms, latence=150ms)")
        print("Appuyer sur 'q' pour quitter, 's' pour sauvegarder un frame.")
        frame_idx = 0

        def read_frame():
            """Lit (frame BGR, depth uint16|None) selon la source."""
            if use_realsense:
                return camera_ctx.read()
            else:
                ret, f = cap.read()
                return (f, None) if ret else (None, None)

        ctx = camera_ctx if use_realsense else cap

        # Utiliser la calibration RealSense si disponible
        if use_realsense:
            with camera_ctx as cam_dev:
                cam = cam_dev.get_calibration()
                depth_scale = cam_dev.get_depth_scale()
                while True:
                    frame, depth = cam_dev.read()
                    if frame is None:
                        break
                    detections, poses_robot, tracked, annotated = run_pipeline_on_frame(
                        frame, cam, tf, args.method, yolo_model,
                        args.conf, args.size, args.z,
                        depth_frame=depth, mot=mot
                    )
                    if tracked:
                        print_poses(poses_robot, tracked=tracked, step=frame_idx)
                    cv2.putText(annotated,
                                f"RealSense | Frame {frame_idx} | "
                                f"{len(poses_robot)} det. | {mot.n_confirmed} tracks",
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                    cv2.imshow("Phase 2 — RealSense RGB-D", annotated)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        break
                    if key == ord('s'):
                        fname = f"test_images/frame_{frame_idx:04d}.jpg"
                        os.makedirs("test_images", exist_ok=True)
                        cv2.imwrite(fname, annotated)
                        print(f"Frame sauvegardé : {fname}")
                    frame_idx += 1
        else:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                detections, poses_robot, tracked, annotated = run_pipeline_on_frame(
                    frame, cam, tf, args.method, yolo_model,
                    args.conf, args.size, args.z, mot=mot
                )
                if tracked:
                    print_poses(poses_robot, tracked=tracked, step=frame_idx)
                cv2.putText(annotated,
                            f"Frame {frame_idx} | {len(poses_robot)} det. | "
                            f"{mot.n_confirmed} tracks",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                cv2.imshow("Phase 2 — Pipeline temps réel", annotated)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                if key == ord('s'):
                    fname = f"test_images/frame_{frame_idx:04d}.jpg"
                    os.makedirs("test_images", exist_ok=True)
                    cv2.imwrite(fname, annotated)
                    print(f"Frame sauvegardé : {fname}")
                frame_idx += 1
            cap.release()

        cv2.destroyAllWindows()

    else:
        print("Aucune source fournie. Utiliser --image, --video, --live ou --generate-test.")
        print("Exemple : python main_phase2.py --generate-test")


if __name__ == "__main__":
    main()
