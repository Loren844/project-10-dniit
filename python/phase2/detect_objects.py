"""
detect_objects.py
Détection d'objets par deux méthodes complémentaires :
  1. Marqueurs ArUco  — pose exacte, sans apprentissage (calibration / tests)
  2. YOLO             — détection d'objets quelconques sur le tapis roulant

Les deux méthodes retournent des objets Detection normalisés pour que le reste
du pipeline (pose_estimation.py) soit indépendant de la méthode utilisée.

Usage direct (test sur image) :
    python detect_objects.py --image test_images/scene.jpg --method aruco
    python detect_objects.py --image test_images/scene.jpg --method yolo --model yolov8n.pt
    python detect_objects.py --live  --method aruco

Structure de retour (Detection) :
    .method       str          'aruco' | 'yolo'
    .label        str          ID ArUco ou classe YOLO
    .confidence   float        1.0 pour ArUco, score YOLO sinon
    .bbox         (x,y,w,h)    Bounding box en pixels
    .corners      np.ndarray   Coins 2D (ArUco: 4×2, YOLO: None)
    .center_px    (cx, cy)     Centre en pixels
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np


# ---------------------------------------------------------------------------
@dataclass
class Detection:
    method:     str
    label:      str
    confidence: float
    bbox:       tuple           # (x, y, w, h) en pixels
    corners:    Optional[np.ndarray] = field(default=None)  # (4, 2) pour ArUco

    @property
    def center_px(self) -> tuple[float, float]:
        x, y, w, h = self.bbox
        return (x + w / 2.0, y + h / 2.0)


# ---------------------------------------------------------------------------
# --- ArUco ------------------------------------------------------------------
# ---------------------------------------------------------------------------

# Dictionnaires ArUco supportés (par taille croissante)
ARUCO_DICTS = {
    "4x4_50":   cv2.aruco.DICT_4X4_50,
    "4x4_100":  cv2.aruco.DICT_4X4_100,
    "5x5_50":   cv2.aruco.DICT_5X5_50,
    "5x5_100":  cv2.aruco.DICT_5X5_100,
    "6x6_50":   cv2.aruco.DICT_6X6_50,
    "6x6_100":  cv2.aruco.DICT_6X6_100,
}


def detect_aruco(
    frame: np.ndarray,
    dict_name: str = "5x5_50",
    refine: bool = True,
) -> list[Detection]:
    """
    Détecte les marqueurs ArUco dans une image BGR.

    Paramètres
    ----------
    frame     : image BGR (H × W × 3)
    dict_name : clé dans ARUCO_DICTS
    refine    : si True, affine les coins sous-pixel

    Retourne
    --------
    liste de Detection (une par marqueur détecté)
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    aruco_dict   = cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[dict_name])
    params       = cv2.aruco.DetectorParameters()
    detector     = cv2.aruco.ArucoDetector(aruco_dict, params)
    corners_list, ids, _ = detector.detectMarkers(gray)

    detections: list[Detection] = []
    if ids is None:
        return detections

    if refine:
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-4)
        corners_list = [
            cv2.cornerSubPix(gray, c, (5, 5), (-1, -1), criteria)
            for c in corners_list
        ]

    for corners, marker_id in zip(corners_list, ids.ravel()):
        pts = corners.reshape(4, 2)  # (4, 2)
        x_coords = pts[:, 0]
        y_coords = pts[:, 1]
        x0, y0 = int(x_coords.min()), int(y_coords.min())
        w  = int(x_coords.max()) - x0
        h  = int(y_coords.max()) - y0
        detections.append(Detection(
            method     = "aruco",
            label      = str(marker_id),
            confidence = 1.0,
            bbox       = (x0, y0, w, h),
            corners    = pts,
        ))
    return detections


# ---------------------------------------------------------------------------
# --- YOLO -------------------------------------------------------------------
# ---------------------------------------------------------------------------

def detect_yolo(
    frame: np.ndarray,
    model_path: str,
    conf_threshold: float = 0.5,
    target_classes: Optional[list[str]] = None,
) -> list[Detection]:
    """
    Détecte des objets avec un modèle YOLO (ultralytics).

    Paramètres
    ----------
    frame           : image BGR
    model_path      : chemin vers le modèle (.pt) — ex: 'yolov8n.pt'
    conf_threshold  : seuil de confiance minimal
    target_classes  : si fourni, filtre uniquement ces classes (ex: ['bottle', 'box'])

    Retourne
    --------
    liste de Detection
    """
    try:
        from ultralytics import YOLO  # type: ignore
    except ImportError:
        raise ImportError(
            "Le module 'ultralytics' est requis pour la détection YOLO.\n"
            "Installer avec : pip install ultralytics"
        )

    model   = YOLO(model_path)
    results = model(frame, verbose=False)[0]

    detections: list[Detection] = []
    for box in results.boxes:
        conf  = float(box.conf[0])
        if conf < conf_threshold:
            continue
        cls_id = int(box.cls[0])
        label  = model.names[cls_id]
        if target_classes and label not in target_classes:
            continue
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        detections.append(Detection(
            method     = "yolo",
            label      = label,
            confidence = conf,
            bbox       = (x1, y1, x2 - x1, y2 - y1),
            corners    = None,
        ))
    return detections


# ---------------------------------------------------------------------------
# --- Visualisation ----------------------------------------------------------
# ---------------------------------------------------------------------------

def draw_detections(frame: np.ndarray, detections: list[Detection]) -> np.ndarray:
    """Dessine les détections sur une copie de l'image."""
    out = frame.copy()
    for d in detections:
        x, y, w, h = d.bbox
        color = (0, 255, 0) if d.method == "aruco" else (255, 128, 0)

        # Bounding box
        cv2.rectangle(out, (x, y), (x + w, y + h), color, 2)

        # Coins ArUco (quadrilatère précis)
        if d.corners is not None:
            pts = d.corners.astype(int)
            cv2.polylines(out, [pts], isClosed=True, color=(0, 200, 255), thickness=2)
            for i, (px, py) in enumerate(pts):
                cv2.circle(out, (px, py), 5, (0, 0, 255) if i == 0 else (255, 255, 0), -1)

        # Label
        label_str = f"{d.method}:{d.label} ({d.confidence:.2f})"
        cv2.putText(out, label_str, (x, max(y - 8, 15)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # Centre
        cx, cy = d.center_px
        cv2.drawMarker(out, (int(cx), int(cy)), (0, 255, 255),
                       cv2.MARKER_CROSS, 12, 2)
    return out


# ---------------------------------------------------------------------------
# --- CLI --------------------------------------------------------------------
# ---------------------------------------------------------------------------

def _parse_args():
    parser = argparse.ArgumentParser(description="Détection d'objets (ArUco / YOLO)")
    src = parser.add_mutually_exclusive_group(required=True)
    src.add_argument("--image", metavar="FILE", help="Image de test")
    src.add_argument("--live",  action="store_true", help="Flux webcam")
    parser.add_argument("--method", choices=["aruco", "yolo"], default="aruco")
    parser.add_argument("--dict",   default="5x5_50",
                        help="Dictionnaire ArUco (défaut 5x5_50)")
    parser.add_argument("--model",  default="yolov8n.pt",
                        help="Modèle YOLO (.pt)")
    parser.add_argument("--conf",   type=float, default=0.5,
                        help="Seuil de confiance YOLO")
    parser.add_argument("--cam",    type=int,   default=0)
    return parser.parse_args()


def _run_detection(frame: np.ndarray, args) -> list[Detection]:
    if args.method == "aruco":
        return detect_aruco(frame, dict_name=args.dict)
    else:
        return detect_yolo(frame, model_path=args.model, conf_threshold=args.conf)


def main():
    args = _parse_args()

    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            print(f"Impossible de lire : {args.image}", file=sys.stderr)
            sys.exit(1)
        detections = _run_detection(frame, args)
        print(f"{len(detections)} détection(s):")
        for d in detections:
            print(f"  [{d.method}] label={d.label}  conf={d.confidence:.2f}"
                  f"  bbox={d.bbox}  centre={d.center_px}")
        out = draw_detections(frame, detections)
        cv2.imshow("Détections", out)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    else:  # live
        cap = cv2.VideoCapture(args.cam)
        if not cap.isOpened():
            print(f"Impossible d'ouvrir la caméra {args.cam}", file=sys.stderr)
            sys.exit(1)
        print("Appuyer sur 'q' pour quitter.")
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            detections = _run_detection(frame, args)
            out = draw_detections(frame, detections)
            cv2.putText(out, f"{len(detections)} objet(s)", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
            cv2.imshow("Détection temps réel", out)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
