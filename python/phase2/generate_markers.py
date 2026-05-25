"""
generate_markers.py
Génère une planche de marqueurs ArUco imprimables / affichables.

Usage :
    python generate_markers.py              # génère markers_0_to_5.png
    python generate_markers.py --ids 0 1 2  # marqueurs spécifiques
    python generate_markers.py --size 300   # taille en pixels par marqueur
"""

import argparse
import os
import cv2
import numpy as np

ARUCO_DICTS = {
    "5x5_50": cv2.aruco.DICT_5X5_50,
    "4x4_50": cv2.aruco.DICT_4X4_50,
}

def generate_marker_sheet(ids: list[int], size_px: int = 250,
                           dict_name: str = "5x5_50",
                           output: str = "test_images/aruco_markers.png"):
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[dict_name])
    margin = 20
    cols = min(3, len(ids))
    rows = (len(ids) + cols - 1) // cols
    W = cols * (size_px + margin) + margin
    H = rows * (size_px + margin + 30) + margin

    sheet = np.ones((H, W, 3), dtype=np.uint8) * 255

    for idx, marker_id in enumerate(ids):
        row, col = divmod(idx, cols)
        x = margin + col * (size_px + margin)
        y = margin + row * (size_px + margin + 30)

        marker = cv2.aruco.generateImageMarker(aruco_dict, marker_id, size_px)
        marker_bgr = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        sheet[y:y+size_px, x:x+size_px] = marker_bgr

        cv2.putText(sheet, f"ArUco ID {marker_id} ({dict_name})",
                    (x, y + size_px + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (50, 50, 50), 1)

    os.makedirs(os.path.dirname(output) or ".", exist_ok=True)
    cv2.imwrite(output, sheet)
    print(f"Marqueurs générés : {output}")
    print(f"→ Ouvre ce fichier sur ton téléphone et montre-le à la webcam.")
    return output

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ids",  type=int, nargs="+", default=list(range(6)))
    parser.add_argument("--size", type=int, default=250)
    parser.add_argument("--dict", default="5x5_50")
    parser.add_argument("--output", default="test_images/aruco_markers.png")
    args = parser.parse_args()
    generate_marker_sheet(args.ids, args.size, args.dict, args.output)
