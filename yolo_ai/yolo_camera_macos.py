import cv2
import time
from ultralytics import YOLO

# 1. Charger un modèle YOLO pré-entraîné (plus simple pour tester)
model = YOLO('yolov8n.pt') 

# 2. Ouvrir la webcam integree macOS (index 1 car le telephone est sur 0)
cap = cv2.VideoCapture(1)

# macOS (AVFoundation) a besoin d'un instant pour initialiser la caméra
time.sleep(2)

while cap.isOpened():
    success, frame = cap.read()
    if success:
        # Lancer la détection sur l'image de la webcam
        results = model(frame)
        
        # Afficher le résultat à l'écran
        annotated_frame = results[0].plot()
        cv2.imshow("Test Phase 1 - DNIIT", annotated_frame)

        # Appuyer sur 'q' pour quitter
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        break

cap.release()
cv2.destroyAllWindows()