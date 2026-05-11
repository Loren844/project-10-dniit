import cv2
from ultralytics import YOLO

# 1. Charger un modèle YOLO pré-entraîné (plus simple pour tester)
model = YOLO('yolov8n.pt') 

# 2. Ouvrir la webcam (0 est l'indice par défaut de la caméra du PC)
cap = cv2.VideoCapture(0)

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