from ultralytics import YOLO
import cv2

# Charger le modèle YOLOv8
model = YOLO("yolov8n.pt")

# Ouvrir la vidéo
video_path = "test.mp4"
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Erreur : impossible d'ouvrir la vidéo.")
    exit()

while True:
    ret, frame = cap.read()

    # Fin de la vidéo
    if not ret:
        break

    # Détection YOLO
    results = model(frame, verbose=False)

    # Dessiner les bounding boxes
    annotated_frame = results[0].plot()

    # Afficher
    cv2.imshow("YOLOv8 - Detection", annotated_frame)

    # Appuyer sur Q pour quitter
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Libérer les ressources
cap.release()
cv2.destroyAllWindows()

