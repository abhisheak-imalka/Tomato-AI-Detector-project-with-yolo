import serial
import time
import cv2
import random
import numpy as np
from ultralytics import YOLO

# Load the YOLO model
model = YOLO("my_model.pt")  # Ensure correct model path

# Open webcam
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

# Connect to Arduino
arduino = serial.Serial('COM5', 9600)
time.sleep(2)  # Allow time for connection

# Generate random colors for each class
num_classes = len(model.names)
colors = {i: [random.randint(0, 255) for _ in range(3)] for i in range(num_classes)}

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        continue  # Skip if no frame is captured

    # Run YOLO detection
    results = model(frame, conf=0.2)  # Lower confidence for better detection

    detected = False  # Flag to check object detection

    # Process detection results
    for result in results:
        for box in result.boxes:
            detected = True
            x1, y1, x2, y2 = map(int, box.xyxy[0])  # Bounding box coordinates
            class_id = int(box.cls[0])  # Class ID
            confidence = float(box.conf[0])  # Confidence score
            label = f"{model.names[class_id]}: {confidence:.2f}"  # Class name + confidence

            # Get the color for this class
            color = colors.get(class_id, (0, 255, 0))  # Default to green if not found

            # Draw bounding box with unique color
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            print(f"Detected: {label}")  # Print detection info

    # Send signal to Arduino
    if detected:
        arduino.write(b'G')  # Turn ON LED
        print("Sent 'G' to Arduino")
    else:
        arduino.write(b'0')  # Turn OFF LED
        print("Sent '0' to Arduino")

    # Show camera feed with detections
    cv2.imshow('YOLO Detection', frame)

    # Exit on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()
