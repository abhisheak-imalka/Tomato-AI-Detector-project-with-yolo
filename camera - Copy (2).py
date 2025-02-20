import serial
import time
import cv2
from ultralytics import YOLO

# Load the trained YOLO model
model = YOLO("my_model.pt")  # Ensure correct path

# Open webcam
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)  # Use DirectShow for better compatibility

# Connect to Arduino
arduino = serial.Serial('COM4', 9600)  # Replace 'COM4' with your port
time.sleep(2)  # Allow time for connection

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        continue  # Skip if no frame is captured

    # Run YOLO detection
    results = model(frame, conf=0.2)  # Lower confidence threshold to catch more objects

    detected = False  # Detection flag

    # Process detection results
    for result in results:
        if result.boxes:  # If there are detected objects
            detected = True
            print("Object Detected!")  # Debugging print

    # Send signal to Arduino based on detection
    if detected:
        arduino.write(b'G')  # Send signal to turn ON LED
        print("Sent 'G' to Arduino")
    else:
        arduino.write(b'0')  # Send signal to turn OFF LED
        print("Sent '0' to Arduino")

    # Show the camera feed
    cv2.imshow('YOLO Detection', frame)

    # Exit on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()
