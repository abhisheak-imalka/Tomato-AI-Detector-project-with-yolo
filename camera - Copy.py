import serial
import time
import cv2
from ultralytics import YOLO

# Load the trained model
model = YOLO("my_model.pt")

# Initialize camera
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)  # Ensure correct camera access

# Connect to Arduino
arduino = serial.Serial('COM4', 9600)  # Replace 'COM4' with your actual port
time.sleep(2)  # Wait for the connection to establish

while True:
    ret, frame = cap.read()
    if not ret:
        continue  # Skip if frame is not captured

    # Run YOLO detection on the frame
    results = model(frame)

    detected = False  # Flag to track detection
    for result in results:
        if result.boxes:  # If any object is detected
            detected = True
            break

    if detected:
        arduino.write(b'G')  # Send signal to Arduino
    else:
        arduino.write(b'0')  # Send "Not Detected" signal

    cv2.imshow('YOLO Detection', frame)  # Display the frame

    if cv2.waitKey(1) & 0xFF == ord('q'):  # Quit with 'q'
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()
