import serial
import time
import cv2
import random
import numpy as np
import threading
from ultralytics import YOLO
import tkinter as tk
from tkinter import Label
from PIL import Image, ImageTk

# Load the YOLO model
model = YOLO("my_model.pt")  # Ensure the correct path to the model

# Connect to Arduino
arduino = serial.Serial('COM5', 9600)
time.sleep(2)  # Allow time for connection

# Generate random colors for each class
num_classes = len(model.names)
colors = {i: [random.randint(0, 255) for _ in range(3)] for i in range(num_classes)}

# Global variables
running = False  # Flag to start/stop detection
cap = None

# Function to start detection
def start_detection():
    global running, cap
    if running:
        return  # Avoid multiple starts
    running = True
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)  # Open webcam
    threading.Thread(target=run_detection, daemon=True).start()  # Run in background

# Function to stop detection
def stop_detection():
    global running, cap
    running = False
    if cap:
        cap.release()
        cap = None
    arduino.write(b'0')  # Send signal to turn off LEDs
    print("Detection Stopped")

# YOLO detection function
def run_detection():
    global running, cap
    while running:
        ret, frame = cap.read()
        if not ret:
            continue  # Skip if no frame

        # Run YOLO detection
        results = model(frame, conf=0.2)  # Lower confidence for better detection

        detected = False  # Flag for object detection

        # Process results
        for result in results:
            for box in result.boxes:
                detected = True
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # Bounding box coordinates
                class_id = int(box.cls[0])  # Class ID
                confidence = float(box.conf[0])  # Confidence score
                label = f"{model.names[class_id]}: {confidence:.2f}"

                # Get color for class
                color = colors.get(class_id, (0, 255, 0))

                # Draw bounding box and label
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                print(f"Detected: {label}")

        # Send signal to Arduino
        if detected:
            arduino.write(b'G')  # Turn ON LED
        else:
            arduino.write(b'0')  # Turn OFF LED

        # Convert frame for Tkinter
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(frame)
        imgtk = ImageTk.PhotoImage(image=img)
        camera_label.imgtk = imgtk
        camera_label.configure(image=imgtk)

        if not running:
            break

    cap.release()

# GUI Interface
root = tk.Tk()
root.title("YOLO Object Detection")
root.geometry("800x600")

# Camera feed label
camera_label = Label(root)
camera_label.pack()

# Buttons
start_button = tk.Button(root, text="Start Detection", command=start_detection, bg="green", fg="white", font=("Arial", 14))
start_button.pack(pady=10)

stop_button = tk.Button(root, text="Stop Detection", command=stop_detection, bg="red", fg="white", font=("Arial", 14))
stop_button.pack(pady=10)

# Run the GUI
root.mainloop()
