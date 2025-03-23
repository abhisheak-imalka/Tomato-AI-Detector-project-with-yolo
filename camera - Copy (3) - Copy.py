import serial
import time
import cv2
import random
import numpy as np
import threading
from ultralytics import YOLO
import tkinter as tk
from tkinter import Label, Entry, Button
from PIL import Image, ImageTk

# Load the YOLO model
model = YOLO("my_model.pt")  # Ensure the correct path to the model

# Global variables
arduino = None  # Serial connection object
running = False  # Flag to start/stop detection
cap = None

# Generate random colors for each class
num_classes = len(model.names)
colors = {i: [random.randint(0, 255) for _ in range(3)] for i in range(num_classes)}

# Function to connect to Arduino
def connect_arduino():
    global arduino
    com_port = com_entry.get()  # Get user input COM port
    try:
        arduino = serial.Serial(com_port, 9600, timeout=1)
        time.sleep(2)  # Allow time for connection
        status_label.config(text=f"Connected to {com_port}", fg="green")
        start_button.config(state="normal")  # Enable start button
    except Exception as e:
        status_label.config(text=f"Error: {str(e)}", fg="red")

# Function to start detection
def start_detection():
    global running, cap
    if running or arduino is None:
        return  # Avoid multiple starts or if Arduino is not connected
    running = True
    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)  # Open webcam
    threading.Thread(target=run_detection, daemon=True).start()  # Run in background

# Function to stop detection
def stop_detection():
    global running, cap
    running = False
    if cap:
        cap.release()
        cap = None
    if arduino:
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
        results = model(frame, conf=0.2)  # Adjust confidence threshold if needed

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

# COM Port Input
com_label = Label(root, text="Enter COM Port:")
com_label.pack()

com_entry = Entry(root)
com_entry.pack()

connect_button = Button(root, text="Connect", command=connect_arduino, bg="blue", fg="white", font=("Arial", 12))
connect_button.pack(pady=5)

status_label = Label(root, text="Not Connected", fg="red")
status_label.pack()

# Camera feed label
camera_label = Label(root)
camera_label.pack()

# Buttons
start_button = Button(root, text="Start Detection", command=start_detection, bg="green", fg="white", font=("Arial", 14), state="disabled")
start_button.pack(pady=10)

stop_button = Button(root, text="Stop Detection", command=stop_detection, bg="red", fg="white", font=("Arial", 14))
stop_button.pack(pady=10)

# Run the GUI
root.mainloop()
