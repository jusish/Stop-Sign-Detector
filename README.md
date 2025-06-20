# 🚦 Real-Time Stop Sign Detection with Arduino Control

This project uses OpenCV and a trained Haar Cascade classifier to detect stop signs in real-time via webcam. When a stop sign is detected, a command is sent to an Arduino (`1` for stop, `0` for move). It logs events with timestamps and can simulate detection using the `f` key.

---

## 🧠 Features

- Real-time stop sign detection using a webcam  
- Sends serial commands to an Arduino for robotic control  
- Fake stop simulation with keyboard input  
- Logs detections and state changes with timestamps  
- Automatically resumes movement after 3 seconds  
- Records the detection video output  

---

## 🛠️ How It Works

1. Captures webcam video.
2. Crops top two-thirds of each frame for better focus.
3. Applies blur and grayscale conversion.
4. Detects stop signs using a Haar cascade.
5. Sends `'1'` to Arduino when a stop sign (or fake stop) is detected.
6. Sends `'0'` after 3 seconds or if no stop sign is seen.
7. Saves event logs and video of detection.
8. Allows manual testing with the **`f` key** (fake stop).

---

## 🔌 Arduino Setup

- Upload `stop_sign_control.ino` to your Arduino via the Arduino IDE.
- Arduino's **Pin 13 LED**:
  - `ON` = Stop detected
  - `OFF` = Move command
- Update the serial port in `stop_sign_detection_serial.py`:
  - Windows: `COM3`, `COM4`, etc.
  - Linux/macOS: `/dev/ttyUSB0`, `/dev/ttyACM0`

---

## 🧰 Installation

Install required Python packages:

```bash
pip install opencv-python numpy pyserial
```

## Thank you!!!