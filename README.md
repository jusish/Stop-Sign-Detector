Stop Sign Detection
In this project, I developed a real-time stop sign detection system using a Haar cascade classifier with OpenCV and integrated it with an Arduino for robotic control. The system detects stop signs via a webcam, sends serial commands (1 for stop, 0 for move) to an Arduino, logs events with timestamps, and supports a fake stop feature for testing.
## Train the Cascade
I trained a Haar Cascade Classifier using OpenCV’s opencv_traincascade utility. I used 4000 negative images (backgrounds) and generated approximately 2000 positive images of stop signs in various settings, combined into a single .vec file. The script haar_cascade_steps.py outlines the training process.
After training for a few hours, a cascade.xml file (e.g., stop_sign_classifier_2.xml) is generated in the output directory.
Setup and Usage

Install Dependencies:
pip install opencv-python numpy pyserial


Directory Structure:
stop-sign-detection/
├── logs/stop_sign.log              # Event logs with timestamps
├── stop_sign_videos/out/           # Output video directory
├── media/                          # Images for README
├── stop_sign_classifier_2.xml      # Trained Haar cascade
├── stop_sign_detection_serial.py   # Python script with detection and serial
├── stop_sign_control.ino           # Arduino sketch
├── haar_cascade_steps.py           # Cascade training script
└── README.md


Arduino Setup:

Upload stop_sign_control.ino to your Arduino via the Arduino IDE.
Update the serial port in stop_sign_detection_serial.py (e.g., COM3 or /dev/ttyUSB0).


Run the Script:
python stop_sign_detection_serial.py


Show a paper stop sign or screen image to the webcam.
Press f for fake stop, ESC to exit.
Check logs/stop_sign.log and stop_sign_videos/out/detected_output_serial.avi for results.



Notes

Ensure stop_sign_classifier_2.xml is in the project root or train a new cascade using haar_cascade_steps.py.
Arduino LED (pin 13) indicates stop (1, ON) or move (0, OFF).
Logs include timestamps for detections, fake stops, and state changes.

