import cv2
import numpy as np
import time
import os
import logging
import serial

def setup_logging():
    """Configure logging to file with timestamps."""
    os.makedirs('logs', exist_ok=True)
    logging.basicConfig(
        filename='logs/stop_sign.log',
        level=logging.INFO,
        format='%(asctime)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S.%f'
    )
    logging.info("Program started")

def crop_frame(image):
    """Crop the top two-thirds of the image to focus on the relevant area."""
    if len(image.shape) == 3:
        height, length, _ = image.shape
    else:
        height, length = image.shape
    return image[0: height//3*2, 0: length]

def main():
    setup_logging()

    # Initialize serial communication
    try:
        ser = serial.Serial('COM3', 9600, timeout=1)  # Adjust port: 'COM3' (Windows) or '/dev/ttyUSB0' (Linux)
        time.sleep(2)  # Allow time for serial connection to stabilize
        logging.info("Serial port opened successfully")
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        logging.error(f"Error opening serial port: {e}")
        return

    # Load stop sign cascade classifier
    stop_sign_cascade = cv2.CascadeClassifier('stop_sign_classifier_2.xml')
    if stop_sign_cascade.empty():
        print("Error: Could not load stop sign classifier.")
        logging.error("Could not load stop sign classifier")
        ser.close()
        return

    # Initialize webcam
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        logging.error("Could not open webcam")
        ser.close()
        return

    # Get frame dimensions for video writer
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # Initialize video writer
    os.makedirs('stop_sign_videos/out', exist_ok=True)
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    video = cv2.VideoWriter('stop_sign_videos/out/detected_output_serial.avi', fourcc, 20, (width, height))

    # Create display window
    cv2.namedWindow('Stop Sign Detection', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Stop Sign Detection', 640, 480)

    last_detection_time = 0
    send_stop = False

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame from webcam.")
            logging.error("Could not read frame from webcam")
            break

        # Crop frame to focus on top two-thirds
        cropped_frame = crop_frame(frame)

        # Preprocess frame: blur and convert to grayscale
        img_filter = cv2.GaussianBlur(cropped_frame, (5, 5), 0)
        gray_filtered = cv2.cvtColor(img_filter, cv2.COLOR_BGR2GRAY)

        # Detect stop signs
        stop_signs = stop_sign_cascade.detectMultiScale(
            gray_filtered,
            scaleFactor=1.05,
            minNeighbors=15,
            minSize=(30, 30)
        )

        # Handle key presses
        key = cv2.waitKey(1) & 0xFF
        fake_stop_triggered = (key == ord('f'))

        # Handle stop sign detection and serial communication
        current_time = time.time()
        if (len(stop_signs) > 0 or fake_stop_triggered) and not send_stop:
            # Draw rectangles around detected stop signs (if any)
            for (x, y, w, h) in stop_signs:
                cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 255, 0), 3)

            # Send '1' to Arduino and log
            status = "STOP DETECTED (FAKE)" if fake_stop_triggered else "STOP DETECTED"
            print(status)
            logging.info(status)
            ser.write(b'1')
            cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            last_detection_time = current_time
            send_stop = True
        else:
            # Check if 3 seconds have passed since last detection
            if send_stop and (current_time - last_detection_time >= 3):
                print("MOVING")
                logging.info("MOVING")
                ser.write(b'0')
                cv2.putText(frame, "NO STOP SIGN", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                send_stop = False
            # If no stop sign or after 3 seconds, send '0'
            elif not send_stop:
                print("MOVING")
                # logging.info("MOVING")  # Avoid flooding log with MOVING
                ser.write(b'0')
                cv2.putText(frame, "NO STOP SIGN", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Write frame to output video
        video.write(frame)

        # Display frame
        cv2.imshow('Stop Sign Detection', frame)

        # Exit on ESC key
        if key == 27:
            break

    # Cleanup
    logging.info("Program stopped")
    video.release()
    cap.release()
    ser.close()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()