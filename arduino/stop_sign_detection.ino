/*
  Stop Sign Detection Robot Controller
  Arduino code to receive UART signals from Python stop sign detection system

  Hardware Setup:
  - Arduino Uno/Nano
  - Motor driver (L298N or similar)
  - DC motors for robot movement
  - LED indicators (optional)
  - Serial connection to computer running Python detection

  Pin Configuration:
  - Pin 2, 3: Motor A control
  - Pin 4, 5: Motor B control
  - Pin 6: Motor A speed (PWM)
  - Pin 7: Motor B speed (PWM)
  - Pin 13: Built-in LED (status indicator)
  - Pin 12: External stop indicator LED (red)
  - Pin 11: External move indicator LED (green)
*/

// Motor control pins
const int motorA_pin1 = 2;
const int motorA_pin2 = 3;
const int motorB_pin1 = 4;
const int motorB_pin2 = 5;
const int motorA_speed = 6;  // PWM pin
const int motorB_speed = 7;  // PWM pin

// LED indicator pins
const int statusLED = 13;     // Built-in LED
const int stopLED = 12;       // Red LED for stop indication
const int moveLED = 11;       // Green LED for move indication

// System variables
bool robotStopped = false;
int motorSpeed = 150;         // Motor speed (0-255)
unsigned long lastCommandTime = 0;
const unsigned long commandTimeout = 5000;  // 5 seconds timeout

// Serial communication
String inputString = "";
bool stringComplete = false;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("Arduino Stop Sign Controller Started");
  Serial.println("Waiting for commands from Python detection system...");

  // Initialize motor control pins
  pinMode(motorA_pin1, OUTPUT);
  pinMode(motorA_pin2, OUTPUT);
  pinMode(motorB_pin1, OUTPUT);
  pinMode(motorB_pin2, OUTPUT);
  pinMode(motorA_speed, OUTPUT);
  pinMode(motorB_speed, OUTPUT);

  // Initialize LED pins
  pinMode(statusLED, OUTPUT);
  pinMode(stopLED, OUTPUT);
  pinMode(moveLED, OUTPUT);

  // Initial state - stopped for safety
  stopRobot();

  // Startup LED sequence
  startupSequence();

  // Reserve space for input string
  inputString.reserve(10);
}

void loop() {
  // Check for serial input
  if (stringComplete) {
    processCommand(inputString.trim());
    inputString = "";
    stringComplete = false;
  }

  // Check for command timeout (safety feature)
  if (millis() - lastCommandTime > commandTimeout) {
    // No command received for too long, stop robot for safety
    if (!robotStopped) {
      Serial.println("Command timeout - stopping robot for safety");
      stopRobot();
    }
  }

  // Update status LED (heartbeat)
  updateStatusLED();

  delay(50);  // Small delay for stability
}

void serialEvent() {
  /*
    SerialEvent occurs whenever new data comes in the hardware serial RX.
    This routine is run between each time loop() runs, so using delay inside
    loop can delay response. Multiple bytes of data may be available.
  */
  while (Serial.available()) {
    char inChar = (char)Serial.read();

    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}

void processCommand(String command) {
  lastCommandTime = millis();

  int commandValue = command.toInt();

  if (commandValue == 1) {
    // Stop command received
    if (!robotStopped) {
      Serial.println("STOP command received - stopping robot");
      stopRobot();
    }
  } else if (commandValue == 0) {
    // Move command received
    if (robotStopped) {
      Serial.println("MOVE command received - starting robot");
      moveForward();
    }
  } else {
    Serial.println("Invalid command: " + command);
  }
}

void stopRobot() {
  // Stop all motors
  digitalWrite(motorA_pin1, LOW);
  digitalWrite(motorA_pin2, LOW);
  digitalWrite(motorB_pin1, LOW);
  digitalWrite(motorB_pin2, LOW);
  analogWrite(motorA_speed, 0);
  analogWrite(motorB_speed, 0);

  // Update status
  robotStopped = true;

  // Update LEDs
  digitalWrite(stopLED, HIGH);
  digitalWrite(moveLED, LOW);

  Serial.println("Robot STOPPED");
}

void moveForward() {
  // Move both motors forward
  digitalWrite(motorA_pin1, HIGH);
  digitalWrite(motorA_pin2, LOW);
  digitalWrite(motorB_pin1, HIGH);
  digitalWrite(motorB_pin2, LOW);
  analogWrite(motorA_speed, motorSpeed);
  analogWrite(motorB_speed, motorSpeed);

  // Update status
  robotStopped = false;

  // Update LEDs
  digitalWrite(stopLED, LOW);
  digitalWrite(moveLED, HIGH);

  Serial.println("Robot MOVING FORWARD");
}

void turnLeft() {
  // Turn left by stopping right motor and running left motor
  digitalWrite(motorA_pin1, LOW);
  digitalWrite(motorA_pin2, LOW);
  digitalWrite(motorB_pin1, HIGH);
  digitalWrite(motorB_pin2, LOW);
  analogWrite(motorA_speed, 0);
  analogWrite(motorB_speed, motorSpeed);

  robotStopped = false;
  digitalWrite(stopLED, LOW);
  digitalWrite(moveLED, HIGH);

  Serial.println("Robot TURNING LEFT");
}

void turnRight() {
  // Turn right by stopping left motor and running right motor
  digitalWrite(motorA_pin1, HIGH);
  digitalWrite(motorA_pin2, LOW);
  digitalWrite(motorB_pin1, LOW);
  digitalWrite(motorB_pin2, LOW);
  analogWrite(motorA_speed, motorSpeed);
  analogWrite(motorB_speed, 0);

  robotStopped = false;
  digitalWrite(stopLED, LOW);
  digitalWrite(moveLED, HIGH);

  Serial.println("Robot TURNING RIGHT");
}

void moveBackward() {
  // Move both motors backward
  digitalWrite(motorA_pin1, LOW);
  digitalWrite(motorA_pin2, HIGH);
  digitalWrite(motorB_pin1, LOW);
  digitalWrite(motorB_pin2, HIGH);
  analogWrite(motorA_speed, motorSpeed);
  analogWrite(motorB_speed, motorSpeed);

  robotStopped = false;
  digitalWrite(stopLED, LOW);
  digitalWrite(moveLED, HIGH);

  Serial.println("Robot MOVING BACKWARD");
}

void updateStatusLED() {
  // Heartbeat pattern on status LED
  static unsigned long lastBlink = 0;
  static bool ledState = false;

  if (millis() - lastBlink > 1000) {  // Blink every second
    ledState = !ledState;
    digitalWrite(statusLED, ledState);
    lastBlink = millis();
  }
}

void startupSequence() {
  // LED startup sequence to indicate system is ready
  Serial.println("Performing startup sequence...");

  for (int i = 0; i < 3; i++) {
    digitalWrite(statusLED, HIGH);
    digitalWrite(stopLED, HIGH);
    digitalWrite(moveLED, HIGH);
    delay(200);

    digitalWrite(statusLED, LOW);
    digitalWrite(stopLED, LOW);
    digitalWrite(moveLED, LOW);
    delay(200);
  }

  // Final state - show stopped
  digitalWrite(stopLED, HIGH);

  Serial.println("Startup sequence complete - ready for commands");
}

void emergencyStop() {
  // Emergency stop function
  stopRobot();

  // Flash all LEDs rapidly to indicate emergency
  for (int i = 0; i < 10; i++) {
    digitalWrite(statusLED, HIGH);
    digitalWrite(stopLED, HIGH);
    digitalWrite(moveLED, HIGH);
    delay(100);

    digitalWrite(statusLED, LOW);
    digitalWrite(stopLED, LOW);
    digitalWrite(moveLED, LOW);
    delay(100);
  }

  // Return to normal stop state
  digitalWrite(stopLED, HIGH);

  Serial.println("EMERGENCY STOP ACTIVATED");
}

// Additional utility functions for advanced robot control

void setMotorSpeed(int speed) {
  // Set motor speed (0-255)
  motorSpeed = constrain(speed, 0, 255);
  Serial.println("Motor speed set to: " + String(motorSpeed));
}

void calibrateMotors() {
  // Motor calibration sequence
  Serial.println("Starting motor calibration...");

  // Test each motor individually
  Serial.println("Testing Motor A...");
  digitalWrite(motorA_pin1, HIGH);
  digitalWrite(motorA_pin2, LOW);
  analogWrite(motorA_speed, 100);
  delay(1000);
  analogWrite(motorA_speed, 0);

  Serial.println("Testing Motor B...");
  digitalWrite(motorB_pin1, HIGH);
  digitalWrite(motorB_pin2, LOW);
  analogWrite(motorB_speed, 100);
  delay(1000);
  analogWrite(motorB_speed, 0);

  stopRobot();
  Serial.println("Motor calibration complete");
}

void printStatus() {
  // Print current system status
  Serial.println("=== ROBOT STATUS ===");
  Serial.println("Robot Stopped: " + String(robotStopped ? "YES" : "NO"));
  Serial.println("Motor Speed: " + String(motorSpeed));
  Serial.println("Last Command: " + String((millis() - lastCommandTime) / 1000) + " seconds ago");
  Serial.println("Uptime: " + String(millis() / 1000) + " seconds");
  Serial.println("==================");
}