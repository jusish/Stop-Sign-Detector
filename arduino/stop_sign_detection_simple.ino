void setup() {
  // Initialize serial communication at 9600 baud
  Serial.begin(9600);

  // Set up LED pin (e.g., pin 13 for built-in LED)
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW); // Start with LED off (MOVING)
}

void loop() {
  // Check if data is available on serial port
  if (Serial.available() > 0) {
    char command = Serial.read(); // Read the incoming byte

    // Process command
    if (command == '1') {
      digitalWrite(13, HIGH); // Turn LED on (STOP)
    } else if (command == '0') {
      digitalWrite(13, LOW);  // Turn LED off (MOVING)
    }
  }
}