// Take in sensor input and give motor commands for line following

// Pin setup
const int leftSensorPin = A0;  // Phototransistor output
const int centerSensorPin = A1;
const int rightSensorPin = A2;

// Set thresholds
const int leftThreshold = 800;
const int centerThreshold = 800;
const int rightThreshold = 260;

// Initialize sensor on/off variable
boolean leftSensor;
boolean centerSensor;
boolean rightSensor;

// Motor setup idk how to do this


void setup() {
  pinMode(leftSensorPin, INPUT);   // Set sensor pins as input
  pinMode(centerSensorPin, INPUT);
  pinMode(rightSensorPin, INPUT);
  Serial.begin(9600);          // Initialize serial communication for debugging
}

void loop() {
  // Read the sensor value
  int leftSensorValue = analogRead(leftSensorPin);
  int centerSensorValue = analogRead(centerSensorPin);
  int rightSensorValue = analogRead(rightSensorPin);

  // Print sensor values for testing
  // Serial.print(leftSensorValue);
  // Serial.print(" ");
  // Serial.print(centerSensorValue);
  // Serial.print(" ");
  // Serial.println(rightSensorValue);

  if (leftSensorValue <= leftThreshold) {
    leftSensor = false;
  } else {
    leftSensor = true;
  }

  if (centerSensorValue <= centerThreshold) {
    centerSensor = false;
  } else {
    centerSensor = true;
  }
  
  if (rightSensorValue <= rightThreshold) {
    rightSensor = false;
  } else {
    rightSensor = true;
  }

  // Print sensor boolean values for testing
  Serial.print(leftSensor);
  Serial.print(" ");
  Serial.print(centerSensor);
  Serial.print(" ");
  Serial.println(rightSensor);

  delay(100);  // Wait for 100 milliseconds before the next read
}
