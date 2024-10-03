// Code for sensors that detects if it sees the tape or not

// Pin setup
const int sensorPin = A0;  // Phototransistor output

void setup() {
  pinMode(sensorPin, INPUT);   // Set sensor pin as input
  Serial.begin(9600);          // Initialize serial communication for debugging
}

void loop() {
  // Read the sensor value
  int sensorValue = analogRead(sensorPin);

  Serial.println(sensorValue);
  
  // if (sensorValue == HIGH) {
  //   Serial.println("HIGH");
  // } else {
  //   Serial.println("LOW");
  // }

  delay(100);  // Wait for 100 milliseconds before the next read
}
