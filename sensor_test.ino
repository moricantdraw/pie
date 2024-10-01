// Code for sensors that detects if it sees the tape or not

// Pin setup
const int sensorPin = 2;  // Phototransistor output connected to digital pin 2

void setup() {
  pinMode(sensorPin, INPUT);   // Set sensor pin as input
  Serial.begin(9600);          // Initialize serial communication for debugging
}

void loop() {
  // Read the sensor value
  int sensorValue = digitalRead(sensorPin);
  
  if (sensorValue == HIGH) {
    digitalWrite(ledPin, HIGH);
    Serial.println("floor");
  } else {
    digitalWrite(ledPin, LOW);
    Serial.println("tape");
  }

  delay(100);  // Wait for 100 milliseconds before the next read
}
