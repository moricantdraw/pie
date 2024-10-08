// Take in sensor input and output whether each sensor is on the line or not

//Include libraries
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

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
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *left = AFMS.getMotor(1);
Adafruit_DCMotor *right = AFMS.getMotor(2);

void setup() {
  pinMode(leftSensorPin, INPUT);   // Set sensor pins as input
  pinMode(centerSensorPin, INPUT);
  pinMode(rightSensorPin, INPUT);

  AFMS.begin(); //start motors and set initial speed
  left->setSpeed(85);
  right->setSpeed(75);

  Serial.begin(9600);          // Initialize serial communication for debugging
}

void loop() {
  bool leftRead = readSensor(leftSensorPin, leftThreshold);
  bool rightRead = readSensor(rightSensorPin, rightThreshold);

  motorControl(leftRead, rightRead);
  
}

bool readSensor(int sensorPin, int threshold){
  bool reading;

  int sensorVal = analogRead(sensorPin);
  
  if (sensorVal <= threshold){
    reading = false;
  }
  else{
    reading = true;
  }

  return reading;
}


void motorControl(bool leftSense, bool rightSense){
  if (!leftSense && !rightSense){
    left->setSpeed(85); //same!
    right->setSpeed(75);
  }
  else if (leftSense){
    left->setSpeed(65);
    right->setSpeed(75);
  }
  else if(rightSense){
    left->setSpeed(75);
    right->setSpeed(55);
  }



}

// // Read the sensor value
//   int leftSensorValue = analogRead(leftSensorPin);
//   int centerSensorValue = analogRead(centerSensorPin);
//   int rightSensorValue = analogRead(rightSensorPin);

//   // Print sensor values for testing
//   // Serial.print(leftSensorValue);
//   // Serial.print(" ");
//   // Serial.print(centerSensorValue);
//   // Serial.print(" ");
//   // Serial.println(rightSensorValue);

//   if (leftSensorValue <= leftThreshold) {
//     leftSensor = false;
//   } else {
//     leftSensor = true;
//   }

//   if (centerSensorValue <= centerThreshold) {
//     centerSensor = false;
//   } else {
//     centerSensor = true;
//   }
  
//   if (rightSensorValue <= rightThreshold) {
//     rightSensor = false;
//   } else {
//     rightSensor = true;
//   }

//   // Print sensor boolean values for testing
//   Serial.print(leftSensor);
//   Serial.print(" ");
//   Serial.print(centerSensor);
//   Serial.print(" ");
//   Serial.println(rightSensor);

//   delay(100);  // Wait for 100 milliseconds before the next read

