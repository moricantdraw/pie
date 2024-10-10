// Take in sensor input and command motors to follow a tape line on the floor

//Include libraries
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Pin setup
const int leftSensorPin = A1;  // Phototransistor output
const int centerSensorPin = A2;
const int rightSensorPin = A3;

// Set thresholds
const int leftThreshold = 790;
const int centerThreshold = 875;
const int rightThreshold = 260;

// Initialize sensor on/off variable
boolean leftSensor;
boolean centerSensor;
boolean rightSensor;

// Motor setup
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *left = AFMS.getMotor(1);
Adafruit_DCMotor *right = AFMS.getMotor(2);

void setup() {
  pinMode(leftSensorPin, INPUT);   // Set sensor pins as input
  pinMode(centerSensorPin, INPUT);
  pinMode(rightSensorPin, INPUT);

  AFMS.begin(); //start motors and set initial speed
  left->setSpeed(20);
  right->setSpeed(20);

  Serial.begin(9600);          // Initialize serial communication for debugging
}

void loop() {
  bool leftRead = readSensor(leftSensorPin, leftThreshold);
  bool rightRead = readSensor(rightSensorPin, rightThreshold);

  Serial.print(leftRead);
  Serial.print(" ");
  Serial.println(rightRead);

  motorControl(leftRead, rightRead);

  // left->run(FORWARD);
  // right->run(FORWARD);
  
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
    left->setSpeed(20); //same!
    right->setSpeed(20);
  }
  else if (leftSense){
    left->setSpeed(0);
    right->setSpeed(40);
  }
  else if(rightSense){
    left->setSpeed(40);
    right->setSpeed(0);
  }
  // else {
  //   left->setSpeed(0);
  //   right->setSpeed(0);
  // }

  left->run(BACKWARD);
  right->run(BACKWARD);


}
