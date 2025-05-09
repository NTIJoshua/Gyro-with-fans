#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Motor pins (L293D)
const int leftMotor1 = 5;
const int leftMotor2 = 6;
const int rightMotor1 = 10;
const int rightMotor2 = 9;

void setup() {
  Wire.begin();
  mpu.initialize();
  
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  
  // Stop motors
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, LOW);
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, LOW);
}

void loop() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  
  // Simple tilt detection
  if (ay > 5000) { // Tilted left
    digitalWrite(leftMotor1, HIGH);  // Left motor on
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, LOW);
  } 
  else if (ay < -5000) { // Tilted right
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, HIGH); // Right motor on
    digitalWrite(rightMotor2, LOW);
  }
  else { // Level
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, LOW);
  }
  
  delay(100);
}