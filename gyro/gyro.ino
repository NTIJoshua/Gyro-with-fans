#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Fan control pins
const int FAN1_PIN = 9;
const int FAN2_PIN = 10;

// PID Constants (you'll need to tune these)
float Kp = 2.0;  // Proportional gain
float Ki = 0.1;   // Integral gain
float Kd = 0.5;   // Derivative gain

// Balance variables
float angleY = 0;         // Current tilt angle
float targetAngle = 0;    // Desired balance angle
float error = 0;          // Current error
float lastError = 0;      // Previous error
float integral = 0;       // Integral term
float derivative = 0;     // Derivative term
float output = 0;         // PID output

// Timing variables
unsigned long lastTime = 0;
float deltaTime = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  // Initialize MPU6050
  mpu.initialize();
  
  // Verify connection
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1) {
      // Blink LED to indicate failure
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
    }
  }
  
  // Set up fan pins
  pinMode(FAN1_PIN, OUTPUT);
  pinMode(FAN2_PIN, OUTPUT);
  
  // Stop fans initially
  analogWrite(FAN1_PIN, 0);
  analogWrite(FAN2_PIN, 0);
  
  Serial.println("System initialized");
  delay(1000); // Allow sensors to stabilize
}

void loop() {
  // Calculate time difference
  unsigned long currentTime = millis();
  deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds
  lastTime = currentTime;
  
  // Get sensor data
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Calculate tilt angle using complementary filter
  float accelAngle = atan2(ax, az) * 180/PI; // Convert to degrees
  float gyroRate = gy / 131.0; // Convert to degrees/sec
  
  // Complementary filter
  angleY = 0.98 * (angleY + gyroRate * deltaTime) + 0.02 * accelAngle;
  
  // PID calculations
  error = angleY - targetAngle;
  integral += error * deltaTime;
  derivative = (error - lastError) / deltaTime;
  output = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;
  
  // Constrain output to fan speed range (0-255)
  output = constrain(output, -255, 255);
  
  // Control fans based on PID output
  if (output > 0) {
    // Need to correct clockwise - run fan1
    analogWrite(FAN1_PIN, abs(output));
    analogWrite(FAN2_PIN, 0);
  } 
  else if (output < 0) {
    // Need to correct counter-clockwise - run fan2
    analogWrite(FAN1_PIN, 0);
    analogWrite(FAN2_PIN, abs(output));
  }
  else {
    // Balanced - stop both fans
    analogWrite(FAN1_PIN, 0);
    analogWrite(FAN2_PIN, 0);
  }
  
  // Print debug information
  Serial.print("Angle: "); Serial.print(angleY);
  Serial.print(" | Output: "); Serial.print(output);
  Serial.print(" | Fan1: "); Serial.print(output > 0 ? abs(output) : 0);
  Serial.print(" | Fan2: "); Serial.println(output < 0 ? abs(output) : 0);
  
  delay(10); // Small delay to prevent serial flooding
}