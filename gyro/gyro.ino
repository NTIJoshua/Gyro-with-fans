#include <Wire.h>
#include <MPU6050.h>

MPU6050 gyro;

// PID-konstanter för stabilisering
float Kp = 1.5; // Proportionell term
float Ki = 0.01; // Integral term
float Kd = 0.5; // Derivata term

float error, lastError, integral, derivative;

// Variabler för filtrering och vinkelberäkning
float angleX, angleY;
float gyroX, gyroY;
float accelAngleX, accelAngleY;
float alpha = 0.98; // Komplementärfilter viktning

// PWM-pinnar för fläktstyrning
const int fan1 = 9;
const int fan2 = 10;

void setup() {
    Serial.begin(9600);
    Wire.begin();
    gyro.initialize();
    if (!gyro.testConnection()) {
        Serial.println("MPU6050-anslutning misslyckades!");
        while (1);
    }
    Serial.println("MPU6050 anslutet.");
    pinMode(fan1, OUTPUT);
    pinMode(fan2, OUTPUT);
}

void loop() {
    int16_t ax, ay, az, gx, gy, gz;
    gyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Omvandla accelerometerdata till vinkel
    accelAngleX = atan2(ay, az) * 180 / PI;
    accelAngleY = atan2(ax, az) * 180 / PI;

    // Omvandla gyroskopdata till vinkeländring
    gyroX = gx / 131.0; // Skalfaktor för MPU6050 (131 LSB/°/s)
    gyroY = gy / 131.0;

    // Komplementärfilter för att kombinera gyro och accelerometer
    angleX = alpha * (angleX + gyroX * 0.01) + (1 - alpha) * accelAngleX;
    angleY = alpha * (angleY + gyroY * 0.01) + (1 - alpha) * accelAngleY;

    // PID-reglering
    error = angleX;
    integral += error * 0.01;
    derivative = (error - lastError) / 0.01;
    float output = Kp * error + Ki * integral + Kd * derivative;
    lastError = error;

    // Styr fläktarna baserat på PID-output
    int fanSpeed = constrain(abs(output), 0, 255);
    if (output > 0) {
        analogWrite(fan1, fanSpeed);
        analogWrite(fan2, 0);
    } else {
        analogWrite(fan1, 0);
        analogWrite(fan2, fanSpeed);
    }

    // Skriv ut data för debugging
    Serial.print("Angle X: "); Serial.print(angleX);
    Serial.print(" | Angle Y: "); Serial.println(angleY);
    Serial.print(" | Fan Speed: "); Serial.println(fanSpeed);

    delay(10); // Loop-fördröjning

}
