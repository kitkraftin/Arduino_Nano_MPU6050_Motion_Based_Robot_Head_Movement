#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h>

// Initialize MPU6050 and Servo
MPU6050 mpu;
Servo servo;

int servoPin = 9;          // Servo motor signal pin
int servoAngle = 90;       // Neutral position (90 degrees)
const int minAngle = 65;   // Minimum servo angle
const int maxAngle = 113;  // Maximum servo angle

void setup() {
  Serial.begin(9600);
  Wire.begin();            // Initialize I2C communication
  mpu.initialize();        // Initialize MPU6050

  // Check MPU6050 connection
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connected successfully!");
  } else {
    Serial.println("MPU6050 connection failed!");
    while (1);  // Halt if connection fails
  }

  // Attach the servo to the pin
  servo.attach(servoPin);
  servo.write(servoAngle);  // Set servo to neutral position
}

void loop() {
  int16_t ax, ay, az;      // Variables to store accelerometer data

  // Get acceleration values
  mpu.getAcceleration(&ax, &ay, &az);

  // Calculate tilt angle based on Y-axis acceleration
  float tilt = atan2(ay, az) * 180 / PI;  // Tilt angle in degrees
  Serial.print("Y-Axis Tilt Angle: ");
  Serial.println(tilt);

  // Map the tilt angle to servo range (65° to 113°)
  if (tilt < -10) {  // Tilted backward
    servoAngle = minAngle;  // Move servo to minimum angle
  } else if (tilt > 10) {  // Tilted forward
    servoAngle = maxAngle;  // Move servo to maximum angle
  } else {
    servoAngle = 90;  // Neutral position
  }

  // Move the servo
  servo.write(servoAngle);

  // Debugging output
  Serial.print("Servo Angle: ");
  Serial.println(servoAngle);

  delay(50);  // Small delay for stability
}
