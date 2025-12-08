/*==========================================
DATE:10TH DEC 25
AUTHOR:HEYAN
DESCRIPTION:
      Advanced Line Follower with Gyro Sensor
      Combines IR Sensors and Gyro for Smooth Line Following
      Uses a Fusion Algorithm and PID Control

      Hardware:
      - 2x DC Motors with L298N Driver
      - 3x IR Line Sensors
      - MPU6050 Gyroscope/Accelerometer
    Hardware Wiring Reference
    Motors: Left Front & Rear wired together; Right Front & Rear wired together.
    MPU6050: SDA $\to$ A4, SCL $\to$ A5.
    IR Sensors: Left $\to$ Pin 11, Center $\to$ Pin 10, Right $\to$ Pin 12.
  ==========================================*/

#include <Wire.h>

// ==========================================
//              PIN DEFINITIONS
// ==========================================
// L298N Motor Driver
const int enA = 9;  // Right Speed
const int in1 = 8;  // Right Direction A
const int in2 = 7;  // Right Direction B
const int in3 = 6;  // Left Direction A
const int in4 = 5;  // Left Direction B
const int enB = 3;  // Left Speed

// IR Sensors (0 = White/Line, 1 = Black/NoLine or vice versa depending on sensor)
// ASSUMPTION: This code assumes HIGH (1) = BLACK LINE. 
// If your sensors are LOW for line, invert the logic in readSensors().
const int pinL = 11; 
const int pinC = 10; 
const int pinR = 12;

// MPU6050 Address
const int MPU_ADDR = 0x68;

// ==========================================
//              TUNING PARAMETERS
// ==========================================
// Motor Base Speed (0-255)
int baseSpeed = 120; 

// PID Constants (YOU MUST TUNE THESE)
float Kp = 0.15;  // Proportional (Reaction to error)
float Ki = 0.0001; // Integral (Fixes small persistent errors)
float Kd = 2.0;   // Derivative (Dampens oscillation/wobble)

// Fusion Parameters
float FILTER_GAIN = 0.1; // 0.1 = Trust Sensors 10%, Trust Gyro 90% (Very smooth)
float GYRO_SCALE = 2.0;  // Multiplier to convert Rotation Speed -> Lateral Position change

// ==========================================
//              GLOBAL VARIABLES
// ==========================================
float estimatedPosition = 0; // The Fused Position (-1000 to 1000)
float gyroErrorZ = 0;
unsigned long lastTime;
float lastError = 0;
float integral = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // --- PIN MODES ---
  pinMode(enA, OUTPUT); pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  pinMode(pinL, INPUT); pinMode(pinC, INPUT); pinMode(pinR, INPUT);

  // --- MPU6050 SETUP ---
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // Power Mgmt
  Wire.write(0);    // Wake up
  Wire.endTransmission(true);

  // --- GYRO CALIBRATION ---
  // Robot MUST be still during this!
  Serial.println("Calibrating Gyro... DO NOT MOVE ROBOT");
  for (int i = 0; i < 500; i++) {
    gyroErrorZ += readRawGyro();
    delay(2);
  }
  gyroErrorZ /= 500.0;
  Serial.println("Calibration Done.");
  
  lastTime = millis();
}

void loop() {
  // 1. Calculate Time Delta
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0; // Seconds
  lastTime = currentTime;

  // ==========================================
  //        STEP 1: SENSOR FUSION
  // ==========================================
  
  // A. PREDICTION (Gyroscope)
  // Read rotation rate (deg/sec). Raw / 131.0 = Degrees per second
  float rotationRate = (readRawGyro() - gyroErrorZ) / 131.0;
  
  // Estimate where the line moved based on how much we turned
  // If we turn right (+), the line moves left relative to us (-).
  float predictedChange = -rotationRate * GYRO_SCALE;
  estimatedPosition += predictedChange;

  // B. MEASUREMENT (IR Weighted Average)
  int irValL = digitalRead(pinL);
  int irValC = digitalRead(pinC);
  int irValR = digitalRead(pinR);
  
  // Check if we see the line at all
  int sum = irValL + irValC + irValR;
  
  if (sum > 0) {
    // We see the line! Calculate Weighted Position.
    // Left = -1000, Center = 0, Right = 1000
    float measuredPos = ((irValL * -1000) + (irValC * 0) + (irValR * 1000)) / sum;
    
    // FUSION: Combine Prediction (Smooth) with Measurement (Accurate)
    // estimatedPosition = (0.9 * Prediction) + (0.1 * Measurement)
    estimatedPosition = ((1.0 - FILTER_GAIN) * estimatedPosition) + (FILTER_GAIN * measuredPos);
  } 
  else {
    // GAP DETECTED (sum == 0).
    // Do NOT update with measurement. Trust the Gyro prediction 100%.
    // This allows the robot to "ghost" across gaps.
  }

  // ==========================================
  //        STEP 2: PID CONTROL
  // ==========================================
  
  // Target is 0 (Center). Error = Target - Current
  float error = 0 - estimatedPosition;
  
  // P Term
  float P = error;
  
  // I Term
  integral += error * dt;
  // Anti-windup (Limit I to prevent getting stuck)
  if(integral > 1000) integral = 1000;
  if(integral < -1000) integral = -1000;
  
  // D Term
  float D = (error - lastError) / dt;
  lastError = error;

  // PID Calculation
  float correction = (Kp * P) + (Ki * integral) + (Kd * D);

  // ==========================================
  //        STEP 3: MOTOR DRIVER
  // ==========================================
  
  int speedLeft = baseSpeed + correction;
  int speedRight = baseSpeed - correction;

  // Constrain speeds to valid PWM range (0-255)
  speedLeft = constrain(speedLeft, 0, 255);
  speedRight = constrain(speedRight, 0, 255);

  setMotors(speedLeft, speedRight);
}

// --- HELPER FUNCTIONS ---

int16_t readRawGyro() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47); // Register for Gyro Z
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);
  return (Wire.read() << 8 | Wire.read());
}

void setMotors(int speedL, int speedR) {
  // Left Motor
  if (speedL >= 0) {
    digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
  } else {
    digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
    speedL = -speedL; // Make positive for PWM
  }
  analogWrite(enB, speedL);

  // Right Motor
  if (speedR >= 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
    speedR = -speedR; // Make positive for PWM
  }
  analogWrite(enA, speedR);
}

/*-----------------------------------------------------------------------------------------
How to Tune This (The Concept in Action)
Since this uses a fusion algorithm, you have one extra variable to tune compared to a normal robot.

Tune GYRO_SCALE first:

Set FILTER_GAIN = 0.0 (Trust Gyro 100%).
Place the robot on the line.
Physically twist the robot with your hand. The estimatedPosition (print it to Serial Monitor) should change even though the wheels aren't moving.
Adjust GYRO_SCALE until the robot "fights" you when you twist it.
Tune FILTER_GAIN:

Set it to 0.1 or 0.2.
If the robot is too jittery (shaking on the line), lower this number (trust Gyro more).
If the robot drifts off the line slowly and doesn't correct itself, raise this number (trust sensors more).
Tune PID (Kp, Kd):

Kp: Controls how aggressively it turns. Start small (0.1).
Kd: Controls the "braking." If it wobbles, increase Kd.
Would you like me to explain how to verify if your specific IR sensors send 1 for Black or 0 for Black? That is the most common reason for the code "doing nothing."
------------------------------------------------------------------------------------------*/