/*================================================================
  FINAL CODE: FUSION PID LINE FOLLOWER (IR + GYRO)
  UPDATED MOTOR DRIVER PINS (Simplified 2-Pin Direction + STBY)
================================================================*/

#include <Arduino.h> // Standard Arduino functions
#include <Wire.h>    // REQUIRED for I2C communication (MPU6050)

// ================================================================
// 1. PIN DEFINITIONS (Matched to your Hardware)
// ================================================================

// --- SENSORS (Analog Pins) ---
const int rightIRPin  = A0; 
const int middleIRPin = A1; 
const int leftIRPin   = A2; 

// --- MPU6050 (I2C Pins) ---
const int SCL_Pin = A5;
const int SDA_Pin = A4;

// --- MOTORS (Simplified L298N/Motor Shield Setup) ---
const PIN_Motor_PWMA 5    // Speed control (PWM) for Motor A (Right)
const PIN_Motor_PWMB 6    // Speed control (PWM) for Motor B (Left)
const PIN_Motor_AIN_1 7   // Direction control for Motor A (Right)
const PIN_Motor_BIN_1 8   // Direction control for Motor B (Left)
const PIN_Motor_STBY  3   // Standby pin


// ================================================================
// 2. GLOBAL CONSTANTS & TUNING PARAMETERS
// ================================================================

const int lineThreshold = 500; // Analog value > 500 means "ON LINE"
const int maxPWM = 255;
int baseSpeed = 100;      // Default cruising speed (0-maxPWM)
int maxSpeed  = 180;      // Cap the maximum speed

// PID Constants (Tuning is ESSENTIAL)
double Kp = 0.15;     // Proportional
double Ki = 0.0001;   // Integral
double Kd = 2.0;      // Derivative


// ==========================================
//          CLASS: MOTOR DRIVER (REVISED)
// ==========================================
class MotorDriver {
  private:
    int pwma, pwmb;  // PWM Pins
    int ain1, bin1;  // Direction Pins (AIN1 for Motor A, BIN1 for Motor B)
    int stby;        // Standby Pin

  public:
    // Constructor using the new defined pins
    MotorDriver(int pA, int pB, int dA, int dB, int s) {
      pwma = pA; pwmb = pB; 
      ain1 = dA; bin1 = dB;
      stby = s;
    }

    void init() {
      pinMode(pwma, OUTPUT); pinMode(pwmb, OUTPUT); 
      pinMode(ain1, OUTPUT); pinMode(bin1, OUTPUT);
      pinMode(stby, OUTPUT); 

      // Enable motor driver
      digitalWrite(stby, HIGH);
      
      // Start stopped
      drive(0, 0);
    }

    void drive(int speedLeft, int speedRight) {
      // Constrain speed to safe range
      speedLeft = constrain(speedLeft, -maxSpeed, maxSpeed);
      speedRight = constrain(speedRight, -maxSpeed, maxSpeed);

      // --- Motor B (Left) Logic ---
      // Direction is set by BIN_1 pin (HIGH = Forward, LOW = Reverse, or vice-versa)
      if (speedLeft >= 0) { // Forward
        digitalWrite(bin1, HIGH); // Assuming HIGH is forward for Left Motor
        analogWrite(pwmb, speedLeft);
      } else { // Backward
        digitalWrite(bin1, LOW);  // Assuming LOW is backward for Left Motor
        analogWrite(pwmb, -speedLeft);
      }

      // --- Motor A (Right) Logic ---
      // Direction is set by AIN_1 pin
      if (speedRight >= 0) { // Forward
        digitalWrite(ain1, HIGH); // Assuming HIGH is forward for Right Motor
        analogWrite(pwma, speedRight);
      } else { // Backward
        digitalWrite(ain1, LOW);  // Assuming LOW is backward for Right Motor
        analogWrite(pwma, -speedRight);
      }
    }
    
    void stop() {
      drive(0, 0);
    }
};

// ==========================================
//          CLASS: PID CONTROLLER
//          (No changes needed here)
// ==========================================
class PID {
  private:
    float kp, ki, kd;
    float previousError, integral;
    
  public:
    PID(float p, float i, float d) {
      kp = p; ki = i; kd = d;
      previousError = 0; integral = 0;
    }

    float compute(float error, float dt) {
      float P = error * kp;
      
      integral += error * dt;
      integral = constrain(integral, -5000, 5000); 
      float I = integral * ki;
      
      float D = ((error - previousError) / dt) * kd;
      previousError = error;

      return P + I + D;
    }
};

// ==========================================
//          CLASS: FUSION ROBOT
//          (No changes needed here)
// ==========================================
class FusionRobot {
  private:
    int pinL, pinC, pinR;
    const int MPU_ADDR = 0x68;
    float gyroErrorZ;       
    float currentPosition;  
    float fusionGain;       

  public:
    FusionRobot(int l, int c, int r) {
      pinL = l; pinC = c; pinR = r;
      currentPosition = 0;
      fusionGain = 0.1; 
    }

    void init() {
      Wire.begin();
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x6B); Wire.write(0); 
      Wire.endTransmission(true);

      calibrateGyro();
    }

    void calibrateGyro() {
      Serial.println("Calibrating Gyro... Keep robot still!");
      float sum = 0;
      for(int i=0; i<500; i++) {
        sum += readRawGyro();
        delay(2);
      }
      gyroErrorZ = sum / 500.0;
      Serial.print("Gyro Bias: "); Serial.println(gyroErrorZ);
    }

    int16_t readRawGyro() {
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x47); 
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_ADDR, 2, true);
      return (Wire.read() << 8 | Wire.read());
    }

    float getPosition(float dt) {
      float rotationRate = (readRawGyro() - gyroErrorZ) / 131.0; 
      float prediction = currentPosition + (-rotationRate * 2.0 * dt); 

      int L = (analogRead(pinL) > lineThreshold);
      int C = (analogRead(pinC) > lineThreshold);
      int R = (analogRead(pinR) > lineThreshold);
      int sum = L + C + R;

      if (sum > 0) {
        float measuredPos = ((L * -1000) + (C * 0) + (R * 1000)) / (float)sum;
        currentPosition = ((1.0 - fusionGain) * prediction) + (fusionGain * measuredPos);
      } else {
        currentPosition = prediction;
      }
      
      return currentPosition;
    }
};

// ==========================================
//              MAIN SKETCH
// ==========================================

// 1. Instantiate Objects
// Motors: (PWMA=5, PWMB=6, AIN1=7, BIN1=8, STBY=3)
MotorDriver motors(PIN_Motor_PWMA, PIN_Motor_PWMB, PIN_Motor_AIN_1, PIN_Motor_BIN_1, PIN_Motor_STBY); 
// PID: (Kp, Ki, Kd)
PID steeringPID(Kp, Ki, Kd);   
// Robot: (L=A2, C=A1, R=A0)
FusionRobot robot(leftIRPin, middleIRPin, rightIRPin);        

unsigned long lastTime;

void setup() {
  Serial.begin(9600);
  motors.init();
  robot.init(); 
  lastTime = millis();
}

void loop() {
  // Time Calculation for Math (dt) in seconds
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  // 1. Get Fused Position 
  float position = robot.getPosition(dt);

  // 2. Calculate PID Error
  float error = 0 - position;
  float correction = steeringPID.compute(error, dt);

  // 3. Drive Motors
  // Motor A (Right) uses PWMA, Motor B (Left) uses PWMB
  float speedRight = baseSpeed + correction;
  float speedLeft  = baseSpeed - correction;
  
  motors.drive(speedLeft, speedRight);
  
  // Debugging
  Serial.print("Pos:"); Serial.print(position);
  Serial.print(" Err:"); Serial.println(correction);
}