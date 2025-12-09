/*================================================================
  FINAL CODE: FUSION PID LINE FOLLOWER (IR + GYRO)
  
  ** USES ANALOG IR SENSORS (A2, A1, A0) FOR MEASUREMENT **
================================================================*/

#include <Arduino.h> // Standard Arduino functions
#include <Wire.h>    // REQUIRED for I2C communication (MPU6050)

// ================================================================
// 1. PIN DEFINITIONS (Matched to your Hardware)
// ================================================================

// --- SENSORS (Analog Pins) ---
// Note: These pins must be used with analogRead()
const int pinL = A2;  // Left IR Sensor
const int pinC = A1;  // Center IR Sensor
const int pinR = A0;  // Right IR Sensor

// --- MPU6050 (I2C Pins) ---
const int MPU_ADDR = 0x68;
const int SCL_Pin = A5; // Clock (Handled by Wire.h)
const int SDA_Pin = A4; // Data (Handled by Wire.h)

// --- MOTORS (Simplified 2-Pin Direction + STBY Setup) ---
const int PIN_Motor_PWMA = 5;    // PWM Speed for Motor A (Right)
const int PIN_Motor_PWMB = 6;    // PWM Speed for Motor B (Left)
const int PIN_Motor_AIN_1 = 7;   // Direction control for Motor A (Right)
const int PIN_Motor_BIN_1 = 8;   // Direction control for Motor B (Left)
const int PIN_Motor_STBY = 3;    // Standby pin


// ================================================================
// 2. GLOBAL CONSTANTS & TUNING PARAMETERS
// ================================================================

const int lineThreshold = 500; // Analog value > 500 means "ON LINE"
const int maxPWM = 255;
int baseSpeed = 50;      // Default cruising speed (0-maxPWM)
int maxSpeed  = 100;      // Cap the maximum speed

// PID Constants (Tuning is ESSENTIAL)
double Kp = 0.02;     // Proportional
double Ki = 0.0001;   // Integral
double Kd = 2.0;      // Derivative

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


// ==========================================
//          CLASS: MOTOR DRIVER 
// ==========================================
class MotorDriver {
  private:
    int pwma, pwmb;  // PWM Pins
    int ain1, bin1;  // Direction Pins 
    int stby;        // Standby Pin

  public:
    MotorDriver(int pA, int pB, int dA, int dB, int s) {
      pwma = pA; pwmb = pB; 
      ain1 = dA; bin1 = dB;
      stby = s;
    }

    void init() {
      pinMode(pwma, OUTPUT); 
      pinMode(pwmb, OUTPUT); 
      pinMode(ain1, OUTPUT); 
      pinMode(bin1, OUTPUT);
      pinMode(stby, OUTPUT); 

      // Enable motor driver
      digitalWrite(stby, HIGH);
      drive(0, 0);
    }

    void drive(int speedLeft, int speedRight) {
      speedLeft = constrain(speedLeft, -maxSpeed, maxSpeed);
      speedRight = constrain(speedRight, -maxSpeed, maxSpeed);

      // --- Motor B (Left) Logic ---
      if (speedLeft >= 0) { // Forward
        digitalWrite(bin1, HIGH); 
        analogWrite(pwmb, speedLeft);
      } else { // Backward
        digitalWrite(bin1, LOW);  
        analogWrite(pwmb, -speedLeft);
      }

      // --- Motor A (Right) Logic ---
      if (speedRight >= 0) { // Forward
        digitalWrite(ain1, HIGH); 
        analogWrite(pwma, speedRight);
      } else { // Backward
        digitalWrite(ain1, LOW);  
        analogWrite(pwma, -speedRight);
      }
    }
    
    void stop() {
      drive(0, 0);
    }
};

// ==========================================
//          CLASS: PID CONTROLLER
//          (Simplified as a global function)
// ==========================================
float computePID(float error, float dt) {
  // P Term
  float P = error;
  
  // I Term
  integral += error * dt;
  // Anti-windup (Limit I to prevent getting stuck)
  integral = constrain(integral, -1000, 1000); 
  
  // D Term
  float D = (error - lastError) / dt;
  lastError = error;

  // PID Calculation
  return (Kp * P) + (Ki * integral) + (Kd * D);
}


// ==========================================
//          HELPER FUNCTIONS
// ==========================================

int16_t readRawGyro() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47); // Register for Gyro Z
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);
  return (Wire.read() << 8 | Wire.read());
}

// ------------------------------------------
//            MAIN SETUP AND LOOP
// ------------------------------------------

// 1. Instantiate Objects
MotorDriver motors(PIN_Motor_PWMA, PIN_Motor_PWMB, PIN_Motor_AIN_1, PIN_Motor_BIN_1, PIN_Motor_STBY); 

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // --- PIN MODES ---
  motors.init(); // Sets motor pin modes and enables standby
  
  // --- GYRO SETUP & CALIBRATION ---
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // Power Mgmt
  Wire.write(0);    // Wake up
  Wire.endTransmission(true);

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
  float rotationRate = (readRawGyro() - gyroErrorZ) / 131.0;
  float predictedChange = -rotationRate * GYRO_SCALE * dt; // Multiply by dt
  estimatedPosition += predictedChange;

  // B. MEASUREMENT (IR Analog Reading)
  // Convert analog reading to boolean (1=On Line, 0=Off Line)
  int L = (analogRead(pinL) > lineThreshold); 
  int C = (analogRead(pinC) > lineThreshold);
  int R = (analogRead(pinR) > lineThreshold); 
  
  int sum = L + C + R;
  
  if (sum > 0) {
    // Calculate Weighted Position: L = -1000, C = 0, R = 1000
    float measuredPos = ((L * -1000) + (C * 0) + (R * 1000)) / (float)sum;
    
    // FUSION: Combine Prediction (Smooth Gyro) with Measurement (Accurate IR)
    // Complementary Filter: Fused = (Trust Gyro) * Prediction + (Trust IR) * Measurement
    estimatedPosition = ((1.0 - FILTER_GAIN) * estimatedPosition) + (FILTER_GAIN * measuredPos);
  } 
  // If sum == 0 (GAP DETECTED), the 'estimatedPosition' retains the Gyro prediction.

  // ==========================================
  //        STEP 2: PID CONTROL
  // ==========================================
  
  float error = 0 - estimatedPosition;
  float correction = computePID(error, dt);

  // ==========================================
  //        STEP 3: MOTOR DRIVER
  // ==========================================
  
  // Calculate differential speeds
  float speedLeft = baseSpeed - correction;
  float speedRight = baseSpeed + correction; // Note the addition here
  
  // Constrain speeds to 0-maxPWM range 
  // (The drive function handles the forward/reverse logic for negative speeds)
  speedLeft = constrain(speedLeft, -maxPWM, maxPWM);
  speedRight = constrain(speedRight, -maxPWM, maxPWM);
  
  motors.drive(speedLeft, speedRight);
  
  // Debugging
  Serial.print("Pos:"); Serial.print(estimatedPosition);
  Serial.print(" Err:"); Serial.println(correction);
}