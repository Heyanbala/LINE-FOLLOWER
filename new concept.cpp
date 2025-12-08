/*===========================================
DATE:6TH DEC 25
DESCRIBTION: NEW CONCEPT
BY: HEYAN
===========================================*/
#include <Wire.h>

// ==========================================
//          CLASS: MOTOR DRIVER
// ==========================================
class MotorDriver {
  private:
    int enA, in1, in2; // Right Motor Pins
    int enB, in3, in4; // Left Motor Pins

  public:
    MotorDriver(int eA, int i1, int i2, int eB, int i3, int i4) {
      enA = eA; in1 = i1; in2 = i2;
      enB = eB; in3 = i3; in4 = i4;
    }

    void init() {
      pinMode(enA, OUTPUT); pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
      pinMode(enB, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
    }

    void drive(int speedLeft, int speedRight) {
      // Constrain speed to PWM limits
      speedLeft = constrain(speedLeft, -255, 255);
      speedRight = constrain(speedRight, -255, 255);

      // --- Left Motor Logic ---
      if (speedLeft >= 0) {
        digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
        analogWrite(enB, speedLeft);
      } else {
        digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
        analogWrite(enB, -speedLeft);
      }

      // --- Right Motor Logic ---
      if (speedRight >= 0) {
        digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
        analogWrite(enA, speedRight);
      } else {
        digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
        analogWrite(enA, -speedRight);
      }
    }
    
    void stop() {
      drive(0, 0);
    }
};

// ==========================================
//          CLASS: PID CONTROLLER
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
      // Proportional
      float P = error * kp;
      
      // Integral
      integral += error * dt;
      // Safety: Clamp Integral to prevent "Windup"
      integral = constrain(integral, -1000, 1000); 
      float I = integral * ki;
      
      // Derivative
      float D = ((error - previousError) / dt) * kd;
      previousError = error;

      return P + I + D;
    }
};

// ==========================================
//          CLASS: FUSION ROBOT
// ==========================================
class FusionRobot {
  private:
    int pinL, pinC, pinR;
    const int MPU_ADDR = 0x68;
    float gyroErrorZ;
    float currentPosition; // -1000 (Left) to 1000 (Right)
    float fusionGain;      // 0.0 = Trust Gyro, 1.0 = Trust IR

  public:
    FusionRobot(int l, int c, int r) {
      pinL = l; pinC = c; pinR = r;
      currentPosition = 0;
      fusionGain = 0.1; // Default: Trust Gyro mostly, correct with IR
    }

    void init() {
      pinMode(pinL, INPUT); pinMode(pinC, INPUT); pinMode(pinR, INPUT);
      
      Wire.begin();
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x6B); Wire.write(0); Wire.endTransmission(true);

      // Calibrate Gyro on startup
      calibrateGyro();
    }

    void calibrateGyro() {
      Serial.println("Calibrating... Stay Still!");
      float sum = 0;
      for(int i=0; i<500; i++) {
        sum += readRawGyro();
        delay(2);
      }
      gyroErrorZ = sum / 500.0;
    }

    int16_t readRawGyro() {
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x47);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_ADDR, 2, true);
      return (Wire.read() << 8 | Wire.read());
    }

    // This is the "Sensor Fusion" Core
    float getPosition(float dt) {
      // 1. PREDICT (Dead Reckoning using Gyro)
      float rotationRate = (readRawGyro() - gyroErrorZ) / 131.0; 
      // If we turn Right (+), the line moves Left (-). Multiplier 2.0 is specific to wheel size.
      float prediction = currentPosition + (-rotationRate * 2.0); 

      // 2. MEASURE (IR Sensors)
      int L = digitalRead(pinL);
      int C = digitalRead(pinC);
      int R = digitalRead(pinR);
      int sum = L + C + R;

      if (sum > 0) {
        // Line detected! Update the prediction.
        // Assuming Logic 1 = Black Line
        // L=-1000, C=0, R=1000
        float measuredPos = ((L * -1000) + (C * 0) + (R * 1000)) / sum;
        
        // COMPLEMENTARY FILTER: Merge Prediction and Measurement
        currentPosition = ((1.0 - fusionGain) * prediction) + (fusionGain * measuredPos);
      } else {
        // Gap detected! Trust prediction 100%
        currentPosition = prediction;
      }
      
      return currentPosition;
    }
};

// ==========================================
//              MAIN SKETCH
// ==========================================

// 1. Instantiate Objects
MotorDriver motors(9, 8, 7, 3, 6, 5); // enA, in1, in2, enB, in3, in4
PID steeringPID(0.15, 0.0001, 2.0);   // Kp, Ki, Kd
FusionRobot robot(11, 10, 12);        // L, C, R pins

unsigned long lastTime;
int baseSpeed = 100;

void setup() {
  Serial.begin(9600);
  motors.init();
  robot.init(); // This calibrates the Gyro
  lastTime = millis();
}

void loop() {
  // Time Calculation for Math (dt)
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  // 1. Get Fused Position
  float position = robot.getPosition(dt);

  // 2. Calculate PID Error (Target is 0)
  float error = 0 - position;
  float correction = steeringPID.compute(error, dt);

  // 3. Drive Motors
  motors.drive(baseSpeed + correction, baseSpeed - correction);
  
  // Debugging (Plot this in Serial Plotter)
  Serial.print("Pos:"); Serial.print(position);
  Serial.print(" Err:"); Serial.println(correction);
}
/*----------------------------------------------------------------------------
Why this C++ Structure?
Encapsulation: The variables for the PID (like integral) are hidden inside the PID class. You can't accidentally mess them up in the main loop.
Scalability: If you decide to add 2 more motors or a different sensor, you just update the specific Class, you don't have to rewrite the whole code.
Readability: Your void loop() is now tiny and easy to read. It reads like English: "Get position, compute error, drive motors."
Next Step: When you upload this, open the Serial Plotter (Tools > Serial Plotter) in Arduino. You will see a graph of Pos (Position).

Twist the robot with your hand (wheels off ground).
You should see the Position graph move smoothly because of the Gyro, even if the sensors aren't changing perfectly.
*---------------------------------------------------------------------------*/