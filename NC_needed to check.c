/*================================================
DATE:10TH DEC 25
BY: HEYAN
DESCRIBTION:
        1. The 4-Motor Setup (Hardware Layer)
        Since you said your 4 motors "act like 2," you are using Skid Steering.

        Wiring: Connect the Left Front & Left Rear motors to the same output on the motor driver (e.g., OUT1/OUT2). Do the same for the Right side.
        Code: You treat it exactly like a 2-motor robot. No special code is needed for the 4 motors.
        2. Weighted Average with Only 3 Sensors?
        Yes, you can do this. While 8 sensors are better, 3 sensors give you a coarse "Analog" position if you arrange them close enough that two sensors can see the line at the same time.

        The Math (Weighted Average for 3 Sensors): Instead of just 0 (Left) and 1 (Right), we assign values:
====================================================*/

Left Sensor: -1000
Center Sensor: 0
Right Sensor: +1000
// Logic to turn 3 digital sensors into one "Analog" Position
// Returns a value between -1000 (Left) and 1000 (Right). 0 is Center.
float getWeightedPosition() {
  int L = digitalRead(leftPin);   // 0 or 1
  int C = digitalRead(centerPin); // 0 or 1
  int R = digitalRead(rightPin);  // 0 or 1

  // Avoid division by zero if robot is lifted up
  int sum = L + C + R;
  if (sum == 0) return previousPosition; // "Memory" of last known spot

  // The Weighted Formula: (Value * Position) / Total_Value
  // Left is -1000, Center is 0, Right is +1000
  float position = ( (L * -1000) + (C * 0) + (R * 1000) ) / sum;
  
  previousPosition = position;
  return position;
}


/*================================================  
3. The "Fusion" Concept (Kalman Filter Lite)
You asked for Sensor Fusion. In a line follower, we fuse Vision (IR Sensors) with Proprioception (Inner Ear/MPU6050).

The Problem: IR sensors are "Absolute" but slow and noisy (they jitter). The MPU6050 is "Relative" and smooth but drifts over time. The Solution: A Complementary Filter (a simplified Kalman Filter).

Step A (Predict): Use the Gyroscope to guess where the line went.
If I turned right by 5 degrees, the line should have moved 'left' relative to me.
Step B (Update): Use the IR Sensors to correct that guess.
The Gyro thinks I'm at -500, but the IR sees I'm at -450. I will trust the IR a little bit to fix my drift.
4. The Complete "Fusion" Code
This is a conceptual implementation of fusing MPU6050 rotation rate with your 3-sensor array.
================================================*/

#include <Wire.h>

// --- PINS ---
// Motor Pins (L298N)
int enA = 9; int in1 = 8; int in2 = 7;
int in3 = 6; int in4 = 5; int enB = 3;
// Sensor Pins
int pinL = 11; int pinC = 10; int pinR = 12;

// --- VARIABLES ---
const int MPU_ADDR = 0x68;
float gyroZ = 0, gyroError = 0;
float estimatedPosition = 0; // THIS is your "Fused" absolute position
float lastTime;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  // 1. Wake MPU
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0); Wire.endTransmission(true);
  
  // 2. Calibrate Gyro (Don't move robot during this!)
  for(int i=0; i<200; i++) {
    gyroError += readGyro();
  }
  gyroError /= 200;

  // 3. Setup Motor/Sensor Pins
  // (Add pinMode lines here...)
}

void loop() {
  float dt = (millis() - lastTime) / 1000.0;
  lastTime = millis();

  // --- STEP 1: PREDICTION (The Gyro) ---
  // Read rotation speed (deg/sec)
  float rotationRate = readGyro() - gyroError; 
  
  // If we turn RIGHT, the line moves LEFT relative to us.
  // We approximate: Position changes based on rotation speed.
  // '50' is a magic number relating angular speed to line position shift. Tune this!
  estimatedPosition += (rotationRate * dt * 50.0); 

  // --- STEP 2: MEASUREMENT (The IR Sensors) ---
  float irPosition = getWeightedPosition(); // Uses the function from section 2

  // --- STEP 3: FUSION (The Filter) ---
  // If sensors see the line (sum > 0), we trust them to fix the Gyro drift.
  // If sensors see NOTHING (Gap), we skip this and rely 100% on Gyro (Prediction).
  
  int sensorSum = digitalRead(pinL) + digitalRead(pinC) + digitalRead(pinR);
  
  if (sensorSum > 0) {
    // 0.05 is the "Trust Factor" (Gain). 
    // Low (0.01) = Trust Gyro (Smooth). High (0.9) = Trust IR (Jittery).
    estimatedPosition = (0.95 * estimatedPosition) + (0.05 * irPosition);
  }

  // --- STEP 4: PID CONTROL ---
  // Now we use 'estimatedPosition' for PID instead of raw sensor data
  int error = 0 - estimatedPosition; // We want position to be 0
  
  // ... Insert standard PID code here using 'error' ...
}

// Helper to read raw Gyro Z
int16_t readGyro() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);
  return (Wire.read() << 8 | Wire.read());
}

/*---------------------------------------------------------------
 Insert getWeightedPosition() function from Section 2 here
Gap Handling: If you lift the robot or hit a white gap, sensorSum becomes 0. The code skips the "Fusion" step and purely uses estimatedPosition += rotationRate. The robot will "remember" it was turning and continue the curve blindly until it finds the line again.
Noise Rejection: If one IR sensor flickers (bad wiring or bad reflection), the Gyro data (which is smooth) dampens the error, so the motors don't jitter.
This video demonstrates a very similar concept where an MPU6050 is used to correct and smooth out the motion of a robot, effectively "fusing" the data for straight lines and turns.

Sensor Fusion (MPU6050 + HMC5883L) || Kalman Filter
---------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------
    Additional Concepts to Improve Performance


1. Dynamic Speed Control (Adaptive Throttle)
Currently, you use a fixed baseSpeed (120).

The Problem: If you set the speed high enough to win a race, the robot will fly off the track on sharp corners. If you set it low enough to survive corners, it will be too slow on the straightaways.
The Concept: Automatically slow down when the error is high (cornering) and speed up when the error is low (straight).
Add this logic right before setMotors:

// ==========================================
//      NEW CONCEPT: DYNAMIC SPEED
// ==========================================

// 1. Define a maximum speed and a turn speed
int maxSpeed = 200; 
int turnSpeed = 100;

// 2. Map the error to a speed. 
// If Error is 0 (Center), go MaxSpeed. 
// If Error is 1000 (Far Left/Right), go TurnSpeed.
int currentBaseSpeed = map(abs(estimatedPosition), 0, 1000, maxSpeed, turnSpeed);

// 3. Use this dynamic base speed for the motor calculation
int speedLeft = currentBaseSpeed + correction;
int speedRight = currentBaseSpeed - correction;
2. Intersection & Finish Line Logic (The "All Black" Problem)
Your current weighted average math has a flaw regarding intersections.

The Math: ((-1000*1) + (0*1) + (1000*1)) / 3 = 0
The Problem: If the robot hits a Crossroad (All sensors see black) or the Finish Line, your math returns 0. The robot thinks, "I am perfectly in the center!" and drives straight through.
The Fix: You need a specific check for when sum == 3 (or sum >= 2 for wider lines).
Update the Measurement Section:

  // ... inside loop ...
  int sum = irValL + irValC + irValR;

  // NEW CONCEPT: Special Case Handling
  if (sum == 3) {
     // ALL SENSORS BLACK
     // Option A: It's a Finish Line -> Stop
     setMotors(0, 0);
     while(1); // Freeze robot
     
     // Option B: It's an intersection -> Force a specific direction?
     // usually better to just keep going straight (which your code does by default) 
     // but be aware that PID will treat this as "Zero Error".
  }
  else if (sum > 0) {
     // ... Your existing weighted average code ...
  }
3. A "Safety Start" Button
Right now, your robot starts moving the nanosecond the Arduino powers up. This makes it hard to calibrate the Gyro (which requires the robot to be perfectly still) because you are usually handling the robot to plug in the battery.

The Concept: Use a push button to "arm" the robot.
The Logic:
Setup runs.
Gyro Calibrates.
Robot waits in a while loop until button is pressed.
Robot starts the main loop.
In Setup:

  pinMode(4, INPUT_PULLUP); // Button on Pin 4

  Serial.println("Ready. Press Button to Start.");
  while(digitalRead(4) == HIGH) {
    // Do nothing. Wait for button press (LOW).
    // This gives you time to put the robot on the line carefully.
  }
  delay(1000); // Give you 1 second to move your hand away
  lastTime = millis(); // Reset timer so D-term doesn't spike
Summary of What to Add
Dynamic Speed: To go fast on straights and safe on curves.
Intersection Handling: To prevent the robot from getting confused by crossroads.
Start Button: To ensure the Gyro calibrates while the robot is truly still.
Critical Note on Performance: In your final run, comment out all Serial.print lines. Serial communication is very slow. Printing to the console takes precious milliseconds, which increases your dt and makes the PID controller react slower. Removing them will make the robot significantly smoother.
-----------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------
    Troubleshooting Physical Issues


1. The "Dead Zone" (Motor Stall Speed)
This is the #1 reason PID feels "sluggish."

The Problem: In code, 1 is greater than 0. In the real world, if you send PWM 20 to a motor, it probably won't move at all. It just hums. Friction is too high.
Why it hurts you: Your PID calculates a small correction (e.g., "Turn left a tiny bit, speed 30"). The motor receives 30, doesn't move, and the error gets worse. The robot looks "drunk" or slow to react.
The Fix: Find your Minimum Start Speed.
Write a simple sketch that increases speed by 1 every 100ms.
See at what number the wheels actually start spinning on the ground.
Let's say it's 60. In your code, you should ensure that if the motor is moving, it never receives less than 60 (unless it's stopping).
2. The L298N Voltage Drop
The Problem: The L298N driver is very old technology. It "eats" about 1.5V to 2V internally.
Example: If you use 4 AA batteries (6V total), your motors are only getting 4V. This makes them weak and unable to correct fast errors.
The Fix: Ensure your battery voltage is at least 2V higher than the motor's rated voltage. (e.g., Use 2x 18650 Li-Ion batteries = 7.4V, which gives the motors a healthy ~5.5V).
3. MPU6050 Vibration Noise
The Problem: The MPU6050 is very sensitive. As your robot drives over the floor, the chassis vibrates. The accelerometer/gyro picks this up as "movement," creating noisy data.
The Symptom: The robot twitches even on a straight line.
The Fix: Soft Mounting. Do not screw the MPU6050 directly into the chassis. Stick it on using a piece of double-sided foam tape or a small piece of sponge. This acts as a shock absorber.
4. Ambient Light Interference (IR Sensors)
The Problem: Your IR sensors work by bouncing light off the floor. If you test in a dark room, it works great. If you take it to a brightly lit competition hall (or sunlight), the external light floods the sensors.
The Symptom: The robot acts blind or thinks everything is White.
The Fix:
Calibration: Make sure you test the sensors in the actual lighting conditions of the track.
The Skirt: If the lighting is causing issues, build a small cardboard "skirt" around the sensors to block outside light, creating a shadow for them to work in.
Final Check: The "Invert" Logic
Before you put the robot on the track, double-check your sensor logic in the code versus the real track.

Your Code currently assumes: HIGH (1) = BLACK LINE.
Some Sensors: Output LOW (0) when they see Black.
Quick Test:

Upload the code with DEBUG 1.
Open Serial Monitor.
Hold the robot in the air.
Put your finger over the sensor (simulating Black).
If the Serial Monitor shows the position changing towards that sensor, you are good.
If it does the opposite (or nothing), you need to change the logic in readSensors.
If you tackle these 4 physical checks, your robot will be incredibly stable. Good luck with the testing!
---------------------------------------------------------*/