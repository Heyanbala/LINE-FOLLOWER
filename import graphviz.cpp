// ================================================================
// LINE FOLLOWER: PID CONTROL (Specific Pinout Version)
// Sensors: L=A2, M=A1, R=A0
// Motors:  Custom 5-Pin Setup (Dir + PWM)
// ================================================================

// --- SENSORS ---
const int leftIRPin   = A2; // Left Sensor
const int middleIRPin = A1; // Middle Sensor
const int rightIRPin  = A0; // Right Sensor

// --- MOTOR PINS (User Provided) ---
#define PIN_Motor_PWMA 5    // Speed control (PWM) for Motor A (Left?)
#define PIN_Motor_PWMB 6    // Speed control (PWM) for Motor B (Right?)
#define PIN_Motor_AIN_1 7   // Direction control for Motor A
#define PIN_Motor_BIN_1 8   // Direction control for Motor B
#define PIN_Motor_STBY  3   // Standby pin

// --- SETTINGS ---
int lineThreshold = 500;   // > 500 = Black Line, < 500 = White Surface
int baseSpeed = 80;        // Base speed (0-255). Lower is safer for testing.
int maxSpeed = 150;        // Limit max speed to prevent flying off track

// --- PID CONSTANTS ---
double Kp = 25.0;   // Proportional (Main turning power)
double Ki = 0.0;    // Integral (Accumulates error over time - keep 0 usually)
double Kd = 15.0;   // Derivative (Dampens the wobble)

// --- PID VARIABLES ---
double integral = 0.0;
double lastError = 0.0;
int lastKnownDirection = 0; // -1: Left, 1: Right

void setup() {
  Serial.begin(9600);

  // Sensor Pins
  pinMode(leftIRPin, INPUT);
  pinMode(middleIRPin, INPUT);
  pinMode(rightIRPin, INPUT);

  // Motor Pins
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_STBY, OUTPUT);

  // Enable Motor Driver
  digitalWrite(PIN_Motor_STBY, HIGH);
  
  Serial.println("System Ready. Waiting 2 seconds...");
  delay(2000);
}

void loop() {
  // 1. READ SENSORS (Analog 0-1023)
  int leftVal   = analogRead(leftIRPin);
  int middleVal = analogRead(middleIRPin);
  int rightVal  = analogRead(rightIRPin);

  // Convert to Boolean (True = Line Detected)
  bool leftOnLine   = (leftVal > lineThreshold);
  bool middleOnLine = (middleVal > lineThreshold);
  bool rightOnLine  = (rightVal > lineThreshold);

  // Debugging (Uncomment to tune threshold)
  // Serial.print(leftVal); Serial.print("\t");
  // Serial.print(middleVal); Serial.print("\t");
  // Serial.println(rightVal);

  // 2. CALCULATE ERROR
  // Target is 0. Negative = Line is to Left. Positive = Line is to Right.
  int error = 0;

  if (leftOnLine)  error -= 1;
  if (rightOnLine) error += 1;

  // Track Last Direction for "Lost Line" logic
  if (error < 0) lastKnownDirection = -1;
  else if (error > 0) lastKnownDirection = 1;
  else if (middleOnLine) lastKnownDirection = 0;

  // 3. LOST LINE LOGIC
  // If all sensors see White (low values), we are lost.
  if (!leftOnLine && !middleOnLine && !rightOnLine) {
    searchForLine();
    return; // Skip PID this loop
  }

  // 4. PID CALCULATIONS
  double pidError = (double)error;
  
  // Integral (only adds up if needed, usually Keep Ki=0 for line followers)
  integral += pidError;
  
  // Derivative (Change since last loop)
  double derivative = pidError - lastError;

  // Final PID Output
  double pidOutput = (Kp * pidError) + (Ki * integral) + (Kd * derivative);

  lastError = pidError;

  // 5. MOTOR CONTROL
  // Adjust speeds based on PID
  // If pidOutput is positive (Error > 0, Line on Right), we need to turn Right.
  // To turn Right: Increase Left Motor, Decrease Right Motor.
  
  int speedA = baseSpeed - pidOutput; // Motor A (Left)
  int speedB = baseSpeed + pidOutput; // Motor B (Right)

  setMotorA(speedA);
  setMotorB(speedB);

  delay(5); // Stability delay
}

// ================================================================
// HELPER FUNCTIONS
// ================================================================

// Control Motor A (Assumes AIN_1 controls direction)
void setMotorA(int speed) {
  speed = constrain(speed, -maxSpeed, maxSpeed);

  if (speed > 0) {
    digitalWrite(PIN_Motor_AIN_1, HIGH); // Forward
    analogWrite(PIN_Motor_PWMA, speed);
  } else {
    digitalWrite(PIN_Motor_AIN_1, LOW);  // Reverse
    analogWrite(PIN_Motor_PWMA, -speed); // PWM must be positive
  }
}

// Control Motor B (Assumes BIN_1 controls direction)
void setMotorB(int speed) {
  speed = constrain(speed, -maxSpeed, maxSpeed);

  if (speed > 0) {
    digitalWrite(PIN_Motor_BIN_1, HIGH); // Forward
    analogWrite(PIN_Motor_PWMB, speed);
  } else {
    digitalWrite(PIN_Motor_BIN_1, LOW);  // Reverse
    analogWrite(PIN_Motor_PWMB, -speed);
  }
}

// Logic to find line if lost
void searchForLine() {
  // If we lost the line, spin in the direction we last saw it.
  int searchSpeed = 90;
  
  if (lastKnownDirection == -1) {
    // Last saw line on Left -> Spin Left
    setMotorA(-searchSpeed);
    setMotorB(searchSpeed);
  } else {
    // Last saw line on Right -> Spin Right
    setMotorA(searchSpeed);
    setMotorB(-searchSpeed);
  }
}