// Pin definitions for IR sensors
const int leftIRPin   = A2;
const int middleIRPin = A1;
const int rightIRPin  = A0;

// Motor pins (provided definitions)
#define PIN_Motor_PWMA 5    // Speed control (PWM) for Motor A
#define PIN_Motor_PWMB 6    // Speed control (PWM) for Motor B
#define PIN_Motor_AIN_1 7   // Direction control for Motor A
#define PIN_Motor_BIN_1 8   // Direction control for Motor B
#define PIN_Motor_STBY  3   // Standby pin

int lineThreshold = 500;  // Adjust based on your testing
int baseSpeed = 50;        // Base forward speed

// PID constants - start with these and adjust
double Kp = 0.5;
double Ki = 0.1;
double Kd = 0.1;

// PID variables
double integral = 0.0;
double lastError = 0.0;

// This variable will hold the last known direction of the line:
// -1 means line was last seen on the left side,
//  0 means line was centered,
// +1 means line was last seen on the right side.
int lastKnownDirection = 0;

void setup() {
  Serial.begin(9600);

  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_STBY, OUTPUT);

  // Enable motor driver
  digitalWrite(PIN_Motor_STBY, HIGH);
}

void loop() {
  int leftVal   = analogRead(leftIRPin);
  int middleVal = analogRead(middleIRPin);
  int rightVal  = analogRead(rightIRPin);

  // Print values for debugging
  Serial.print("L: "); Serial.print(leftVal);
  Serial.print(" M: "); Serial.print(middleVal);
  Serial.print(" R: "); Serial.println(rightVal);

  // Determine if sensors see the line
  bool leftOnLine   = (leftVal > lineThreshold);
  bool middleOnLine = (middleVal > lineThreshold);
  bool rightOnLine  = (rightVal > lineThreshold);

  // Compute a simple error:
  // -1 if line on left, +1 if on right, 0 if centered
  int error = 0;
  if (leftOnLine)   error -= 1;
  if (rightOnLine)  error += 1;
  
  // If no sensors see the line, use the last known direction to keep searching
  if (!leftOnLine && !middleOnLine && !rightOnLine) {
    // Search for the line by rotating based on the previous direction
    Serial.println("No line detected - searching...");
    error = lastKnownDirection;
    searchForLine();  // Retain last direction for searching
  } else {
    // Update lastKnownDirection based on current line position
    if (error < 0) lastKnownDirection = -1;
    else if (error > 0) lastKnownDirection = 1;
    else if (middleOnLine) lastKnownDirection = 0;
  }

  // PID computation
  double pidError = (double)error; // error as a double for PID math
  integral += pidError;
  double derivative = pidError - lastError;
  double pidOutput = (Kp * pidError) + (Ki * integral) + (Kd * derivative);
  lastError = pidError;

  // Map pidOutput to a speed offset
  pidOutput = constrain(pidOutput, -2.0, 2.0); // limit PID output for safety
  int offset = map((int)(pidOutput * 100), -200, 200, -100, 100);

  int leftMotorSpeed = baseSpeed - offset;
  int rightMotorSpeed = baseSpeed + offset;

  // Constrain motor speeds
  leftMotorSpeed = constrain(leftMotorSpeed, 0, 150);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 150);

  // Drive motors with the computed speeds
  Serial.print(" Error: "); Serial.print(error);
  Serial.print(" | PID: "); Serial.print(pidOutput);
  Serial.print(" | Offset: "); Serial.println(offset);

  driveMotors(leftMotorSpeed, rightMotorSpeed);

  delay(100);
}

// Function to make the robot search for the line when it's lost
void searchForLine() {
  // Turn in the appropriate direction based on the last sensor readings
  if (lastKnownDirection == -1) {
    // Last line detected on the left, so turn right
    turnRight();
  } else if (lastKnownDirection == 1) {
    // Last line detected on the right, so turn left
    turnLeft();
  } else {
    // If we are centered or haven't moved, we can choose a direction
    turnRight();  // Arbitrary choice for initial direction
  }
}

void turnLeft() {
  // Turn in place while checking for line center
  analogWrite(PIN_Motor_PWMA, 50);  // Slow left motor
  analogWrite(PIN_Motor_PWMB, 50);  // Slow right motor

  // Start turning
  digitalWrite(PIN_Motor_AIN_1, LOW);  // Left motor backward
  digitalWrite(PIN_Motor_BIN_1, HIGH); // Right motor forward

  unsigned long turnStartTime = millis(); // Record time when turn starts

  while (millis() - turnStartTime < 2000) {  // Turn for 2 seconds or until line is found
    int middleVal = analogRead(middleIRPin); // Read center sensor

    // If the middle sensor detects the line, stop turning
    if (middleVal > lineThreshold) {
      stopMotors();
      return;  // Exit the function, stop turning
    }
    delay(50);  // Small delay to prevent overloading the loop
  }

  // Stop motors after turn duration (if line not found)
  stopMotors();

  // Pause for a moment before continuing
  delay(400);
}

void turnRight() {
  // Turn in place while checking for line center
  analogWrite(PIN_Motor_PWMA, 50);  // Slow left motor
  analogWrite(PIN_Motor_PWMB, 50);  // Slow right motor

  // Start turning
  digitalWrite(PIN_Motor_AIN_1, HIGH);  // Left motor forward
  digitalWrite(PIN_Motor_BIN_1, LOW);   // Right motor backward

  unsigned long turnStartTime = millis(); // Record time when turn starts

  while (millis() - turnStartTime < 2000) {  // Turn for 2 seconds or until line is found
    int middleVal = analogRead(middleIRPin); // Read center sensor

    // If the middle sensor detects the line, stop turning
    if (middleVal > lineThreshold) {
      stopMotors();
      return;  // Exit the function, stop turning
    }
    delay(50);  // Small delay to prevent overloading the loop
  }

  // Stop motors after turn duration (if line not found)
  stopMotors();

  // Pause for a moment before continuing
  delay(400);
}

void driveMotors(int leftSpeed, int rightSpeed) {
  // Set direction pins for forward motion
  digitalWrite(PIN_Motor_AIN_1, HIGH);
  digitalWrite(PIN_Motor_BIN_1, HIGH);

  analogWrite(PIN_Motor_PWMA, leftSpeed);
  analogWrite(PIN_Motor_PWMB, rightSpeed);
}

void stopMotors() {
  analogWrite(PIN_Motor_PWMA, 0);
  analogWrite(PIN_Motor_PWMB, 0);

  digitalWrite(PIN_Motor_AIN_1, LOW);
  digitalWrite(PIN_Motor_BIN_1,LOW);
}