// --- SENSOR PINS (V3 car, digital IR) ---
const int leftIRPin   = 2;
const int middleIRPin = 4;
const int rightIRPin  = 10;

// --- MOTOR PINS (V3 car, L298N-style driver) ---
const int enA = 6;   // PWM left motor
const int in1 = 11;  // Left motor forward
const int in2 = 9;   // Left motor backward
const int enB = 5;   // PWM right motor
const int in3 = 7;   // Right motor forward
const int in4 = 8;   // Right motor backward

// --- SETTINGS ---
const int SPEED_CRUISE = 110;   // Base forward speed
const int SPEED_TURN   = 140;   // Spin speed
const int SPEED_STOP   = 0;
const unsigned long LOST_LINE_TIMEOUT_MS = 1200; // Failsafe timeout

// ==========================================
// 0. HELPER FUNCTIONS
// ==========================================

// Drive motors with signed speeds (-255..255)
void drive(int speedLeft, int speedRight) {
  // Left motor
  if (speedLeft >= 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, speedLeft);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, -speedLeft);
  }

  // Right motor
  if (speedRight >= 0) {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, speedRight);
  } else {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, -speedRight);
  }
}

void stopMotors() {
  analogWrite(enA, SPEED_STOP);
  analogWrite(enB, SPEED_STOP);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

// ==========================================
// 1. BEHAVIOUR INTERFACE
// ==========================================
class Behaviour {
  public:
    virtual bool takeControl() = 0;
    virtual void action() = 0;
    virtual String name() = 0;
};

// ==========================================
// 2. BEHAVIOURS
// ==========================================

// Background cruise
class Cruise : public Behaviour {
  public:
    bool takeControl() override {
      return true;
    }
    void action() override {
      drive(SPEED_CRUISE, SPEED_CRUISE);
    }
    String name() override { return "Cruise"; }
};

// Turn left when left sees line and middle is off
class TurnLeft : public Behaviour {
  public:
    bool takeControl() override {
      int leftVal = digitalRead(leftIRPin);
      int middleVal = digitalRead(middleIRPin);
      return (leftVal == LOW && middleVal == HIGH); // Line is black => LOW
    }
    void action() override {
      drive(-SPEED_TURN, SPEED_TURN);
    }
    String name() override { return "Turn Left"; }
};

// Turn right when right sees line and middle is off
class TurnRight : public Behaviour {
  public:
    bool takeControl() override {
      int rightVal = digitalRead(rightIRPin);
      int middleVal = digitalRead(middleIRPin);
      return (rightVal == LOW && middleVal == HIGH);
    }
    void action() override {
      drive(SPEED_TURN, -SPEED_TURN);
    }
    String name() override { return "Turn Right"; }
};

// Optional failsafe: stop if all sensors white too long
class LostLine : public Behaviour {
  public:
    bool takeControl() override {
      bool leftWhite = digitalRead(leftIRPin) == HIGH;
      bool midWhite = digitalRead(middleIRPin) == HIGH;
      bool rightWhite = digitalRead(rightIRPin) == HIGH;
      if (leftWhite && midWhite && rightWhite) {
        if (!timerStarted) {
          timerStarted = true;
          lostSince = millis();
        }
        return (millis() - lostSince) > LOST_LINE_TIMEOUT_MS;
      }
      timerStarted = false;
      return false;
    }
    void action() override {
      stopMotors();
    }
    String name() override { return "Lost Line"; }
  private:
    bool timerStarted = false;
    unsigned long lostSince = 0;
};

// ==========================================
// 3. GLOBAL OBJECTS
// ==========================================
Cruise behaviourCruise;
TurnLeft behaviourLeft;
TurnRight behaviourRight;
LostLine behaviourLost;

// ==========================================
// 4. SETUP
// ==========================================
void setup() {
  Serial.begin(9600);
  pinMode(leftIRPin, INPUT);
  pinMode(middleIRPin, INPUT);
  pinMode(rightIRPin, INPUT);

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  stopMotors();
}

// ==========================================
// 5. MAIN LOOP: BEHAVIOUR ARBITRATOR
// ==========================================
void loop() {
  Behaviour* active = nullptr;

  // Priority order: lost line > right > left > cruise
  if (behaviourLost.takeControl()) {
    active = &behaviourLost;
  } else if (behaviourRight.takeControl()) {
    active = &behaviourRight;
  } else if (behaviourLeft.takeControl()) {
    active = &behaviourLeft;
  } else {
    active = &behaviourCruise;
  }

  if (active != nullptr) {
    active->action();
  }

  delay(10);
}

