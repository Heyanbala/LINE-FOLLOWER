// --- SENSOR PINS ---
const int leftIRPin   = 2;
const int middleIRPin = 4;
const int rightIRPin  = 10;

// --- MOTOR PINS ---
const int enA = 6;   const int in1 = 11;  const int in2 = 9;
const int enB = 5;   const int in3 = 7;   const int in4 = 8;

// --- SETTINGS ---
const int SPEED_CRUISE = 100;
const int SPEED_TURN   = 120;

// ==========================================
// 0. HELPER FUNCTIONS (Must be before classes)
// ==========================================

// Helper function to move motors
// Speed: -255 to 255
void move(int speedL, int speedR) {
  // Motor A (Left)
  if (speedL > 0) { 
    digitalWrite(in1, HIGH); 
    digitalWrite(in2, LOW); 
    analogWrite(enA, speedL); 
  }
  else { 
    digitalWrite(in1, LOW); 
    digitalWrite(in2, HIGH); 
    analogWrite(enA, -speedL); 
  }
  
  // Motor B (Right)
  if (speedR > 0) { 
    digitalWrite(in3, HIGH); 
    digitalWrite(in4, LOW); 
    analogWrite(enB, speedR); 
  }
  else { 
    digitalWrite(in3, LOW); 
    digitalWrite(in4, HIGH); 
    analogWrite(enB, -speedR); 
  }
}

// ==========================================
// 1. THE BEHAVIOR INTERFACE
// ==========================================
class Behavior {
  public:
    virtual bool takeControl() = 0; // Returns true if this behavior wants to run
    virtual void action() = 0;      // The code to run
    virtual String name() = 0;      // For debugging
};

// ==========================================
// 2. DEFINE THE BEHAVIORS
// ==========================================

// --- Behavior: CRUISE (Lowest Priority) ---
// Always wants to run. Drives straight.
class Cruise : public Behavior {
  public:
    bool takeControl() override {
      return true; // Always true (background task)
    }
    void action() override {
      move(SPEED_CRUISE, SPEED_CRUISE); // Drive straight
    }
    String name() override { return "Cruise"; }
};

// --- Behavior: TURN LEFT (Medium Priority) ---
// Uses difference (Left - Middle) to decide
class TurnLeft : public Behavior {
  public:
    bool takeControl() override {
      int leftVal = digitalRead(leftIRPin);
      int middleVal = digitalRead(middleIRPin);
      
      // Calculate Difference (L - M)
      // If Left is Line(0) and Middle is Surface(1), result is -1.
      int diff = leftVal - middleVal;
      
      return (diff == -1); 
    }
    void action() override {
      move(-SPEED_TURN, SPEED_TURN); // Spin Left
    }
    String name() override { return "Turn Left"; }
};

// --- Behavior: TURN RIGHT (Medium Priority) ---
// Uses difference (Right - Middle) to decide
class TurnRight : public Behavior {
  public:
    bool takeControl() override {
      int rightVal = digitalRead(rightIRPin);
      int middleVal = digitalRead(middleIRPin);
      
      // Calculate Difference (R - M)
      // If Right is Line(0) and Middle is Surface(1), result is -1.
      int diff = rightVal - middleVal;
      
      return (diff == -1);
    }
    void action() override {
      move(SPEED_TURN, -SPEED_TURN); // Spin Right
    }
    String name() override { return "Turn Right"; }
};

// ==========================================
// 3. GLOBAL OBJECTS
// ==========================================
Cruise behaviorCruise;
TurnLeft behaviorLeft;
TurnRight behaviorRight;

void setup() {
  Serial.begin(9600);
  
  pinMode(leftIRPin, INPUT); pinMode(middleIRPin, INPUT); pinMode(rightIRPin, INPUT);
  pinMode(enA, OUTPUT); pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
}

// ==========================================
// 4. THE ARBITRATOR (The Logic Loop)
// ==========================================
void loop() {
  Behavior* activeBehavior = nullptr;

  // Check behaviors in order of PRIORITY (Highest to Lowest)
  
  // Priority 1: Right Turn
  if (behaviorRight.takeControl()) {
    activeBehavior = &behaviorRight;
  }
  // Priority 2: Left Turn (Else if ensures we don't do both)
  else if (behaviorLeft.takeControl()) {
    activeBehavior = &behaviorLeft;
  }
  // Priority 3: Cruise (Default)
  else {
    activeBehavior = &behaviorCruise;
  }

  // EXECUTE THE WINNER
  if (activeBehavior != nullptr) {
    // Serial.println(activeBehavior->name()); // Uncomment to debug
    activeBehavior->action();
  }
  
  delay(10); // Small delay for stability
}