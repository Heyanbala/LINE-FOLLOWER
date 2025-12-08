#define IR_LEFT 2
#define IR_MID 3
#define IR_RIGHT 4

void setup() {
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_MID, INPUT);
  pinMode(IR_RIGHT, INPUT);

  Serial.begin(115200);
}

void loop() {
  int L = digitalRead(IR_LEFT);
  int M = digitalRead(IR_MID);
  int R = digitalRead(IR_RIGHT);

  // Send CSV format: L,M,R
  Serial.print(L);
  Serial.print(",");
  Serial.print(M);
  Serial.print(",");
  Serial.println(R);

  delay(5);  // high sampling rate
}
