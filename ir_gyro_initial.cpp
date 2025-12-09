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


/*===================================================================
=====================================================================*/
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Set up sensor range (optional - currently set to default)
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("");
  delay(100);
}

void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.println("");
  delay(500);
}
/*-----------------------------------------------------------
    Important Concepts to Implement Before a Competition
Step 3: Check the Output
Connect your Arduino to your computer via USB.
Click the Upload button (Right Arrow icon).
Once uploaded, open the Serial Monitor (Top right magnifying glass icon).
Important: Make sure the baud rate in the bottom right corner of the Serial Monitor is set to 115200 baud.
What you should see:

If successful: You will see "MPU6050 Found!" followed by a stream of X, Y, and Z values updating every half second.
If you move the sensor, the numbers should change rapidly.
Troubleshooting
"Failed to find MPU6050 chip": Double-check your wiring. Ensure VCC is 5V, and SDA/SCL are connected to A4/A5 (on an Uno).
Gibberish text: Check that the Serial Monitor is set to 115200 baud.
-----------------------------------------------------------------*/