#include <Wire.h>

#define IR_LEFT 2
#define IR_MID 3
#define IR_RIGHT 4

// Simple data-capture sketch used to log raw accel/gyro for offline analysis in Gyro.ipynb.
// The logs from this were used to tune the bias/LPF/deadband pipeline implemented in Gyro_denoised.cpp.
const uint8_t ICM_ADDR = 0x68; // WHO_AM_I = 0x98 on this board (ICM-20689)
unsigned long lastImuPrintMs = 0;
const unsigned long imuPrintIntervalMs = 500;

int16_t read16(uint8_t reg) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(ICM_ADDR, (uint8_t)2, (uint8_t)true);
  int16_t v = (Wire.read() << 8) | Wire.read();
  return v;
}

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

void setup() {
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_MID, INPUT);
  pinMode(IR_RIGHT, INPUT);

  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Wire.begin();
  Wire.setClock(400000);

  // Check WHO_AM_I (logged in the notebook to confirm 0x98)
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(0x75);
  Wire.endTransmission(false);
  Wire.requestFrom(ICM_ADDR, (uint8_t)1, (uint8_t)true);
  uint8_t whoami = Wire.available() ? Wire.read() : 0xFF;
  Serial.print("WHO_AM_I = 0x");
  Serial.println(whoami, HEX);

  // Wake and configure
  writeReg(0x6B, 0x01);   // PWR_MGMT_1: clock = gyro X, sleep off
  writeReg(0x1A, 0x03);   // CONFIG: DLPF
  writeReg(0x1B, 0x08);   // GYRO_CONFIG: ±500 dps
  writeReg(0x1C, 0x10);   // ACCEL_CONFIG: ±8g
  writeReg(0x1D, 0x03);   // ACCEL_CONFIG2: accel DLPF
  writeReg(0x19, 0x07);   // SMPLRT_DIV

  delay(100);
}

void loop() {
  // IR CSV
  const int irLeft = digitalRead(IR_LEFT);
  const int irMid = digitalRead(IR_MID);
  const int irRight = digitalRead(IR_RIGHT);
  //Serial.print(irLeft); Serial.print(",");
  //Serial.print(irMid);  Serial.print(",");
  //Serial.println(irRight);

  const unsigned long now = millis();
  if (now - lastImuPrintMs >= imuPrintIntervalMs) {
    // Read accel (0x3B), temp (0x41), gyro (0x43)
    int16_t ax = read16(0x3B);
    int16_t ay = read16(0x3D);
    int16_t az = read16(0x3F);
    int16_t gx = read16(0x43);
    int16_t gy = read16(0x45);
    int16_t gz = read16(0x47);

    // Convert to units
    const float accelScale = 8.0f / 32768.0f;     // g/LSB for ±8g
    const float gyroScale  = 500.0f / 32768.0f;   // dps/LSB for ±500 dps

    Serial.print("Acceleration X: "); Serial.print(ax * accelScale);
    Serial.print(", Y: "); Serial.print(ay * accelScale);
    Serial.print(", Z: "); Serial.print(az * accelScale);
    Serial.println(" g");

    Serial.print("Rotation X: "); Serial.print(gx * gyroScale);
    Serial.print(", Y: "); Serial.print(gy * gyroScale);
    Serial.print(", Z: "); Serial.print(gz * gyroScale);
    Serial.println(" dps");

    Serial.println("");
    lastImuPrintMs = now;
  }

  delay(5);
}