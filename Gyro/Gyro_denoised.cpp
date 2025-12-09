#include <Wire.h>
#include <Arduino.h>
#include <math.h>

// --- IMU address (ICM-20689 or compatible)
const uint8_t ICM_ADDR = 0x68;

// --- Gyro filtering parameters
float gx_bias = 0.0f, gy_bias = 0.0f, gz_bias = 0.0f;
float gx_lpf = 0.0f, gy_lpf = 0.0f, gz_lpf = 0.0f;
const float alpha = 0.15f;   // LPF smoothing
const float deadband = 0.3f; // dps

// --- Scales: ±8 g, ±500 dps
const float accelScale = 8.0f / 32768.0f;   // g/LSB
const float gyroScale  = 500.0f / 32768.0f; // dps/LSB

int16_t read16(uint8_t reg) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(ICM_ADDR, (uint8_t)2, (uint8_t)true);
  return (Wire.read() << 8) | Wire.read();
}

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

void calibrateGyroBias(uint16_t samples = 200) {
  gx_bias = gy_bias = gz_bias = 0.0f;
  for (uint16_t i = 0; i < samples; i++) {
    int16_t gx_raw = read16(0x43);
    int16_t gy_raw = read16(0x45);
    int16_t gz_raw = read16(0x47);
    gx_bias += gx_raw * gyroScale;
    gy_bias += gy_raw * gyroScale;
    gz_bias += gz_raw * gyroScale;
    delay(2);
  }
  gx_bias /= samples;
  gy_bias /= samples;
  gz_bias /= samples;
}

inline float lpfStep(float prev, float raw) {
  return prev + alpha * (raw - prev);
}

inline float applyDeadband(float v) {
  return (fabsf(v) < deadband) ? 0.0f : v;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin();
  Wire.setClock(400000);

  // WHO_AM_I
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(0x75);
  Wire.endTransmission(false);
  Wire.requestFrom(ICM_ADDR, (uint8_t)1, (uint8_t)true);
  uint8_t who = Wire.available() ? Wire.read() : 0xFF;
  Serial.print("WHO_AM_I = 0x");
  Serial.println(who, HEX);

  // Wake and configure
  writeReg(0x6B, 0x01); // PWR_MGMT_1: clock = gyro X
  writeReg(0x1A, 0x03); // CONFIG: DLPF
  writeReg(0x1B, 0x08); // GYRO_CONFIG: ±500 dps
  writeReg(0x1C, 0x10); // ACCEL_CONFIG: ±8g
  writeReg(0x1D, 0x03); // ACCEL_CONFIG2: DLPF
  writeReg(0x19, 0x07); // SMPLRT_DIV
  delay(100);

  Serial.println("Calibrating gyro bias... keep sensor still");
  calibrateGyroBias(200);
  Serial.print("Bias gx, gy, gz (dps): ");
  Serial.print(gx_bias, 4); Serial.print(", ");
  Serial.print(gy_bias, 4); Serial.print(", ");
  Serial.println(gz_bias, 4);
}

void loop() {
  // Read accel
  int16_t ax_raw = read16(0x3B);
  int16_t ay_raw = read16(0x3D);
  int16_t az_raw = read16(0x3F);
  float ax = ax_raw * accelScale;
  float ay = ay_raw * accelScale;
  float az = az_raw * accelScale;

  // Read gyro
  int16_t gx_r = read16(0x43);
  int16_t gy_r = read16(0x45);
  int16_t gz_r = read16(0x47);
  float gx = gx_r * gyroScale;
  float gy = gy_r * gyroScale;
  float gz = gz_r * gyroScale;

  // Bias remove
  float gx_b = gx - gx_bias;
  float gy_b = gy - gy_bias;
  float gz_b = gz - gz_bias;

  // LPF
  gx_lpf = lpfStep(gx_lpf, gx_b);
  gy_lpf = lpfStep(gy_lpf, gy_b);
  gz_lpf = lpfStep(gz_lpf, gz_b);

  // Deadband
  float gx_out = applyDeadband(gx_lpf);
  float gy_out = applyDeadband(gy_lpf);
  float gz_out = applyDeadband(gz_lpf);

  // Print filtered gyro and LPF accel (bias not removed for accel here)
  Serial.print("Gyro filt+db (dps): ");
  Serial.print(gx_out, 4); Serial.print(", ");
  Serial.print(gy_out, 4); Serial.print(", ");
  Serial.print(gz_out, 4); Serial.print(" | ");
  Serial.print("Accel (g): ");
  Serial.print(ax, 4); Serial.print(", ");
  Serial.print(ay, 4); Serial.print(", ");
  Serial.println(az, 4);

  delay(5);
}

