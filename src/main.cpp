// ESP32 + MPU6050 tilt logger
//
// Reads the IMU at a fixed 100 Hz over I2C (register level, no sensor library),
// fuses accelerometer and gyro with an extended Kalman filter (or the simpler
// complementary filter, see FUSION_EKF below), and streams CSV
// over serial for the Python live plotter in host/.
#include <Arduino.h>
#include <Wire.h>

#include "imu_filter.h"

// ------------------------------------------------------------ configuration
constexpr uint8_t MPU_ADDR = 0x68;        // I2C address with AD0 pin low
constexpr int SDA_PIN = 21, SCL_PIN = 22;
constexpr uint32_t SAMPLE_HZ = 100;
constexpr uint32_t PERIOD_US = 1000000UL / SAMPLE_HZ;
constexpr float FILTER_TAU_S = 0.5f;      // complementary filter crossover time constant
constexpr int CALIBRATION_SAMPLES = 200;  // 2 s at startup; keep the sensor still

// 1: extended Kalman filter (learns residual gyro bias, full 3-axis kinematics, rejects
//    linear acceleration). 0: complementary filter. The CSV format is the same either way,
//    and host/analyze.py --compare can rerun both filters offline on any recorded log.
#ifndef FUSION_EKF
#define FUSION_EKF 1
#endif

// ------------------------------------------- MPU6050 registers (datasheet RM-MPU-6000A)
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;  // first of 14 data bytes: accel(6) temp(2) gyro(6)
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_WHO_AM_I = 0x75;
constexpr float ACCEL_LSB_PER_G = 16384.0f;  // at +/-2 g full scale
constexpr float GYRO_LSB_PER_DPS = 131.0f;   // at +/-250 deg/s full scale

#if FUSION_EKF
imu::AttitudeEKF filter(1.0f / SAMPLE_HZ);
#else
imu::ComplementaryFilter filter(FILTER_TAU_S, 1.0f / SAMPLE_HZ);
#endif
imu::Vec3 gyroBias{0, 0, 0};
uint32_t nextSampleUs = 0;

// ------------------------------------------------------------- I2C helpers
void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);  // repeated start: keep the bus for the read
  Wire.requestFrom(MPU_ADDR, (uint8_t)1);
  return Wire.read();
}

// i-th 16-bit value of a burst read (each value is two bytes, high byte first)
float word(const uint8_t* buf, int i) { return imu::be16(buf[2 * i], buf[2 * i + 1]); }

// One 14-byte burst read, so every axis comes from the same sample instant.
bool readImu(imu::Vec3& accel_g, imu::Vec3& gyro_dps) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)14) != 14) return false;

  uint8_t buf[14];
  for (uint8_t& b : buf) b = Wire.read();  // buffer first: evaluation order of two
                                           // Wire.read() calls in one expression is unspecified
  accel_g = {word(buf, 0) / ACCEL_LSB_PER_G, word(buf, 1) / ACCEL_LSB_PER_G, word(buf, 2) / ACCEL_LSB_PER_G};
  gyro_dps = {word(buf, 4) / GYRO_LSB_PER_DPS, word(buf, 5) / GYRO_LSB_PER_DPS, word(buf, 6) / GYRO_LSB_PER_DPS};
  return true;  // word 3 is the temperature, unused
}

// Average the gyro while stationary; the mean is bias, which would otherwise integrate into drift.
void calibrateGyro() {
  imu::Vec3 a, g, sum{0, 0, 0};
  int n = 0;
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    if (readImu(a, g)) {
      sum = {sum.x + g.x, sum.y + g.y, sum.z + g.z};
      n++;
    }
    delay(1000 / SAMPLE_HZ);
  }
  if (n > 0) gyroBias = {sum.x / n, sum.y / n, sum.z / n};
  Serial.printf("# gyro bias (deg/s): %.3f %.3f %.3f\n", gyroBias.x, gyroBias.y, gyroBias.z);
}

// -------------------------------------------------------------- main program
void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);  // 400 kHz fast mode: the 14-byte read takes about 0.4 ms

  if (readRegister(REG_WHO_AM_I) != 0x68) {
    Serial.println("# ERROR: MPU6050 not found. Check wiring and I2C address.");
    while (true) delay(1000);
  }
  writeRegister(REG_PWR_MGMT_1, 0x01);    // wake up, clock from gyro PLL (more stable than internal RC)
  writeRegister(REG_GYRO_CONFIG, 0x00);   // +/-250 deg/s
  writeRegister(REG_ACCEL_CONFIG, 0x00);  // +/-2 g

  calibrateGyro();
  Serial.println(FUSION_EKF ? "# fusion: extended Kalman filter" : "# fusion: complementary filter");
  Serial.println("# t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,roll_deg,pitch_deg");
  nextSampleUs = micros();
}

void loop() {
  // Fixed-rate scheduling (not delay()): the period does not depend on loop execution time.
  // The signed difference handles micros() wraparound.
  if ((int32_t)(micros() - nextSampleUs) < 0) return;
  nextSampleUs += PERIOD_US;

  imu::Vec3 a, g;
  if (!readImu(a, g)) {
    Serial.println("# WARN: I2C read failed");
    return;
  }
  g = {g.x - gyroBias.x, g.y - gyroBias.y, g.z - gyroBias.z};
  filter.update(a, g);

  Serial.printf("%lu,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.2f,%.2f\n", (unsigned long)millis(),
                a.x, a.y, a.z, g.x, g.y, g.z, filter.roll(), filter.pitch());
}
