// ESP32 tilt and heading logger
//
// Reads an MPU6050 (accelerometer and gyro) and a QMC5883L (magnetometer) on one I2C bus,
// register level and no sensor libraries. Accelerometer and gyro are fused into roll and
// pitch with an extended Kalman filter (or the complementary filter, see FUSION_EKF).
// The magnetometer and gyro z are fused into yaw (heading). Samples are streamed as CSV
// over serial, for the live plotter and the Blender portal in host/.
#include <Arduino.h>
#include <Wire.h>

#include <cmath>

#include "imu_filter.h"

// ------------------------------------------------------------ configuration
constexpr uint8_t MPU_ADDR = 0x68;        // MPU6050 I2C address with AD0 pin low
constexpr uint8_t QMC_ADDR = 0x0D;        // QMC5883L I2C address (fixed)
constexpr int SDA_PIN = 21, SCL_PIN = 22;
constexpr uint32_t SAMPLE_HZ = 100;
constexpr uint32_t PERIOD_US = 1000000UL / SAMPLE_HZ;
constexpr float FILTER_TAU_S = 0.5f;      // complementary filter crossover time constant
constexpr int CALIBRATION_SAMPLES = 200;  // 2 s at startup; keep the sensor still
constexpr int MAG_CAL_SAMPLES = 1500;     // 15 s at startup; rotate the board in every direction
constexpr float YAW_TAU_S = 10.0f;        // gyro heading trusted short-term; magnetometer pulls slowly
// Direction of positive heading. Both the magnetometer and the gyro use it. Turn the board
// clockwise seen from above: if yaw_deg falls, set this to -1.0f (see README).
constexpr float HEADING_SIGN = 1.0f;

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

// ------------------------------------------------ QMC5883L registers (datasheet)
constexpr uint8_t REG_QMC_DATA = 0x00;        // X LSB, X MSB, Y LSB, Y MSB, Z LSB, Z MSB (little-endian)
constexpr uint8_t REG_QMC_CONTROL1 = 0x09;    // mode, output rate, range, oversampling
constexpr uint8_t REG_QMC_SET_RESET = 0x0B;   // set/reset period: 0x01 is the recommended value
constexpr uint8_t REG_QMC_CHIP_ID = 0x0D;     // reads 0xFF on a QMC5883L
constexpr uint8_t QMC_CHIP_ID = 0xFF;
// Continuous mode, 200 Hz output, +/-8 G range, OSR 512: 0b0001'1101
constexpr uint8_t QMC_CONTROL1_VALUE = 0x1D;

#if FUSION_EKF
imu::AttitudeEKF filter(1.0f / SAMPLE_HZ);
#else
imu::ComplementaryFilter filter(FILTER_TAU_S, 1.0f / SAMPLE_HZ);
#endif
imu::YawFilter yawFilter(YAW_TAU_S, 1.0f / SAMPLE_HZ, HEADING_SIGN);
imu::Vec3 gyroBias{0, 0, 0};
imu::Vec3 magOffset{0, 0, 0};  // hard-iron offset in raw counts, from calibrateMag()
bool magOk = false;
uint32_t nextSampleUs = 0;

// ------------------------------------------------------------- I2C helpers
void writeRegister(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);  // repeated start: keep the bus for the read
  Wire.requestFrom(addr, (uint8_t)1);
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

// Six bytes from the QMC5883L: X, Y, Z, each little-endian (LSB first). Raw counts.
bool readMag(imu::Vec3& raw) {
  Wire.beginTransmission(QMC_ADDR);
  Wire.write(REG_QMC_DATA);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(QMC_ADDR, (uint8_t)6) != 6) return false;

  uint8_t buf[6];
  for (uint8_t& b : buf) b = Wire.read();
  // be16() takes high byte first, so swap each pair to read little-endian
  raw = {(float)imu::be16(buf[1], buf[0]), (float)imu::be16(buf[3], buf[2]), (float)imu::be16(buf[5], buf[4])};
  return true;
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

// Hard-iron calibration: the midpoint of each axis's min and max over a full rotation is the
// offset (the sensor's own bias, from nearby magnets and the board's metal). Soft-iron is not
// corrected. Rotate the board slowly through all orientations for the whole period.
void calibrateMag() {
  Serial.println("# magnetometer: rotate the board slowly in every direction for 15 s");
  imu::Vec3 m, lo{32767, 32767, 32767}, hi{-32768, -32768, -32768};
  for (int i = 0; i < MAG_CAL_SAMPLES; i++) {
    if (readMag(m)) {
      lo = {std::fmin(lo.x, m.x), std::fmin(lo.y, m.y), std::fmin(lo.z, m.z)};
      hi = {std::fmax(hi.x, m.x), std::fmax(hi.y, m.y), std::fmax(hi.z, m.z)};
    }
    delay(1000 / SAMPLE_HZ);
  }
  if (hi.x >= lo.x && hi.y >= lo.y && hi.z >= lo.z) {
    magOffset = {(hi.x + lo.x) / 2, (hi.y + lo.y) / 2, (hi.z + lo.z) / 2};
  }
  Serial.printf("# mag offset (counts): %.1f %.1f %.1f\n", magOffset.x, magOffset.y, magOffset.z);
}

// -------------------------------------------------------------- main program
void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);  // 400 kHz fast mode: the 14-byte read takes about 0.4 ms

  if (readRegister(MPU_ADDR, REG_WHO_AM_I) != 0x68) {
    Serial.println("# ERROR: MPU6050 not found. Check wiring and I2C address.");
    while (true) delay(1000);
  }
  writeRegister(MPU_ADDR, REG_PWR_MGMT_1, 0x01);    // wake up, clock from gyro PLL (more stable than internal RC)
  writeRegister(MPU_ADDR, REG_GYRO_CONFIG, 0x00);   // +/-250 deg/s
  writeRegister(MPU_ADDR, REG_ACCEL_CONFIG, 0x00);  // +/-2 g

  calibrateGyro();

  magOk = readRegister(QMC_ADDR, REG_QMC_CHIP_ID) == QMC_CHIP_ID;
  if (magOk) {
    writeRegister(QMC_ADDR, REG_QMC_SET_RESET, 0x01);
    writeRegister(QMC_ADDR, REG_QMC_CONTROL1, QMC_CONTROL1_VALUE);
    calibrateMag();
  } else {
    Serial.println("# WARN: QMC5883L not found. yaw_deg will be nan. Check wiring.");
  }

  Serial.println(FUSION_EKF ? "# fusion: extended Kalman filter" : "# fusion: complementary filter");
  Serial.println("# t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,roll_deg,pitch_deg,yaw_deg");
  nextSampleUs = micros();
}

void loop() {
  // Fixed-rate scheduling (not delay()): the period does not depend on execution time.
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

  // Heading: the magnetometer is read every sample and tilt-compensated with the current
  // roll and pitch. If it is missing or a read fails, yaw runs on the gyro alone.
  imu::Vec3 m;
  bool magValid = magOk && readMag(m);
  float heading = 0.0f;
  if (magValid) {
    m = {m.x - magOffset.x, m.y - magOffset.y, m.z - magOffset.z};
    heading = imu::magHeadingDeg(m, filter.roll(), filter.pitch(), HEADING_SIGN);
  }
  yawFilter.update(g.z, heading, magValid);
  const float yaw = yawFilter.initialized() ? yawFilter.yaw() : NAN;

  Serial.printf("%lu,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\n", (unsigned long)millis(),
                a.x, a.y, a.z, g.x, g.y, g.z, filter.roll(), filter.pitch(), yaw);
}
