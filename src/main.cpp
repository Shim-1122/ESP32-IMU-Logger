// ESP32 attitude + heading logger
//
// ICM-20948 (accel, gyro, AK09916 mag) over I2C, register level, no sensor libs.
// EKF (or complementary, see FUSION_EKF) -> roll/pitch; tilt-compensated mag + gyro z -> yaw.
// CSV out over serial for host/ (live plot, Blender portal).
//
// Two FreeRTOS tasks, one bus (mutex):
//   loop()  prio 2, 100 Hz: accel/gyro, filters, only Serial writer
//   magTask prio 1, 200 Hz poll: mag sample -> heading
// Dual-core ESP32: tasks run in parallel. Single-core C6: interleaved, loop() preempts.
//
// Targets: esp32c6 (default), esp32dev -> ICM-20948. wokwi -> MPU6050 (no ICM part in Wokwi, yaw = nan).
#include <Arduino.h>
#include <Wire.h>

#include <cmath>

#include "imu_filter.h"

// ------------------------------------------------------------ configuration
constexpr int SDA_PIN = 21, SCL_PIN = 22;  // same on C6 and ESP32 DevKits
constexpr uint32_t I2C_HZ = 400000;       // 12-byte read ~0.35 ms
constexpr uint32_t SAMPLE_HZ = 100;
constexpr uint32_t MAG_TASK_HZ = 200;     // 2x mag output rate, no missed samples
constexpr BaseType_t MAG_TASK_CORE = 0;   // C6: only core; ESP32: other core from loop()
constexpr UBaseType_t LOOP_PRIORITY = 2;  // IMU preempts mag task
constexpr UBaseType_t MAG_TASK_PRIORITY = 1;
constexpr uint32_t MAG_TASK_STACK = 4096; // bytes
constexpr TickType_t BUS_TIMEOUT = pdMS_TO_TICKS(5);
constexpr float FILTER_TAU_S = 0.5f;      // complementary filter time constant
constexpr int CALIBRATION_SAMPLES = 200;  // 2 s, hold still
constexpr int MAG_CAL_SAMPLES = 1500;     // 15 s, rotate in every direction
constexpr float YAW_TAU_S = 10.0f;        // gyro short-term, mag long-term
// yaw: CCW-positive about +z (up), 0 = +x at magnetic north. -1 for CW compass heading.
constexpr float HEADING_SIGN = 1.0f;

// 1: EKF (bias learning, full kinematics, accel rejection). 0: complementary.
// Same CSV either way; analyze.py --compare reruns both offline.
#ifndef FUSION_EKF
#define FUSION_EKF 1
#endif

constexpr float ACCEL_LSB_PER_G = 16384.0f;  // +/-2 g
constexpr float GYRO_LSB_PER_DPS = 131.0f;   // +/-250 deg/s

#ifdef IMU_MPU6050
// ------------------------------------------- MPU6050 (RM-MPU-6000A)
constexpr uint8_t IMU_ADDRS[] = {0x68};
constexpr uint8_t IMU_WHO_AM_I_REG = 0x75, IMU_WHO_AM_I = 0x68;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;  // accel(6) temp(2) gyro(6)
constexpr uint8_t IMU_BURST_LEN = 14, GYRO_OFFSET = 8;
constexpr const char* IMU_NAME = "MPU6050";
#else
// ---------------------------------------------- ICM-20948 (DS-000189)
// 4 register banks, REG_BANK_SEL bits 5:4
constexpr uint8_t IMU_ADDRS[] = {0x69, 0x68};  // AD0 high (most boards) / low
constexpr uint8_t REG_BANK_SEL = 0x7F;
// bank 0
constexpr uint8_t IMU_WHO_AM_I_REG = 0x00, IMU_WHO_AM_I = 0xEA;
constexpr uint8_t REG_USER_CTRL = 0x03;     // bit 5: internal I2C master
constexpr uint8_t REG_PWR_MGMT_1 = 0x06;    // bit 7 reset, 6 sleep, 2:0 clock
constexpr uint8_t REG_PWR_MGMT_2 = 0x07;    // 0 = all axes on
constexpr uint8_t REG_INT_PIN_CFG = 0x0F;   // bit 1: bypass, mag on host bus
constexpr uint8_t REG_ACCEL_XOUT_H = 0x2D;  // accel(6) gyro(6), big-endian
constexpr uint8_t IMU_BURST_LEN = 12, GYRO_OFFSET = 6;
// bank 2
constexpr uint8_t REG_GYRO_SMPLRT_DIV = 0x00;   // rate = 1125 / (1 + div)
constexpr uint8_t REG_GYRO_CONFIG_1 = 0x01;     // 5:3 DLPF, 2:1 range, 0 DLPF on
constexpr uint8_t REG_ACCEL_SMPLRT_DIV_1 = 0x10, REG_ACCEL_SMPLRT_DIV_2 = 0x11;
constexpr uint8_t REG_ACCEL_CONFIG = 0x14;      // same layout as GYRO_CONFIG_1
constexpr uint8_t SMPLRT_DIV = 4;               // 225 Hz (> 2x sample rate)
constexpr uint8_t CONFIG_DLPF3_RANGE0 = (3 << 3) | (0 << 1) | 1;  // ~50 Hz LPF, min range
constexpr const char* IMU_NAME = "ICM-20948";
// ---------------------------------------- AK09916 mag (inside ICM-20948)
constexpr uint8_t MAG_ADDR = 0x0C;
constexpr uint8_t REG_MAG_WIA2 = 0x01, MAG_WIA2 = 0x09;  // device ID
constexpr uint8_t REG_MAG_ST1 = 0x10;   // ST1, HXL..HZH (LE), TMPS, ST2
constexpr uint8_t REG_MAG_CNTL2 = 0x31; // mode
constexpr uint8_t REG_MAG_CNTL3 = 0x32; // bit 0 soft reset
constexpr uint8_t MAG_MODE_CONT_100HZ = 0x08;
constexpr uint8_t ST1_DRDY = 0x01, ST2_HOFL = 0x08;     // data ready; overflow
#endif

uint8_t imuAddr = 0;

#if FUSION_EKF
imu::AttitudeEKF filter(1.0f / SAMPLE_HZ);
#else
imu::ComplementaryFilter filter(FILTER_TAU_S, 1.0f / SAMPLE_HZ);
#endif
imu::YawFilter yawFilter(YAW_TAU_S, 1.0f / SAMPLE_HZ, HEADING_SIGN);
imu::Vec3 gyroBias{0, 0, 0};
imu::Vec3 magOffset{0, 0, 0};  // hard-iron, raw counts
bool magOk = false;

// --------------------------------------------- shared between tasks (sharedLock)
struct SharedState {
  float roll = 0.0f, pitch = 0.0f;  // loop() -> magTask
  float heading = 0.0f;             // magTask -> loop()
  uint32_t headingSeq = 0;          // bumps on each new heading
  uint32_t magReadErrors = 0;
};
SharedState shared;
portMUX_TYPE sharedLock = portMUX_INITIALIZER_UNLOCKED;
SemaphoreHandle_t busMutex = nullptr;  // one I2C transfer at a time
TaskHandle_t magTaskHandle = nullptr;

// ------------------------------------------------------------- I2C helpers
// hold busMutex once magTask is running
void writeRegister(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);  // repeated start
  Wire.requestFrom(addr, (uint8_t)1);
  return Wire.read();
}

bool readBurst(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();  // buffer first (read order)
  return true;
}

// i-th big-endian int16
float word(const uint8_t* buf, int i) { return imu::be16(buf[2 * i], buf[2 * i + 1]); }

// one burst = all axes from the same instant
bool readImu(imu::Vec3& accel_g, imu::Vec3& gyro_dps) {
  uint8_t buf[IMU_BURST_LEN];
  if (xSemaphoreTake(busMutex, BUS_TIMEOUT) != pdTRUE) return false;
  const bool ok = readBurst(imuAddr, REG_ACCEL_XOUT_H, buf, IMU_BURST_LEN);
  xSemaphoreGive(busMutex);
  if (!ok) return false;
  const int g = GYRO_OFFSET / 2;
  accel_g = {word(buf, 0) / ACCEL_LSB_PER_G, word(buf, 1) / ACCEL_LSB_PER_G, word(buf, 2) / ACCEL_LSB_PER_G};
  gyro_dps = {word(buf, g) / GYRO_LSB_PER_DPS, word(buf, g + 1) / GYRO_LSB_PER_DPS, word(buf, g + 2) / GYRO_LSB_PER_DPS};
  return true;
}

#ifndef IMU_MPU6050
void selectBank(uint8_t bank) { writeRegister(imuAddr, REG_BANK_SEL, bank << 4); }

enum class MagRead { Ok, NotReady, Failed };

// raw counts (0.15 uT) in accel axes, no offset. NotReady = no new sample (normal at 200 Hz poll).
// Failed = bus error or overflow (magnet too close). Reading through ST2 releases the sample.
MagRead readMag(imu::Vec3& raw) {
  uint8_t b[9];
  if (xSemaphoreTake(busMutex, BUS_TIMEOUT) != pdTRUE) return MagRead::Failed;
  const bool ok = readBurst(MAG_ADDR, REG_MAG_ST1, b, sizeof b);
  xSemaphoreGive(busMutex);
  if (!ok || (b[8] & ST2_HOFL)) return MagRead::Failed;
  if (!(b[0] & ST1_DRDY)) return MagRead::NotReady;
  // little-endian: swap bytes for be16()
  const float mx = imu::be16(b[2], b[1]), my = imu::be16(b[4], b[3]), mz = imu::be16(b[6], b[5]);
  raw = imu::ak09916ToImuFrame({mx, my, mz});  // flip y, z to accel frame
  return MagRead::Ok;
}

bool initImu() {
  for (uint8_t addr : IMU_ADDRS) {
    writeRegister(addr, REG_BANK_SEL, 0);
    if (readRegister(addr, IMU_WHO_AM_I_REG) == IMU_WHO_AM_I) {
      imuAddr = addr;
      break;
    }
  }
  if (imuAddr == 0) return false;
  writeRegister(imuAddr, REG_PWR_MGMT_1, 0x80);  // reset (-> bank 0)
  delay(100);
  writeRegister(imuAddr, REG_PWR_MGMT_1, 0x01);  // wake, auto clock
  delay(20);
  writeRegister(imuAddr, REG_PWR_MGMT_2, 0x00);  // all axes on
  writeRegister(imuAddr, REG_USER_CTRL, 0x00);   // I2C master off
  writeRegister(imuAddr, REG_INT_PIN_CFG, 0x02); // bypass: mag at 0x0C

  selectBank(2);
  writeRegister(imuAddr, REG_GYRO_SMPLRT_DIV, SMPLRT_DIV);
  writeRegister(imuAddr, REG_GYRO_CONFIG_1, CONFIG_DLPF3_RANGE0);
  writeRegister(imuAddr, REG_ACCEL_SMPLRT_DIV_1, 0);
  writeRegister(imuAddr, REG_ACCEL_SMPLRT_DIV_2, SMPLRT_DIV);
  writeRegister(imuAddr, REG_ACCEL_CONFIG, CONFIG_DLPF3_RANGE0);
  selectBank(0);  // all runtime reads are bank 0
  delay(20);
  return true;
}

bool initMag() {
  if (readRegister(MAG_ADDR, REG_MAG_WIA2) != MAG_WIA2) return false;
  writeRegister(MAG_ADDR, REG_MAG_CNTL3, 0x01);  // soft reset
  delay(10);
  writeRegister(MAG_ADDR, REG_MAG_CNTL2, MAG_MODE_CONT_100HZ);
  delay(20);
  return true;
}
#else
bool initImu() {
  if (readRegister(0x68, IMU_WHO_AM_I_REG) != IMU_WHO_AM_I) return false;
  imuAddr = 0x68;
  writeRegister(imuAddr, 0x6B, 0x01);  // PWR_MGMT_1: wake, gyro PLL
  writeRegister(imuAddr, 0x1B, 0x00);  // +/-250 deg/s
  writeRegister(imuAddr, 0x1C, 0x00);  // +/-2 g
  return true;
}
#endif

// mean while still = bias
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

#ifndef IMU_MPU6050
// hard-iron offset = midpoint of min/max per axis over a full rotation (no soft-iron)
void calibrateMag() {
  Serial.println("# magnetometer: rotate the board slowly in every direction for 15 s");
  imu::Vec3 m, lo{32767, 32767, 32767}, hi{-32768, -32768, -32768};
  int n = 0;
  for (int i = 0; i < MAG_CAL_SAMPLES; i++) {
    if (readMag(m) == MagRead::Ok) {
      lo = {std::fmin(lo.x, m.x), std::fmin(lo.y, m.y), std::fmin(lo.z, m.z)};
      hi = {std::fmax(hi.x, m.x), std::fmax(hi.y, m.y), std::fmax(hi.z, m.z)};
      n++;
    }
    delay(1000 / SAMPLE_HZ);
  }
  if (n > 0) magOffset = {(hi.x + lo.x) / 2, (hi.y + lo.y) / 2, (hi.z + lo.z) / 2};
  Serial.printf("# mag offset (counts): %.1f %.1f %.1f from %d samples\n", magOffset.x, magOffset.y, magOffset.z, n);
}

// ------------------------------------------------------------ magnetometer task
// new sample -> tilt-compensate with latest roll/pitch -> publish heading. No Serial.
void magTask(void*) {
  const TickType_t period = pdMS_TO_TICKS(1000 / MAG_TASK_HZ);
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&lastWake, period);

    imu::Vec3 m;
    const MagRead r = readMag(m);
    if (r == MagRead::NotReady) continue;
    if (r == MagRead::Failed) {
      portENTER_CRITICAL(&sharedLock);
      shared.magReadErrors++;
      portEXIT_CRITICAL(&sharedLock);
      continue;
    }
    m = {m.x - magOffset.x, m.y - magOffset.y, m.z - magOffset.z};

    portENTER_CRITICAL(&sharedLock);
    const float roll = shared.roll, pitch = shared.pitch;
    portEXIT_CRITICAL(&sharedLock);

    const float heading = imu::magHeadingDeg(m, roll, pitch, HEADING_SIGN);

    portENTER_CRITICAL(&sharedLock);
    shared.heading = heading;
    shared.headingSeq++;
    portEXIT_CRITICAL(&sharedLock);
  }
}
#endif

// -------------------------------------------------------------- main program
void setup() {
  Serial.begin(115200);
  vTaskPrioritySet(nullptr, LOOP_PRIORITY);  // loopTask
  busMutex = xSemaphoreCreateMutex();
  Wire.begin(SDA_PIN, SCL_PIN, I2C_HZ);

  if (!initImu()) {
    Serial.printf("# ERROR: %s not found on I2C (SDA GPIO %d, SCL GPIO %d). Check wiring.\n", IMU_NAME, SDA_PIN,
                  SCL_PIN);
    while (true) delay(1000);
  }
  Serial.printf("# %s found at I2C address 0x%02X\n", IMU_NAME, imuAddr);

  calibrateGyro();

#ifndef IMU_MPU6050
  magOk = initMag();
  if (magOk) {
    calibrateMag();
    if (xTaskCreatePinnedToCore(magTask, "mag", MAG_TASK_STACK, nullptr, MAG_TASK_PRIORITY, &magTaskHandle,
                                MAG_TASK_CORE) != pdPASS) {
      magOk = false;
      Serial.println("# WARN: could not start the magnetometer task. yaw_deg will be nan.");
    } else {
      Serial.printf("# magnetometer task running at %lu Hz\n", (unsigned long)MAG_TASK_HZ);
    }
  } else {
    Serial.println("# WARN: AK09916 magnetometer not found (bypass mode). yaw_deg will be nan.");
  }
#else
  Serial.println("# MPU6050 has no magnetometer: yaw_deg will be nan.");
#endif

  Serial.println(FUSION_EKF ? "# fusion: extended Kalman filter" : "# fusion: complementary filter");
  Serial.println("# t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,roll_deg,pitch_deg,yaw_deg");
}

void loop() {
  // fixed 100 Hz; sleep (not spin) so magTask runs on the single-core C6
  static TickType_t lastWake = xTaskGetTickCount();
  vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000 / SAMPLE_HZ));

  imu::Vec3 a, g;
  if (!readImu(a, g)) {
    Serial.println("# WARN: I2C read failed");
    return;
  }
  g = {g.x - gyroBias.x, g.y - gyroBias.y, g.z - gyroBias.z};
  filter.update(a, g);

  // swap tilt for heading; stale heading -> gyro-only yaw step
  static uint32_t lastSeq = 0;
  portENTER_CRITICAL(&sharedLock);
  shared.roll = filter.roll();
  shared.pitch = filter.pitch();
  const float heading = shared.heading;
  const uint32_t seq = shared.headingSeq;
  const uint32_t errors = shared.magReadErrors;
  portEXIT_CRITICAL(&sharedLock);

  const bool magFresh = magOk && seq != lastSeq;
  lastSeq = seq;
  yawFilter.update(g.z, heading, magFresh);
  const float yaw = yawFilter.initialized() ? yawFilter.yaw() : NAN;

  // mag errors: report at most 1/s
  static uint32_t lastCheckMs = 0, errorsAtCheck = 0;
  if (millis() - lastCheckMs >= 1000) {
    if (errors != errorsAtCheck) {
      Serial.printf("# WARN: %lu magnetometer read failures in the last second\n",
                    (unsigned long)(errors - errorsAtCheck));
    }
    errorsAtCheck = errors;
    lastCheckMs = millis();
  }

  Serial.printf("%lu,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\n", (unsigned long)millis(),
                a.x, a.y, a.z, g.x, g.y, g.z, filter.roll(), filter.pitch(), yaw);
}
