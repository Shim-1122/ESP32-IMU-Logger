// ESP32 attitude and heading logger
//
// Sensor: TDK InvenSense ICM-20948, a 9-axis IMU (3-axis accelerometer, 3-axis gyroscope, and
// an AK09916 3-axis magnetometer in the same package), on one I2C bus, register level, no
// sensor libraries. Accelerometer and gyro are fused into roll and pitch with an extended
// Kalman filter (or the complementary filter, see FUSION_EKF). The magnetometer, tilt
// compensated with that roll and pitch, is fused with gyro z into yaw (heading). Samples are
// streamed as CSV over serial, for the live plotter and the Blender portal in host/.
//
// Two FreeRTOS tasks share the bus through a mutex:
//   * loop() (Arduino loopTask, priority 2) reads accel and gyro at 100 Hz, runs the attitude
//     and yaw filters, and is the only writer to Serial.
//   * magTask (priority 1) polls the AK09916 at 200 Hz, takes each new 100 Hz sample,
//     tilt-compensates it, and publishes a heading. On the dual-core ESP32 it runs on the other core, in parallel with loop(); on the
//     single-core ESP32-C6 the two interleave and loop() preempts it.
//
// Build targets (platformio.ini): esp32c6 (default) and esp32dev use the ICM-20948. The wokwi
// target defines IMU_MPU6050 to use the simulator's MPU6050 instead (accel and gyro only, so
// yaw is reported as nan), because Wokwi has no ICM-20948 part.
#include <Arduino.h>
#include <Wire.h>

#include <cmath>

#include "imu_filter.h"

// ------------------------------------------------------------ configuration
constexpr int SDA_PIN = 21, SCL_PIN = 22;  // same GPIOs on the ESP32-C6 DevKit and the ESP32 DevKit
constexpr uint32_t I2C_HZ = 400000;       // fast mode: the 12-byte accel/gyro read takes about 0.35 ms
constexpr uint32_t SAMPLE_HZ = 100;
constexpr uint32_t MAG_TASK_HZ = 200;     // twice the AK09916's 100 Hz output, so no sample is missed
constexpr BaseType_t MAG_TASK_CORE = 0;   // the only core on the C6; the other core from loop() on the ESP32
constexpr UBaseType_t LOOP_PRIORITY = 2;  // IMU sampling preempts the magnetometer task
constexpr UBaseType_t MAG_TASK_PRIORITY = 1;
constexpr uint32_t MAG_TASK_STACK = 4096; // bytes
constexpr TickType_t BUS_TIMEOUT = pdMS_TO_TICKS(5);
constexpr float FILTER_TAU_S = 0.5f;      // complementary filter crossover time constant
constexpr int CALIBRATION_SAMPLES = 200;  // 2 s at startup; keep the sensor still
constexpr int MAG_CAL_SAMPLES = 1500;     // 15 s at startup; rotate the board in every direction
constexpr float YAW_TAU_S = 10.0f;        // gyro heading trusted short-term; magnetometer pulls slowly
// Yaw follows the right-hand rule about the sensor's z axis (up, out of the chip's top face):
// it increases when the board turns counterclockwise seen from above, and 0 means the board's
// +x axis points at magnetic north. Set to -1.0f for a clockwise-positive compass heading.
constexpr float HEADING_SIGN = 1.0f;

// 1: extended Kalman filter (learns residual gyro bias, full 3-axis kinematics, rejects
//    linear acceleration). 0: complementary filter. The CSV format is the same either way,
//    and host/analyze.py --compare can rerun both filters offline on any recorded log.
#ifndef FUSION_EKF
#define FUSION_EKF 1
#endif

constexpr float ACCEL_LSB_PER_G = 16384.0f;  // at +/-2 g full scale (both sensors)
constexpr float GYRO_LSB_PER_DPS = 131.0f;   // at +/-250 deg/s full scale (both sensors)

#ifdef IMU_MPU6050
// ------------------------------------------- MPU6050 registers (datasheet RM-MPU-6000A)
constexpr uint8_t IMU_ADDRS[] = {0x68};
constexpr uint8_t IMU_WHO_AM_I_REG = 0x75, IMU_WHO_AM_I = 0x68;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;  // 14 bytes: accel(6) temp(2) gyro(6)
constexpr uint8_t IMU_BURST_LEN = 14, GYRO_OFFSET = 8;
constexpr const char* IMU_NAME = "MPU6050";
#else
// ---------------------------------------------- ICM-20948 registers (datasheet DS-000189)
// Registers are split into four banks selected by REG_BANK_SEL (bank number in bits 5:4).
constexpr uint8_t IMU_ADDRS[] = {0x69, 0x68};  // AD0 high (most breakout boards) or low
constexpr uint8_t REG_BANK_SEL = 0x7F;
// bank 0
constexpr uint8_t IMU_WHO_AM_I_REG = 0x00, IMU_WHO_AM_I = 0xEA;
constexpr uint8_t REG_USER_CTRL = 0x03;     // bit 5 I2C_MST_EN: 0 so the host owns the aux bus
constexpr uint8_t REG_PWR_MGMT_1 = 0x06;    // bit 7 reset, bit 6 sleep, bits 2:0 clock source
constexpr uint8_t REG_PWR_MGMT_2 = 0x07;    // 0: all accel and gyro axes on
constexpr uint8_t REG_INT_PIN_CFG = 0x0F;   // bit 1 BYPASS_EN: AK09916 appears on the host bus
constexpr uint8_t REG_ACCEL_XOUT_H = 0x2D;  // 12 bytes: accel(6) gyro(6), big-endian
constexpr uint8_t IMU_BURST_LEN = 12, GYRO_OFFSET = 6;
// bank 2
constexpr uint8_t REG_GYRO_SMPLRT_DIV = 0x00;   // output rate 1125 / (1 + div) Hz
constexpr uint8_t REG_GYRO_CONFIG_1 = 0x01;     // bits 5:3 DLPF, bits 2:1 range, bit 0 DLPF enable
constexpr uint8_t REG_ACCEL_SMPLRT_DIV_1 = 0x10, REG_ACCEL_SMPLRT_DIV_2 = 0x11;
constexpr uint8_t REG_ACCEL_CONFIG = 0x14;      // same layout as GYRO_CONFIG_1
constexpr uint8_t SMPLRT_DIV = 4;               // 225 Hz output, above twice the 100 Hz sample rate
constexpr uint8_t CONFIG_DLPF3_RANGE0 = (3 << 3) | (0 << 1) | 1;  // about 50 Hz low-pass; +/-250 deg/s or +/-2 g
constexpr const char* IMU_NAME = "ICM-20948";
// ---------------------------------------- AK09916 magnetometer (inside the ICM-20948)
constexpr uint8_t MAG_ADDR = 0x0C;
constexpr uint8_t REG_MAG_WIA2 = 0x01, MAG_WIA2 = 0x09;  // device ID
constexpr uint8_t REG_MAG_ST1 = 0x10;   // 9 bytes: ST1, HXL..HZH (little-endian), TMPS, ST2
constexpr uint8_t REG_MAG_CNTL2 = 0x31; // measurement mode
constexpr uint8_t REG_MAG_CNTL3 = 0x32; // bit 0 soft reset
constexpr uint8_t MAG_MODE_CONT_100HZ = 0x08;
constexpr uint8_t ST1_DRDY = 0x01, ST2_HOFL = 0x08;     // data ready; magnetic overflow
#endif

uint8_t imuAddr = 0;

#if FUSION_EKF
imu::AttitudeEKF filter(1.0f / SAMPLE_HZ);
#else
imu::ComplementaryFilter filter(FILTER_TAU_S, 1.0f / SAMPLE_HZ);
#endif
imu::YawFilter yawFilter(YAW_TAU_S, 1.0f / SAMPLE_HZ, HEADING_SIGN);
imu::Vec3 gyroBias{0, 0, 0};
imu::Vec3 magOffset{0, 0, 0};  // hard-iron offset in raw counts, from calibrateMag()
bool magOk = false;

// --------------------------------------------- state shared between the two tasks
// Every access holds sharedLock, a spinlock that is safe across cores and between tasks on one
// core. Critical sections are a few loads and stores. The I2C bus has its own mutex.
struct SharedState {
  float roll = 0.0f, pitch = 0.0f;  // written by loop(), read by the magnetometer task
  float heading = 0.0f;             // written by the magnetometer task, read by loop()
  uint32_t headingSeq = 0;          // incremented with each new heading
  uint32_t magReadErrors = 0;       // incremented by the task on a failed read
};
SharedState shared;
portMUX_TYPE sharedLock = portMUX_INITIALIZER_UNLOCKED;
SemaphoreHandle_t busMutex = nullptr;  // one I2C transaction at a time; priority inheritance
TaskHandle_t magTaskHandle = nullptr;

// ------------------------------------------------------------- I2C helpers
// Callers hold busMutex once the magnetometer task is running.
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

bool readBurst(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();  // buffer first: evaluation order of two
  return true;                                              // read() calls in one expression is unspecified
}

// i-th 16-bit value of a big-endian burst read
float word(const uint8_t* buf, int i) { return imu::be16(buf[2 * i], buf[2 * i + 1]); }

// One burst read, so every axis comes from the same sample instant.
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

// One AK09916 sample in the accelerometer's axes, raw counts (0.15 uT each), offsets not removed.
// NotReady: no new sample since the last read (normal when polling faster than 100 Hz).
// Failed: bus error, or the field overflowed the sensor's range (a magnet too close).
// Reading through ST2 tells the chip this sample has been taken.
MagRead readMag(imu::Vec3& raw) {
  uint8_t b[9];
  if (xSemaphoreTake(busMutex, BUS_TIMEOUT) != pdTRUE) return MagRead::Failed;
  const bool ok = readBurst(MAG_ADDR, REG_MAG_ST1, b, sizeof b);
  xSemaphoreGive(busMutex);
  if (!ok || (b[8] & ST2_HOFL)) return MagRead::Failed;
  if (!(b[0] & ST1_DRDY)) return MagRead::NotReady;
  // Little-endian (LSB first): be16() takes the high byte first, so swap each pair.
  const float mx = imu::be16(b[2], b[1]), my = imu::be16(b[4], b[3]), mz = imu::be16(b[6], b[5]);
  raw = imu::ak09916ToImuFrame({mx, my, mz});  // y and z flip into the accel/gyro axes
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
  writeRegister(imuAddr, REG_PWR_MGMT_1, 0x80);  // reset to defaults (also selects bank 0)
  delay(100);
  writeRegister(imuAddr, REG_PWR_MGMT_1, 0x01);  // wake up, best available clock (gyro PLL)
  delay(20);
  writeRegister(imuAddr, REG_PWR_MGMT_2, 0x00);  // accel and gyro on, all axes
  writeRegister(imuAddr, REG_USER_CTRL, 0x00);   // internal I2C master off
  writeRegister(imuAddr, REG_INT_PIN_CFG, 0x02); // bypass: AK09916 on the host bus at 0x0C

  selectBank(2);
  writeRegister(imuAddr, REG_GYRO_SMPLRT_DIV, SMPLRT_DIV);
  writeRegister(imuAddr, REG_GYRO_CONFIG_1, CONFIG_DLPF3_RANGE0);
  writeRegister(imuAddr, REG_ACCEL_SMPLRT_DIV_1, 0);
  writeRegister(imuAddr, REG_ACCEL_SMPLRT_DIV_2, SMPLRT_DIV);
  writeRegister(imuAddr, REG_ACCEL_CONFIG, CONFIG_DLPF3_RANGE0);
  selectBank(0);  // every register read from here on is in bank 0
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
  writeRegister(imuAddr, 0x6B, 0x01);  // PWR_MGMT_1: wake up, clock from gyro PLL
  writeRegister(imuAddr, 0x1B, 0x00);  // GYRO_CONFIG: +/-250 deg/s
  writeRegister(imuAddr, 0x1C, 0x00);  // ACCEL_CONFIG: +/-2 g
  return true;
}
#endif

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

#ifndef IMU_MPU6050
// Hard-iron calibration: the midpoint of each axis's min and max over a full rotation is the
// offset (the sensor's own bias, from nearby magnets and the board's metal). Soft-iron is not
// corrected. Rotate the board slowly through all orientations for the whole period.
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
// Polls the AK09916 at MAG_TASK_HZ, tilt-compensates each new sample with the latest roll and
// pitch from the main loop, and publishes the heading. It never touches Serial.
void magTask(void*) {
  const TickType_t period = pdMS_TO_TICKS(1000 / MAG_TASK_HZ);
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&lastWake, period);  // fixed rate, independent of how long the read took

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
  vTaskPrioritySet(nullptr, LOOP_PRIORITY);  // setup() and loop() run in Arduino's loopTask
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
  // Fixed-rate scheduling: vTaskDelayUntil wakes at absolute multiples of the period, so the rate
  // does not depend on execution time, and the task sleeps in between. Sleeping matters on the
  // single-core C6: a busy-wait would starve the magnetometer task and the idle task.
  static TickType_t lastWake = xTaskGetTickCount();
  vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000 / SAMPLE_HZ));

  imu::Vec3 a, g;
  if (!readImu(a, g)) {
    Serial.println("# WARN: I2C read failed");
    return;
  }
  g = {g.x - gyroBias.x, g.y - gyroBias.y, g.z - gyroBias.z};
  filter.update(a, g);

  // Hand the new tilt to the magnetometer task, and take its newest heading. A heading counts
  // as fresh only if the task published one since the last sample; otherwise the yaw filter
  // runs on the gyro alone for this step.
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

  // Report magnetometer failures (bus errors or field overflow) at most once a second, so a
  // problem is visible without flooding the CSV stream.
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
