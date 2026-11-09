// ESP32 tilt and heading logger
//
// Two sensors, two I2C buses, two FreeRTOS tasks:
//   * MPU6050 (accelerometer and gyro) on I2C bus 0, read by the main loop (Arduino loopTask,
//     priority 2). Fused into roll and pitch with an extended Kalman filter (or the
//     complementary filter, see FUSION_EKF), and gyro z feeds the yaw filter.
//   * QMC5883L (magnetometer) on I2C bus 1, read by its own task (priority 1). The task
//     tilt-compensates each reading with the latest roll and pitch and publishes a heading.
// The main loop fuses gyro z with the newest heading into yaw and streams CSV over serial, for
// the live plotter and the Blender portal in host/. Register level, no sensor libraries.
//
// Targets (platformio.ini):
//   * ESP32-C6 (default): one CPU core, so the two tasks interleave on it. Bus 0 is the main
//     I2C controller; bus 1 is the low-power I2C controller, whose pins are fixed (GPIO 6, 7).
//   * ESP32 (esp32dev): two cores. loop() runs on core 1 and the magnetometer task on core 0,
//     so the sensors are read truly in parallel.
// Each bus has a single user, so no bus lock is needed.
#include <Arduino.h>
#include <Wire.h>

#include <cmath>

#include "imu_filter.h"

// ------------------------------------------------------------ configuration
constexpr uint8_t MPU_ADDR = 0x68;        // MPU6050 I2C address with AD0 pin low
constexpr uint8_t QMC_ADDR = 0x0D;        // QMC5883L I2C address (fixed)
constexpr int IMU_SDA_PIN = 21, IMU_SCL_PIN = 22;  // I2C bus 0: MPU6050 (both targets)
#if CONFIG_IDF_TARGET_ESP32C6
constexpr int MAG_SDA_PIN = 6, MAG_SCL_PIN = 7;    // I2C bus 1: QMC5883L on the LP I2C controller (fixed pins)
#else
constexpr int MAG_SDA_PIN = 25, MAG_SCL_PIN = 26;  // I2C bus 1: QMC5883L
#endif
constexpr uint32_t IMU_I2C_HZ = 400000;   // fast mode: the 14-byte read takes about 0.4 ms
constexpr uint32_t MAG_I2C_HZ = 100000;   // standard mode, supported by every controller; 6 bytes take about 0.8 ms
constexpr uint32_t SAMPLE_HZ = 100;
constexpr uint32_t MAG_TASK_HZ = 100;     // magnetometer task rate (the sensor outputs at 200 Hz)
constexpr BaseType_t MAG_TASK_CORE = 0;   // the only core on the C6; the other core from loop() on the ESP32
constexpr UBaseType_t LOOP_PRIORITY = 2;  // IMU sampling preempts the magnetometer task
constexpr UBaseType_t MAG_TASK_PRIORITY = 1;
constexpr uint32_t MAG_TASK_STACK = 4096; // bytes
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

TwoWire& imuBus = Wire;   // bus 0, used only by loop()
TwoWire& magBus = Wire1;  // bus 1, used only by the magnetometer task (after setup)

#if FUSION_EKF
imu::AttitudeEKF filter(1.0f / SAMPLE_HZ);
#else
imu::ComplementaryFilter filter(FILTER_TAU_S, 1.0f / SAMPLE_HZ);
#endif
imu::YawFilter yawFilter(YAW_TAU_S, 1.0f / SAMPLE_HZ, HEADING_SIGN);
imu::Vec3 gyroBias{0, 0, 0};
imu::Vec3 magOffset{0, 0, 0};  // hard-iron offset in raw counts, from calibrateMag()
bool magOk = false;

// ---------------------------------------------------- state shared between cores
// The only data the two tasks exchange. Every access holds sharedLock, a spinlock that is safe
// across cores and between tasks on one core. Critical sections are a few loads and stores.
struct SharedState {
  float roll = 0.0f, pitch = 0.0f;  // written by loop(), read by the magnetometer task
  float heading = 0.0f;             // written by the magnetometer task, read by loop()
  uint32_t headingSeq = 0;          // incremented with each new heading
  uint32_t magReadErrors = 0;       // incremented by the task on a failed read
};
SharedState shared;
portMUX_TYPE sharedLock = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t magTaskHandle = nullptr;

// ------------------------------------------------------------- I2C helpers
void writeRegister(TwoWire& bus, uint8_t addr, uint8_t reg, uint8_t value) {
  bus.beginTransmission(addr);
  bus.write(reg);
  bus.write(value);
  bus.endTransmission();
}

uint8_t readRegister(TwoWire& bus, uint8_t addr, uint8_t reg) {
  bus.beginTransmission(addr);
  bus.write(reg);
  bus.endTransmission(false);  // repeated start: keep the bus for the read
  bus.requestFrom(addr, (uint8_t)1);
  return bus.read();
}

// i-th 16-bit value of a burst read (each value is two bytes, high byte first)
float word(const uint8_t* buf, int i) { return imu::be16(buf[2 * i], buf[2 * i + 1]); }

// One 14-byte burst read, so every axis comes from the same sample instant.
bool readImu(imu::Vec3& accel_g, imu::Vec3& gyro_dps) {
  imuBus.beginTransmission(MPU_ADDR);
  imuBus.write(REG_ACCEL_XOUT_H);
  if (imuBus.endTransmission(false) != 0) return false;
  if (imuBus.requestFrom(MPU_ADDR, (uint8_t)14) != 14) return false;

  uint8_t buf[14];
  for (uint8_t& b : buf) b = imuBus.read();  // buffer first: evaluation order of two
                                             // read() calls in one expression is unspecified
  accel_g = {word(buf, 0) / ACCEL_LSB_PER_G, word(buf, 1) / ACCEL_LSB_PER_G, word(buf, 2) / ACCEL_LSB_PER_G};
  gyro_dps = {word(buf, 4) / GYRO_LSB_PER_DPS, word(buf, 5) / GYRO_LSB_PER_DPS, word(buf, 6) / GYRO_LSB_PER_DPS};
  return true;  // word 3 is the temperature, unused
}

// Six bytes from the QMC5883L: X, Y, Z, each little-endian (LSB first). Raw counts.
bool readMag(imu::Vec3& raw) {
  magBus.beginTransmission(QMC_ADDR);
  magBus.write(REG_QMC_DATA);
  if (magBus.endTransmission(false) != 0) return false;
  if (magBus.requestFrom(QMC_ADDR, (uint8_t)6) != 6) return false;

  uint8_t buf[6];
  for (uint8_t& b : buf) b = magBus.read();
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
// Runs in setup(), before the magnetometer task starts, so it has bus 1 to itself.
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

// ------------------------------------------------------------ magnetometer task
// Reads the QMC5883L at MAG_TASK_HZ, tilt-compensates with the latest roll and pitch from the
// main loop, and publishes the heading. It never touches bus 0 or Serial.
void magTask(void*) {
  const TickType_t period = pdMS_TO_TICKS(1000 / MAG_TASK_HZ);
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&lastWake, period);  // fixed rate, independent of how long the read took

    imu::Vec3 m;
    if (!readMag(m)) {
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

// -------------------------------------------------------------- main program
void setup() {
  Serial.begin(115200);
  vTaskPrioritySet(nullptr, LOOP_PRIORITY);  // setup() and loop() run in Arduino's loopTask
  imuBus.begin(IMU_SDA_PIN, IMU_SCL_PIN, IMU_I2C_HZ);
  const bool magBusOk = magBus.begin(MAG_SDA_PIN, MAG_SCL_PIN, MAG_I2C_HZ);

  if (readRegister(imuBus, MPU_ADDR, REG_WHO_AM_I) != 0x68) {
    Serial.printf("# ERROR: MPU6050 not found on bus 0 (SDA GPIO %d, SCL GPIO %d). Check wiring.\n", IMU_SDA_PIN, IMU_SCL_PIN);
    while (true) delay(1000);
  }
  writeRegister(imuBus, MPU_ADDR, REG_PWR_MGMT_1, 0x01);    // wake up, clock from gyro PLL (more stable than internal RC)
  writeRegister(imuBus, MPU_ADDR, REG_GYRO_CONFIG, 0x00);   // +/-250 deg/s
  writeRegister(imuBus, MPU_ADDR, REG_ACCEL_CONFIG, 0x00);  // +/-2 g

  calibrateGyro();

  magOk = magBusOk && readRegister(magBus, QMC_ADDR, REG_QMC_CHIP_ID) == QMC_CHIP_ID;
  if (magOk) {
    writeRegister(magBus, QMC_ADDR, REG_QMC_SET_RESET, 0x01);
    writeRegister(magBus, QMC_ADDR, REG_QMC_CONTROL1, QMC_CONTROL1_VALUE);
    calibrateMag();
    // From here on, only the task uses bus 1.
    if (xTaskCreatePinnedToCore(magTask, "mag", MAG_TASK_STACK, nullptr, MAG_TASK_PRIORITY, &magTaskHandle,
                                MAG_TASK_CORE) != pdPASS) {
      magOk = false;
      Serial.println("# WARN: could not start the magnetometer task. yaw_deg will be nan.");
    } else {
      Serial.printf("# magnetometer task running on core %d at %lu Hz\n", (int)MAG_TASK_CORE,
                    (unsigned long)MAG_TASK_HZ);
    }
  } else {
    Serial.printf("# WARN: QMC5883L not found on bus 1 (SDA GPIO %d, SCL GPIO %d). yaw_deg will be nan.\n",
                  MAG_SDA_PIN, MAG_SCL_PIN);
  }

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
  static uint32_t lastErrors = 0;
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

  // Report read failures from this core only (Serial is not shared with the task), at most once a
  // second, so a disconnected magnetometer cannot flood the CSV stream.
  static uint32_t lastErrorReportMs = 0;
  if (errors != lastErrors && millis() - lastErrorReportMs >= 1000) {
    Serial.printf("# WARN: magnetometer read failed (%lu total)\n", (unsigned long)errors);
    lastErrors = errors;
    lastErrorReportMs = millis();
  }

  Serial.printf("%lu,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\n", (unsigned long)millis(),
                a.x, a.y, a.z, g.x, g.y, g.z, filter.roll(), filter.pitch(), yaw);
}
