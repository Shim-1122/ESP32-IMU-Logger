// Tilt estimation math for a 6-axis IMU (accelerometer + gyroscope).
//
// No Arduino dependencies: the same code is compiled for the ESP32 and for the
// host unit tests (tests/firmware/).
#pragma once
#include <cmath>
#include <cstdint>

namespace imu {

constexpr float RAD_TO_DEG_F = 57.29577951f;

struct Vec3 {
  float x, y, z;
};

// Combine two bytes from the sensor (high byte first) into a signed 16-bit reading.
inline int16_t be16(uint8_t high, uint8_t low) {
  return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
}

// Tilt from gravity: no drift, but noisy and corrupted by linear acceleration.
inline float accelRoll(const Vec3& a) { return std::atan2(a.y, a.z) * RAD_TO_DEG_F; }
inline float accelPitch(const Vec3& a) {
  return std::atan2(-a.x, std::sqrt(a.y * a.y + a.z * a.z)) * RAD_TO_DEG_F;
}

// Complementary filter: fuse integrated gyro rate (smooth, but drifts) with
// accelerometer tilt (no drift, but noisy):
//
//     angle = alpha * (angle + rate * dt) + (1 - alpha) * accel_angle
//     alpha = tau / (tau + dt)
//
// Content faster than tau comes from the gyro, slower content from the accelerometer.
// Gyro x/y are used directly as roll/pitch rates (small-angle approximation,
// accurate while the other axis is tilted less than about 30 deg).
class ComplementaryFilter {
 public:
  ComplementaryFilter(float tau_s, float dt_s) : alpha_(tau_s / (tau_s + dt_s)), dt_(dt_s) {}

  // accel in g, gyro in deg/s
  void update(const Vec3& accel_g, const Vec3& gyro_dps) {
    const float roll_acc = accelRoll(accel_g);
    const float pitch_acc = accelPitch(accel_g);
    if (!initialized_) {  // initialize from the accelerometer
      roll_ = roll_acc;
      pitch_ = pitch_acc;
      initialized_ = true;
      return;
    }
    roll_ = alpha_ * (roll_ + gyro_dps.x * dt_) + (1.0f - alpha_) * roll_acc;
    pitch_ = alpha_ * (pitch_ + gyro_dps.y * dt_) + (1.0f - alpha_) * pitch_acc;
  }

  float roll() const { return roll_; }
  float pitch() const { return pitch_; }
  float alpha() const { return alpha_; }

 private:
  float alpha_, dt_;
  float roll_ = 0.0f, pitch_ = 0.0f;
  bool initialized_ = false;
};

}  // namespace imu
