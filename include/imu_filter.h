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

// Extended Kalman filter for roll and pitch with online gyro-bias estimation.
// Mirrors AttitudeEKF in host/imu_math.py.
//
//   state x = [roll, pitch, bias_x, bias_y]   (rad, rad, rad/s, rad/s)
//   predict: roll_dot  = p + sin(roll) tan(pitch) q + cos(roll) tan(pitch) r
//            pitch_dot =     cos(roll) q          - sin(roll) r        (p, q bias-corrected)
//   update:  accelerometer = gravity in the body frame
//            h(x) = [-sin(pitch), sin(roll) cos(pitch), cos(roll) cos(pitch)]
//
// Measurement noise grows with | |a| - 1 g |, so the filter leans on the gyro
// during linear acceleration (a swinging limb) instead of following it.
class AttitudeEKF {
 public:
  explicit AttitudeEKF(float dt_s, float gyro_noise_dps = 0.3f, float bias_walk_dps = 0.01f,
                       float accel_noise_g = 0.05f, float accel_dyn_gain = 10.0f)
      : dt_(dt_s), r0_(accel_noise_g * accel_noise_g), dynGain_(accel_dyn_gain) {
    const float g = gyro_noise_dps / RAD_TO_DEG_F, b = bias_walk_dps / RAD_TO_DEG_F;
    q_[0] = q_[1] = g * g * dt_s * dt_s;
    q_[2] = q_[3] = b * b * dt_s;
    const float p0 = 5.0f / RAD_TO_DEG_F, b0 = 2.0f / RAD_TO_DEG_F;
    for (int i = 0; i < 4; i++)
      for (int j = 0; j < 4; j++) P_[i][j] = 0.0f;
    P_[0][0] = P_[1][1] = p0 * p0;
    P_[2][2] = P_[3][3] = b0 * b0;
  }

  // accel in g, gyro in deg/s
  void update(const Vec3& accel_g, const Vec3& gyro_dps) {
    if (!initialized_) {  // initialize from the accelerometer
      x_[0] = accelRoll(accel_g) / RAD_TO_DEG_F;
      x_[1] = accelPitch(accel_g) / RAD_TO_DEG_F;
      x_[2] = x_[3] = 0.0f;
      initialized_ = true;
      return;
    }
    predict(gyro_dps);
    correct(accel_g);
  }

  float roll() const { return x_[0] * RAD_TO_DEG_F; }
  float pitch() const { return x_[1] * RAD_TO_DEG_F; }
  float biasX() const { return x_[2] * RAD_TO_DEG_F; }  // deg/s
  float biasY() const { return x_[3] * RAD_TO_DEG_F; }

 private:
  void predict(const Vec3& gyro_dps) {
    const float p = gyro_dps.x / RAD_TO_DEG_F - x_[2];
    const float q = gyro_dps.y / RAD_TO_DEG_F - x_[3];
    const float r = gyro_dps.z / RAD_TO_DEG_F;
    const float sf = std::sin(x_[0]), cf = std::cos(x_[0]);
    const float tt = std::tan(x_[1]), ct = std::cos(x_[1]);

    x_[0] += dt_ * (p + sf * tt * q + cf * tt * r);
    x_[1] += dt_ * (cf * q - sf * r);

    // F = I + A dt
    float F[4][4] = {{1 + dt_ * (cf * tt * q - sf * tt * r), dt_ * (sf * q + cf * r) / (ct * ct), -dt_, -dt_ * sf * tt},
                     {dt_ * (-sf * q - cf * r), 1, 0, -dt_ * cf},
                     {0, 0, 1, 0},
                     {0, 0, 0, 1}};
    float FP[4][4], Pn[4][4];
    for (int i = 0; i < 4; i++)
      for (int j = 0; j < 4; j++) {
        FP[i][j] = 0;
        for (int k = 0; k < 4; k++) FP[i][j] += F[i][k] * P_[k][j];
      }
    for (int i = 0; i < 4; i++)
      for (int j = 0; j < 4; j++) {
        Pn[i][j] = 0;
        for (int k = 0; k < 4; k++) Pn[i][j] += FP[i][k] * F[j][k];
      }
    for (int i = 0; i < 4; i++)
      for (int j = 0; j < 4; j++) P_[i][j] = Pn[i][j] + (i == j ? q_[i] : 0.0f);
  }

  void correct(const Vec3& a) {
    const float sf = std::sin(x_[0]), cf = std::cos(x_[0]);
    const float st = std::sin(x_[1]), ct = std::cos(x_[1]);
    const float z[3] = {a.x, a.y, a.z};
    const float h[3] = {-st, sf * ct, cf * ct};
    const float H[3][2] = {{0, -ct}, {cf * ct, -sf * st}, {-sf * ct, -cf * st}};  // columns 2, 3 are zero

    const float dev = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z) - 1.0f;
    const float R = r0_ + dynGain_ * dev * dev;

    // PHt (4x3) = P H^T, using only the two nonzero columns of H
    float PHt[4][3];
    for (int i = 0; i < 4; i++)
      for (int j = 0; j < 3; j++) PHt[i][j] = P_[i][0] * H[j][0] + P_[i][1] * H[j][1];
    // S (3x3) = H P H^T + R
    float S[3][3];
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) S[i][j] = H[i][0] * PHt[0][j] + H[i][1] * PHt[1][j] + (i == j ? R : 0.0f);
    float Si[3][3];
    if (!invert3(S, Si)) return;  // degenerate: skip this correction

    float K[4][3];
    for (int i = 0; i < 4; i++)
      for (int j = 0; j < 3; j++) K[i][j] = PHt[i][0] * Si[0][j] + PHt[i][1] * Si[1][j] + PHt[i][2] * Si[2][j];

    float y[3];
    for (int i = 0; i < 3; i++) y[i] = z[i] - h[i];
    for (int i = 0; i < 4; i++) x_[i] += K[i][0] * y[0] + K[i][1] * y[1] + K[i][2] * y[2];

    // P = (I - K H) P, then symmetrize to fight float round-off
    float KH[4][4];
    for (int i = 0; i < 4; i++)
      for (int j = 0; j < 4; j++) KH[i][j] = (j < 2) ? K[i][0] * H[0][j] + K[i][1] * H[1][j] + K[i][2] * H[2][j] : 0.0f;
    float Pn[4][4];
    for (int i = 0; i < 4; i++)
      for (int j = 0; j < 4; j++) {
        Pn[i][j] = P_[i][j];
        for (int k = 0; k < 4; k++) Pn[i][j] -= KH[i][k] * P_[k][j];
      }
    for (int i = 0; i < 4; i++)
      for (int j = 0; j < 4; j++) P_[i][j] = 0.5f * (Pn[i][j] + Pn[j][i]);
  }

  static bool invert3(const float m[3][3], float out[3][3]) {
    const float det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                      m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                      m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    if (std::fabs(det) < 1e-20f) return false;
    const float d = 1.0f / det;
    out[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * d;
    out[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * d;
    out[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * d;
    out[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * d;
    out[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * d;
    out[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * d;
    out[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * d;
    out[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * d;
    out[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * d;
    return true;
  }

  float dt_, r0_, dynGain_;
  float q_[4];
  float x_[4] = {0, 0, 0, 0};
  float P_[4][4];
  bool initialized_ = false;
};

}  // namespace imu
