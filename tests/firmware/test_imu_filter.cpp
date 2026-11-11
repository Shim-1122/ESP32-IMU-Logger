// Host-side unit tests for include/imu_filter.h (the same header the ESP32 runs).
//   g++ -std=c++17 -Wall -Wextra -Iinclude tests/firmware/test_imu_filter.cpp -o test_fw && ./test_fw
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "imu_filter.h"

static int failures = 0;
#define CHECK_NEAR(actual, expected, tol)                                                   \
  do {                                                                                      \
    double a_ = (actual), e_ = (expected);                                                  \
    if (std::fabs(a_ - e_) > (tol)) {                                                       \
      std::printf("FAIL %s:%d  %s = %.6f, expected %.6f\n", __FILE__, __LINE__, #actual, a_, e_); \
      failures++;                                                                           \
    }                                                                                       \
  } while (0)

const double PI = 3.14159265358979;
imu::Vec3 gravityAtPitch(double deg) {  // what a still accelerometer reads at a given pitch
  double r = deg * PI / 180;
  return {(float)-std::sin(r), 0.0f, (float)std::cos(r)};
}

int main() {
  // Big-endian byte assembly, including negative values
  CHECK_NEAR(imu::be16(0x00, 0x64), 100, 0);
  CHECK_NEAR(imu::be16(0xFF, 0x38), -200, 0);
  CHECK_NEAR(imu::be16(0x80, 0x00), -32768, 0);

  // Accelerometer tilt
  CHECK_NEAR(imu::accelPitch(gravityAtPitch(30)), 30, 1e-4);
  CHECK_NEAR(imu::accelRoll({0.0f, 0.5f, 0.8660254f}), 30, 1e-4);

  // alpha = tau / (tau + dt)
  imu::ComplementaryFilter f(0.5f, 0.01f);
  CHECK_NEAR(f.alpha(), 0.5 / 0.51, 1e-6);

  // Starts from the accelerometer angle, not zero
  f.update(gravityAtPitch(20), {0, 0, 0});
  CHECK_NEAR(f.pitch(), 20, 1e-4);

  // Constant gyro bias b settles to an error of tau * b (not unbounded drift like pure integration)
  imu::ComplementaryFilter g(0.5f, 0.01f);
  for (int i = 0; i < 2000; i++) g.update(gravityAtPitch(0), {0, 2.0f, 0});
  CHECK_NEAR(g.pitch(), 0.5 * 2.0, 0.02);  // alpha*b*dt/(1-alpha) simplifies to tau*b

  // Tracks a rotation where gyro and accelerometer agree
  imu::ComplementaryFilter h(0.5f, 0.01f);
  for (int i = 0; i <= 300; i++) h.update(gravityAtPitch(0.1 * i), {0, 10.0f, 0});  // 10 deg/s for 3 s
  CHECK_NEAR(h.pitch(), 30, 0.05);


  // ---------------------------------------------------------------- EKF
  // Starts from the accelerometer angle
  imu::AttitudeEKF e(0.01f);
  e.update(gravityAtPitch(20), {0, 0, 0});
  CHECK_NEAR(e.pitch(), 20, 1e-4);

  // A constant gyro bias is learned, so the angle error goes to ~0 instead of tau * bias
  imu::AttitudeEKF eb(0.01f);
  for (int i = 0; i < 6000; i++) eb.update(gravityAtPitch(0), {0, 2.0f, 0});  // 60 s still
  CHECK_NEAR(eb.biasY(), 2.0, 0.05);
  CHECK_NEAR(eb.pitch(), 0.0, 0.05);

  // Tracks a rotation where gyro and accelerometer agree
  imu::AttitudeEKF er(0.01f);
  for (int i = 0; i <= 300; i++) er.update(gravityAtPitch(0.1 * i), {0, 10.0f, 0});
  CHECK_NEAR(er.pitch(), 30, 0.1);

  // Roll and pitch together (beyond the complementary filter's small-angle assumption)
  imu::AttitudeEKF e2(0.01f);
  const double rr = 40 * PI / 180, pp = 35 * PI / 180;
  const imu::Vec3 tilted{(float)-std::sin(pp), (float)(std::sin(rr) * std::cos(pp)), (float)(std::cos(rr) * std::cos(pp))};
  for (int i = 0; i < 500; i++) e2.update(tilted, {0, 0, 0});
  CHECK_NEAR(e2.roll(), 40, 0.05);
  CHECK_NEAR(e2.pitch(), 35, 0.05);

  // ------------------------------------------------------------ heading (yaw)
  CHECK_NEAR(imu::wrap180(190), -170, 1e-3);
  CHECK_NEAR(imu::wrap180(-190), 170, 1e-3);
  CHECK_NEAR(imu::wrap180(45), 45, 1e-3);

  // Level board: heading is the direction of the horizontal field
  CHECK_NEAR(imu::magHeadingDeg({1.0f, 0.0f, 0.0f}, 0, 0), 0, 1e-3);
  CHECK_NEAR(imu::magHeadingDeg({0.0f, -1.0f, 0.0f}, 0, 0), 90, 1e-3);
  CHECK_NEAR(std::fabs(imu::magHeadingDeg({-1.0f, 0.0f, 0.0f}, 0, 0)), 180, 1e-3);  // +/-180 is the same heading
  // The sign argument flips the direction
  CHECK_NEAR(imu::magHeadingDeg({0.0f, -1.0f, 0.0f}, 0, 0, -1.0f), -90, 1e-3);

  // Tilt compensation: pitch the board 30 deg. A field pointing at heading 0 (north) reads
  // as (cos 30, 0, sin 30) in the body frame, and the heading must still come out as 0.
  const double th = 30 * PI / 180;
  CHECK_NEAR(imu::magHeadingDeg({(float)std::cos(th), 0.0f, (float)std::sin(th)}, 0, 30), 0, 1e-3);
  // Roll the board 40 deg with a north field: the field is unchanged along the roll axis
  CHECK_NEAR(imu::magHeadingDeg({1.0f, 0.0f, 0.0f}, 40, 0), 0, 1e-3);

  // ICM-20948 magnetometer axes -> accelerometer axes: y and z flip, x unchanged
  {
    const imu::Vec3 m = imu::ak09916ToImuFrame({10.0f, 20.0f, 30.0f});
    CHECK_NEAR(m.x, 10, 0);
    CHECK_NEAR(m.y, -20, 0);
    CHECK_NEAR(m.z, -30, 0);
  }
  // Heading is counterclockwise-positive in the z-up frame: turn the board 90 deg CCW from
  // north and north ends up on its right (-y), which must read +90.
  CHECK_NEAR(imu::magHeadingDeg({0.0f, -1.0f, 0.0f}, 0, 0), 90, 1e-3);
  // Same turn seen by the raw AK09916 (its +y is the board's right): +y raw maps to -y.
  CHECK_NEAR(imu::magHeadingDeg(imu::ak09916ToImuFrame({0.0f, 1.0f, 0.0f}), 0, 0), 90, 1e-3);

  // ------------------------------------------------------------ yaw filter
  // Starts from the magnetometer, not zero
  imu::YawFilter y0(10.0f, 0.01f);
  y0.update(0.0f, 45.0f, true);
  CHECK_NEAR(y0.yaw(), 45, 1e-4);

  // Nothing is reported until a magnetometer reading arrives
  imu::YawFilter yn(10.0f, 0.01f);
  yn.update(5.0f, 0.0f, false);
  CHECK_NEAR(yn.initialized() ? 1.0 : 0.0, 0.0, 0);

  // Gyro turns the heading when the magnetometer agrees with the turn
  imu::YawFilter y1(10.0f, 0.01f);
  y1.update(0.0f, 0.0f, true);
  for (int i = 0; i < 300; i++) {  // 10 deg/s for 3 s; the field stays consistent
    y1.update(10.0f, imu::wrap180(10.0f * 0.01f * (i + 1)), true);
  }
  CHECK_NEAR(y1.yaw(), 30, 0.2);

  // Constant gyro bias with a fixed, correct magnetometer: the error settles at tau * bias
  imu::YawFilter y2(10.0f, 0.01f);
  y2.update(0.0f, 0.0f, true);
  for (int i = 0; i < 6000; i++) y2.update(1.0f, 0.0f, true);  // 60 s, bias 1 deg/s
  CHECK_NEAR(y2.yaw(), 10.0, 0.5);

  // Without magnetometer, the gyro alone drifts: after 60 s at 1 deg/s the heading is 60
  imu::YawFilter y3(10.0f, 0.01f);
  y3.update(0.0f, 0.0f, true);
  for (int i = 0; i < 6000; i++) y3.update(1.0f, 0.0f, false);
  CHECK_NEAR(y3.yaw(), 60, 0.5);

  // Crossing +/-180 takes the short way round: the heading stays near 180
  imu::YawFilter yw(10.0f, 0.01f);
  yw.update(0.0f, 179.0f, true);
  for (int i = 0; i < 100; i++) yw.update(0.0f, -179.0f, true);
  CHECK_NEAR(std::fabs(imu::wrap180(yw.yaw() - 180.0f)), 0.0, 2.0);

  if (failures == 0) std::printf("All firmware math tests passed\n");
  return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
