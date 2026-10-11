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

  if (failures == 0) std::printf("All firmware math tests passed\n");
  return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
