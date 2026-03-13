#pragma once
#include "driver/i2c.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <cstdint>

struct mpu_data_t {
  // World-frame vertical acceleration (gravity removed, orientation-independent).
  // 0 at rest. Positive = accelerating upward. Negative = falling / downward push.
  float vertical_accel_g;

  // Current complementary-filter gravity estimate in the sensor frame (units: g).
  // Useful for debugging or feeding into a higher-level detector.
  float grav_x;
  float grav_y;
  float grav_z;

  // Raw gyro in degrees-per-second
  float gx_dps;
  float gy_dps;
  float gz_dps;
};

class SensorReading {
public:
  // Singleton
  static SensorReading &getInstance() {
    static SensorReading instance;
    return instance;
  }

  // ── Public API ────────────────────────────────────────────────────────────
  void startTask();
  QueueHandle_t getQueue() const { return data_queue; }

  esp_err_t readRawAccel(int16_t &ax, int16_t &ay, int16_t &az);
  esp_err_t readRawGyro (int16_t &gx, int16_t &gy, int16_t &gz);

  bool isInitialized() const { return _initialized; }
  bool isCalibrated()  const { return _calibrated;  }

  // Re-seed the gravity estimate from a still-phase average.
  // Device must be stationary for ~(samples × 10 ms).
  void calibrateGravity(int samples = 50);

  // Tune the complementary filter blend at runtime (default α = 0.98).
  //   Higher α → trust gyro more (smoother, drifts slowly over minutes).
  //   Lower  α → trust accel more (corrects drift faster, noisier during motion).
  void setAlpha(float alpha) { _alpha = alpha; }

  SensorReading();

private:
  static constexpr uint8_t MPU_ADDR = 0x68;
  SensorReading(const SensorReading &) = delete;
  SensorReading &operator=(const SensorReading &) = delete;

  // ── Sensor config ─────────────────────────────────────────────────────────
  bool  _initialized;
  bool  _calibrated;
  float accel_sensitivity;  // raw LSB per g
  float gyro_sensitivity;   // raw LSB per °/s

  // ── Complementary filter state ────────────────────────────────────────────
  // Running gravity estimate in the sensor frame (units: g).
  // Seeded by calibrateGravity(), then updated every tick by rotateGravity()
  // and blended with the raw accelerometer reading.
  float _grav_x;
  float _grav_y;
  float _grav_z;

  float   _alpha;    // gyro trust weight  (0 < α < 1)
  int64_t _last_us;  // timestamp of previous loop tick (microseconds)

  // ── Internal helpers ──────────────────────────────────────────────────────
  QueueHandle_t data_queue;

  void  init();
  void  readSensitivity();

  // Rotate the stored gravity estimate by (gx,gy,gz) [rad/s] over dt [s].
  // Uses the Rodrigues small-angle approximation — O(1) and accurate for
  // the short dt values produced by our 100 Hz loop.
  void  rotateGravity(float gx_rps, float gy_rps, float gz_rps, float dt);

  // Project the raw accel vector onto the current gravity estimate,
  // subtract the static 1 g offset, and return signed vertical acceleration.
  float computeVerticalAccel(float ax_g, float ay_g, float az_g) const;

  void  taskLoop();
  static void taskEntry(void *param);
};