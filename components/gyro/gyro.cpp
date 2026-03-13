#include "gyro.h"
#include "esp_log.h"
#include "esp_timer.h"   // esp_timer_get_time() → microseconds
#include "i2cInit.h"
#include "mutex.h"
#include <cmath>

static const char *TAG = "G&A sensor";

// ─────────────────────────────────────────────────────────────────────────────
// Complementary filter alpha:
//
//   gravity_new = α × (gyro_rotated_gravity) + (1−α) × accel_reading
//
// At 100 Hz (dt = 0.01 s) an α of 0.98 gives the gyro a time-constant of
// roughly 0.5 s before the accelerometer can pull it back.  That is long
// enough to ride through a jump (~0.3–0.5 s airborne) without the free-fall
// accel signal corrupting the gravity estimate, yet short enough to correct
// gyro drift over several seconds of normal use.
// ─────────────────────────────────────────────────────────────────────────────
static constexpr float DEFAULT_ALPHA = 0.98f;

// ── Constructor ───────────────────────────────────────────────────────────────
SensorReading::SensorReading()
    : _initialized(false), _calibrated(false),
      accel_sensitivity(16384.0f), gyro_sensitivity(131.0f),
      _grav_x(0.0f), _grav_y(0.0f), _grav_z(1.0f),
      _alpha(DEFAULT_ALPHA), _last_us(0) {
  data_queue = xQueueCreate(10, sizeof(mpu_data_t));
  init();
}

// ── Hardware init ─────────────────────────────────────────────────────────────
void SensorReading::init() {
  I2CManager &i2c = I2CManager::getInstance();
  if (!i2c.isInitialized()) {
    ESP_LOGE(TAG, "I2C Manager not initialized!");
    return;
  }

  ESP_LOGI(TAG, "Initializing MPU6050...");
  {
    MutexGuard lock(i2c.getMutex());

    // Wake the chip (clear SLEEP bit in PWR_MGMT_1)
    uint8_t wake[2] = {0x6B, 0x00};
    esp_err_t err = i2c_master_write_to_device(i2c.getPort(), MPU_ADDR, wake, 2,
                                               pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to wake MPU: %s", esp_err_to_name(err));
      return;
    }
    readSensitivity();
  }

  _initialized = true;
  ESP_LOGI(TAG, "MPU6050 initialized successfully");

  // Seed the gravity estimate while the device is presumably still at boot.
  calibrateGravity(50);
}

// ── Sensitivity registers ─────────────────────────────────────────────────────
void SensorReading::readSensitivity() {
  I2CManager &i2c = I2CManager::getInstance();
  uint8_t a_cfg, g_cfg;

  i2c_master_write_read_device(i2c.getPort(), MPU_ADDR, (uint8_t *)"\x1C", 1,
                               &a_cfg, 1, pdMS_TO_TICKS(100));
  i2c_master_write_read_device(i2c.getPort(), MPU_ADDR, (uint8_t *)"\x1B", 1,
                               &g_cfg, 1, pdMS_TO_TICKS(100));

  switch ((a_cfg >> 3) & 0x03) {
  case 0: accel_sensitivity = 16384.0f; break;
  case 1: accel_sensitivity =  8192.0f; break;
  case 2: accel_sensitivity =  4096.0f; break;
  case 3: accel_sensitivity =  2048.0f; break;
  }

  switch ((g_cfg >> 3) & 0x03) {
  case 0: gyro_sensitivity = 131.0f; break;
  case 1: gyro_sensitivity =  65.5f; break;
  case 2: gyro_sensitivity =  32.8f; break;
  case 3: gyro_sensitivity =  16.4f; break;
  }

  ESP_LOGI(TAG, "Accel sensitivity: %.1f LSB/g   Gyro sensitivity: %.1f LSB/(°/s)",
           accel_sensitivity, gyro_sensitivity);
}

// ── Raw sensor reads ──────────────────────────────────────────────────────────
esp_err_t SensorReading::readRawAccel(int16_t &ax, int16_t &ay, int16_t &az) {
  if (!_initialized) return ESP_FAIL;
  I2CManager &i2c = I2CManager::getInstance();
  MutexGuard lock(i2c.getMutex());

  uint8_t reg = 0x3B;
  uint8_t raw[6];
  esp_err_t ret = i2c_master_write_read_device(i2c.getPort(), MPU_ADDR,
                                               &reg, 1, raw, 6,
                                               pdMS_TO_TICKS(100));
  if (ret != ESP_OK) return ret;

  ax = (int16_t)((raw[0] << 8) | raw[1]);
  ay = (int16_t)((raw[2] << 8) | raw[3]);
  az = (int16_t)((raw[4] << 8) | raw[5]);
  return ESP_OK;
}

esp_err_t SensorReading::readRawGyro(int16_t &gx, int16_t &gy, int16_t &gz) {
  if (!_initialized) return ESP_FAIL;
  I2CManager &i2c = I2CManager::getInstance();
  MutexGuard lock(i2c.getMutex());

  uint8_t reg = 0x43;
  uint8_t raw[6];
  esp_err_t ret = i2c_master_write_read_device(i2c.getPort(), MPU_ADDR,
                                               &reg, 1, raw, 6,
                                               pdMS_TO_TICKS(100));
  if (ret != ESP_OK) return ret;

  gx = (int16_t)((raw[0] << 8) | raw[1]);
  gy = (int16_t)((raw[2] << 8) | raw[3]);
  gz = (int16_t)((raw[4] << 8) | raw[5]);
  return ESP_OK;
}

// ── Gravity calibration ───────────────────────────────────────────────────────
// Averages N still accel readings to find the initial gravity direction.
// This seeds _grav_xyz so the complementary filter starts from a good position
// rather than the generic [0,0,1] default.
void SensorReading::calibrateGravity(int samples) {
  if (!_initialized) {
    ESP_LOGW(TAG, "Cannot calibrate: sensor not initialized");
    return;
  }

  ESP_LOGI(TAG, "Calibrating gravity vector (%d samples, device must be still)...",
           samples);

  double sx = 0.0, sy = 0.0, sz = 0.0;
  int valid = 0;

  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az;
    if (readRawAccel(ax, ay, az) == ESP_OK) {
      sx += ax / accel_sensitivity;
      sy += ay / accel_sensitivity;
      sz += az / accel_sensitivity;
      valid++;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  if (valid == 0) {
    ESP_LOGE(TAG, "Gravity calibration failed: no valid samples");
    return;
  }

  _grav_x = (float)(sx / valid);
  _grav_y = (float)(sy / valid);
  _grav_z = (float)(sz / valid);

  float mag = sqrtf(_grav_x*_grav_x + _grav_y*_grav_y + _grav_z*_grav_z);
  ESP_LOGI(TAG, "Gravity seed: [%.3f, %.3f, %.3f]  |g| = %.3f g",
           _grav_x, _grav_y, _grav_z, mag);

  if (mag < 0.8f || mag > 1.2f) {
    ESP_LOGW(TAG, "Gravity magnitude %.2f g is out of expected range — "
             "was the device moving during calibration?", mag);
  }

  // Reset the timer so the first loop tick computes a clean dt
  _last_us   = esp_timer_get_time();
  _calibrated = true;
}

// ── Complementary filter: gyro rotation step ──────────────────────────────────
//
// The device has rotated by (gx,gy,gz) rad/s over dt seconds.
// We apply that same rotation to the stored gravity vector so it tracks
// how "down" has moved relative to the sensor axes.
//
// Rodrigues small-angle formula for rotating vector v by ω × dt:
//
//   v' = v + (ω × v) × dt
//
// This is accurate to first order in |ω|·dt.  At 100 Hz and typical
// human-motion rates (<< 10 rad/s) the error is < 0.05 % per step.
// ─────────────────────────────────────────────────────────────────────────────
void SensorReading::rotateGravity(float gx_rps, float gy_rps, float gz_rps,
                                  float dt) {
  // Cross product  ω × g
  float cx = gy_rps * _grav_z - gz_rps * _grav_y;
  float cy = gz_rps * _grav_x - gx_rps * _grav_z;
  float cz = gx_rps * _grav_y - gy_rps * _grav_x;

  // Integrate: g' = g + (ω × g) · dt
  _grav_x += cx * dt;
  _grav_y += cy * dt;
  _grav_z += cz * dt;
}

// ── Complementary filter: full update ────────────────────────────────────────
//
// Each tick we:
//   1. Propagate the gravity estimate through the gyro rotation  (gyro step)
//   2. Blend that prediction with the raw accel reading           (accel correction)
//   3. Re-normalise to keep |grav| = 1 g                         (normalise)
//
// The blend weight α controls the trade-off:
//   - Gyro step alone is smooth but drifts (integrates bias over time).
//   - Accel reading alone is accurate long-term but noisy and wrong mid-jump.
//   - α = 0.98 keeps the gyro dominant for ~0.5 s, long enough to span a jump.
//
// We expose the final gravity vector in the output struct so callers can
// visualise or log it for tuning.
// ─────────────────────────────────────────────────────────────────────────────
float SensorReading::computeVerticalAccel(float ax_g, float ay_g,
                                          float az_g) const {
  float mag = sqrtf(_grav_x*_grav_x + _grav_y*_grav_y + _grav_z*_grav_z);
  if (mag < 1e-6f) return 0.0f;

  // Unit vector pointing in the gravity direction (sensor frame)
  float ux = _grav_x / mag;
  float uy = _grav_y / mag;
  float uz = _grav_z / mag;

  // Scalar projection of the current accel onto the gravity axis
  float proj = ax_g * ux + ay_g * uy + az_g * uz;

  // Subtract the static 1 g contribution → 0 at rest,
  // positive upward, negative downward
  return proj - mag;
}

// ── Task plumbing ─────────────────────────────────────────────────────────────
void SensorReading::taskEntry(void *param) {
  static_cast<SensorReading *>(param)->taskLoop();
}

void SensorReading::startTask() {
  xTaskCreate(taskEntry, "mpu_reader", 4096, this, 5, nullptr);
}

// ── Main sensor loop ──────────────────────────────────────────────────────────
void SensorReading::taskLoop() {
  // Seed the timer on first entry (calibrateGravity may have set it already)
  if (_last_us == 0) _last_us = esp_timer_get_time();

  while (true) {
    // ── 1. Compute real dt ────────────────────────────────────────────────
    int64_t now_us = esp_timer_get_time();
    float dt = (now_us - _last_us) * 1e-6f;   // convert µs → seconds
    _last_us = now_us;

    // Guard against spurious large dt (e.g. first tick after a stall)
    if (dt <= 0.0f || dt > 0.5f) dt = 0.01f;

    // ── 2. Read sensors ───────────────────────────────────────────────────
    int16_t ax16, ay16, az16, gx16, gy16, gz16;
    if (readRawAccel(ax16, ay16, az16) != ESP_OK ||
        readRawGyro (gx16, gy16, gz16) != ESP_OK) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // Convert to physical units
    float ax_g  = ax16 / accel_sensitivity;   // g
    float ay_g  = ay16 / accel_sensitivity;
    float az_g  = az16 / accel_sensitivity;

    float gx_dps = gx16 / gyro_sensitivity;   // degrees / second
    float gy_dps = gy16 / gyro_sensitivity;
    float gz_dps = gz16 / gyro_sensitivity;

    // ── 3. Complementary filter ───────────────────────────────────────────
    if (_calibrated) {
      // Convert gyro to rad/s for the rotation step
      constexpr float DEG2RAD = 0.017453293f;
      float gx_rps = gx_dps * DEG2RAD;
      float gy_rps = gy_dps * DEG2RAD;
      float gz_rps = gz_dps * DEG2RAD;

      // Step A — Gyro prediction: rotate gravity estimate by ω × dt
      rotateGravity(gx_rps, gy_rps, gz_rps, dt);

      // Step B — Accel correction: blend gyro prediction with raw accel.
      // During free-fall/jump the accel magnitude is far from 1 g, so we
      // reduce the accel trust proportionally to its deviation from 1 g.
      // This prevents a mid-jump free-fall reading from pulling the gravity
      // estimate toward zero.
      float accel_mag = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
      float deviation  = fabsf(accel_mag - 1.0f);  // 0 at rest, ~1 in free-fall

      // Scale accel weight down when the device is clearly in motion.
      // deviation < 0.1 g  → full accel correction  (weight = 1−α)
      // deviation > 0.5 g  → nearly zero correction (trust gyro only)
      float accel_weight = (1.0f - _alpha) * fmaxf(0.0f,
                             1.0f - deviation / 0.5f);

      float gyro_weight  = 1.0f - accel_weight;

      _grav_x = gyro_weight * _grav_x + accel_weight * ax_g;
      _grav_y = gyro_weight * _grav_y + accel_weight * ay_g;
      _grav_z = gyro_weight * _grav_z + accel_weight * az_g;

      // Step C — Re-normalise to 1 g so magnitude never drifts
      float mag = sqrtf(_grav_x*_grav_x + _grav_y*_grav_y + _grav_z*_grav_z);
      if (mag > 1e-6f) {
        _grav_x /= mag;
        _grav_y /= mag;
        _grav_z /= mag;
      }
    }

    // ── 4. Build output ───────────────────────────────────────────────────
    mpu_data_t data;
    data.gx_dps = gx_dps;
    data.gy_dps = gy_dps;
    data.gz_dps = gz_dps;
    data.grav_x = _grav_x;
    data.grav_y = _grav_y;
    data.grav_z = _grav_z;

    if (_calibrated) {
      data.vertical_accel_g = computeVerticalAccel(ax_g, ay_g, az_g);
    } else {
      // Pre-calibration fallback: assume device is flat, Z axis is up
      data.vertical_accel_g = az_g - 1.0f;
    }

    xQueueSend(data_queue, &data, 0);

    vTaskDelay(pdMS_TO_TICKS(10));  // 100 Hz loop
  }
}