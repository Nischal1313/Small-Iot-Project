#include "jump.h"
#include "gyro.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <cmath>
#include <cstring>

static const char *TAG = "JumpDetector";

// ── Timing & filter constants ─────────────────────────────────────────────────
constexpr int   TIMING_TOLERANCE_MS    = 40;
constexpr int   MAX_PHASE_DURATION_MS  = 800;
constexpr float FILTER_ALPHA_FAST      = 0.4f;
constexpr float FILTER_ALPHA_SLOW      = 0.15f;
constexpr int   CALIBRATION_JUMPS      = 5;

// Peak detection
constexpr float PEAK_DROP_THRESHOLD    = 0.85f; // 15 % drop confirms peak
constexpr float MIN_PEAK_VALUE         = 0.15f; // minimum vertical_accel_g to care about (g)

// Adaptive threshold — now in physical units (g), not raw LSB
constexpr float MIN_THRESHOLD          = 0.15f; // g
constexpr float MAX_THRESHOLD          = 4.0f;  // g  (MPU6050 ±2 g range default)
constexpr float INITIAL_THRESHOLD      = 0.4f;  // g
constexpr float JUMP_THRESHOLD_FACTOR  = 1.3f;

constexpr int   MIN_JUMP_INTERVAL_MS   = 300;
constexpr int   TASK_LOOP_MS           = 10;    // matches gyro task rate

// Four timing configurations {rise ms, fall ms}
static const struct { uint32_t rise; uint32_t fall; }
TIMING_CONFIGS[NUM_TIMING_CONFIGS] = {
  {180, 180},
  {190, 190},
  {200, 200},
  {210, 210},
};

// ── Constructor ───────────────────────────────────────────────────────────────
JumpDetector::JumpDetector(SensorReading *sensor,
                           float    thresholdFactor,
                           uint32_t minIntervalMs)
    : _sensor(sensor),
      _thresholdFactor(thresholdFactor),
      _minIntervalMs(minIntervalMs),
      _avgJump(INITIAL_THRESHOLD),
      _calibrationComplete(false),
      _calibrationJumps(0)
{
  strcpy(_axisVertical.name, "Vert");

  for (int i = 0; i < NUM_TIMING_CONFIGS; i++) {
    JumpConfig &c = _axisVertical.configs[i];
    c.minRiseDuration  = TIMING_CONFIGS[i].rise;
    c.minFallDuration  = TIMING_CONFIGS[i].fall;
    c.jumpCount        = 0;
    c.lastJumpTime     = 0;
    c.state            = STATE_IDLE;
    c.peak             = 0.0f;
    c.valley           = 0.0f;
    c.risingStartTime  = 0;
    c.fallingStartTime = 0;
    c.lastValue        = 0.0f;
    c.filteredFast     = 0.0f;
    c.filteredSlow     = 0.0f;
  }
}

// ── Timing helper ─────────────────────────────────────────────────────────────
uint32_t JumpDetector::getMillis() {
  return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

// ── update() — call this from your task loop or FreeRTOS task ─────────────────
//
// Drains one item from the gyro queue.  The gyro task runs at 100 Hz and
// pushes mpu_data_t structs containing vertical_accel_g — the orientation-
// independent, gravity-compensated vertical acceleration produced by the
// complementary filter.  We use that single float; the raw axes are ignored.
// ─────────────────────────────────────────────────────────────────────────────
void JumpDetector::update() {
  if (!_sensor || !_sensor->isInitialized()) return;

  mpu_data_t data;
  // Non-blocking read from the queue (0 ticks wait)
  if (xQueueReceive(_sensor->getQueue(), &data, 0) != pdTRUE) return;

  uint32_t now = getMillis();

  // vertical_accel_g is already:
  //   • orientation-independent (gravity vector tracked by complementary filter)
  //   • gravity-subtracted      (0 at rest, +g when jumping up, −g when landing)
  //   • in physical units (g)   (not raw LSB)
  updateAxis(_axisVertical, data.vertical_accel_g, now);
}

// ── FreeRTOS task wrapper ─────────────────────────────────────────────────────
void JumpDetector::jumpDetectionTask() {
  while (true) {
    update();
    vTaskDelay(pdMS_TO_TICKS(TASK_LOOP_MS));
  }
}

// ── Axis update — fans out to all timing configs ──────────────────────────────
void JumpDetector::updateAxis(AxisDetector &axis, float value, uint32_t now) {
  for (int i = 0; i < NUM_TIMING_CONFIGS; i++) {
    updateConfig(axis.configs[i], value, now);
  }
}

// ── Per-config state machine ──────────────────────────────────────────────────
void JumpDetector::updateConfig(JumpConfig &config, float value,
                                uint32_t now) {
  // Two-stage EMA filter
  config.filteredFast = FILTER_ALPHA_FAST * value +
                        (1.0f - FILTER_ALPHA_FAST) * config.filteredFast;
  config.filteredSlow = FILTER_ALPHA_SLOW * value +
                        (1.0f - FILTER_ALPHA_SLOW) * config.filteredSlow;

  float fv = config.filteredFast;

  switch (config.state) {

  // ── IDLE: watch for the start of an upward push ───────────────────────────
  case STATE_IDLE:
    if (fv > config.lastValue && fv > MIN_PEAK_VALUE) {
      config.state           = STATE_RISING;
      config.risingStartTime = now;
      config.peak            = fv;
    }
    break;

  // ── RISING: track the peak; confirm it once the signal drops back ─────────
  case STATE_RISING:
    if (fv > config.peak) config.peak = fv;

    if (fv < config.peak * PEAK_DROP_THRESHOLD) {
      uint32_t riseDuration = now - config.risingStartTime;

      if (riseDuration >= config.minRiseDuration - TIMING_TOLERANCE_MS &&
          riseDuration <= config.minRiseDuration + TIMING_TOLERANCE_MS) {
        config.state            = STATE_FALLING;
        config.fallingStartTime = now;
        config.valley           = fv;
      } else {
        config.state = STATE_IDLE; // wrong timing — not a jump shape
      }
    }

    if (now - config.risingStartTime > MAX_PHASE_DURATION_MS)
      config.state = STATE_IDLE;
    break;

  // ── FALLING: wait for fall to complete, then register the jump ────────────
  case STATE_FALLING: {
    if (fv < config.valley) config.valley = fv;

    uint32_t fallDuration = now - config.fallingStartTime;

    if (fallDuration >= config.minFallDuration - TIMING_TOLERANCE_MS &&
        fallDuration <= config.minFallDuration + TIMING_TOLERANCE_MS) {

      // Peak-to-valley swing in physical units (g)
      float diff = fabsf(config.peak - config.valley);

      float threshold = _avgJump * _thresholdFactor;
      if (!_calibrationComplete) threshold *= 0.7f; // lenient during warmup

      if (diff > threshold &&
          now - config.lastJumpTime > _minIntervalMs) {

        config.jumpCount++;
        config.lastJumpTime = now;

        // Update adaptive threshold (clamped to physical bounds)
        _avgJump = 0.92f * _avgJump + 0.08f * diff;
        _avgJump = fmaxf(MIN_THRESHOLD, fminf(MAX_THRESHOLD, _avgJump));

        if (!_calibrationComplete) {
          if (++_calibrationJumps >= CALIBRATION_JUMPS)
            _calibrationComplete = true;
        }

        ESP_LOGD(TAG, "[%s] Jump! diff=%.3fg threshold=%.3fg count=%lu",
                 _axisVertical.name, diff, threshold,
                 (unsigned long)config.jumpCount);
      }

      config.state = STATE_IDLE;
    }

    if (fallDuration > MAX_PHASE_DURATION_MS)
      config.state = STATE_IDLE;

    break;
  }
  } // switch

  config.lastValue = fv;
}

// ── Utility ───────────────────────────────────────────────────────────────────
int JumpDetector::getConfigIndex(JumpConfig *config) {
  for (int i = 0; i < NUM_TIMING_CONFIGS; i++) {
    if (&_axisVertical.configs[i] == config) return i;
  }
  return -1;
}

void JumpDetector::getCounts(uint32_t countsZ[NUM_TIMING_CONFIGS]) {
  for (int i = 0; i < NUM_TIMING_CONFIGS; i++) {
    if (countsZ) countsZ[i] = _axisVertical.configs[i].jumpCount;
  }
}

void JumpDetector::getTimingConfig(int configIndex,
                                   uint32_t &riseDuration,
                                   uint32_t &fallDuration) {
  if (configIndex >= 0 && configIndex < NUM_TIMING_CONFIGS) {
    riseDuration = TIMING_CONFIGS[configIndex].rise;
    fallDuration = TIMING_CONFIGS[configIndex].fall;
  }
}

uint32_t JumpDetector::getAxisTotal(const AxisDetector &axis) const {
  // Return the count from config index 2 (the "Slow" 200 ms pattern)
  return axis.configs[2].jumpCount;
}

float JumpDetector::getAxisRate(const AxisDetector &axis) const {
  uint32_t maxJumps    = 0;
  uint32_t lastJumpTime = 0;

  for (int i = 0; i < NUM_TIMING_CONFIGS; i++) {
    if (axis.configs[i].jumpCount > maxJumps) {
      maxJumps     = axis.configs[i].jumpCount;
      lastJumpTime = axis.configs[i].lastJumpTime;
    }
  }

  if (maxJumps < 2 || lastJumpTime == 0) return 0.0f;

  float timeMinutes = lastJumpTime / 60000.0f;
  return maxJumps / timeMinutes;
}

void JumpDetector::getTotalJumps(uint32_t &totalZ) const {
  totalZ = getAxisTotal(_axisVertical);
}

void JumpDetector::getAverageRates(float &rateZ) const {
  rateZ = getAxisRate(_axisVertical);
}

void JumpDetector::resetSession() {
  _avgJump             = INITIAL_THRESHOLD;
  _calibrationComplete = false;
  _calibrationJumps    = 0;

  for (int i = 0; i < NUM_TIMING_CONFIGS; i++) {
    JumpConfig &c      = _axisVertical.configs[i];
    c.jumpCount        = 0;
    c.lastJumpTime     = 0;
    c.state            = STATE_IDLE;
    c.peak             = 0.0f;
    c.valley           = 0.0f;
    c.risingStartTime  = 0;
    c.fallingStartTime = 0;
    c.lastValue        = 0.0f;
    c.filteredFast     = 0.0f;
    c.filteredSlow     = 0.0f;
  }
}

bool JumpDetector::isCalibrated() const { return _calibrationComplete; }