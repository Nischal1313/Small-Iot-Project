#ifndef JUMP_H
#define JUMP_H

#include "gyro.h"
#include <cstdint>

// Number of timing configurations to test in parallel
#define NUM_TIMING_CONFIGS 4

// Detector state machine
enum DetectorState { STATE_IDLE, STATE_RISING, STATE_FALLING };

// Configuration for a single timing pattern
struct JumpConfig {
  uint32_t minRiseDuration;  // Expected rise time (ms)
  uint32_t minFallDuration;  // Expected fall time (ms)
  uint32_t jumpCount;        // Jumps detected with this config
  uint32_t lastJumpTime;     // Last jump timestamp (ms)
  DetectorState state;       // Current state machine state
  float peak;                // Current peak value (g)
  float valley;              // Current valley value (g)
  uint32_t risingStartTime;  // When rise phase started (ms)
  uint32_t fallingStartTime; // When fall phase started (ms)
  float lastValue;           // Previous filtered value
  float filteredFast;        // Fast EMA — tracks peaks
  float filteredSlow;        // Slow EMA — tracks baseline
};

// Single axis detector (kept for structural compatibility; only Z/vertical used)
struct AxisDetector {
  char name[8];
  JumpConfig configs[NUM_TIMING_CONFIGS];
};

class JumpDetector {
public:
  // sensor         — shared SensorReading singleton (must be initialised)
  // thresholdFactor — adaptive threshold multiplier (default 1.3×)
  // minIntervalMs  — minimum time between registered jumps (default 300 ms)
  JumpDetector(SensorReading *sensor,
               float    thresholdFactor = 1.3f,
               uint32_t minIntervalMs   = 300);

  // Call once per loop tick (or from a FreeRTOS task via jumpDetectionTask)
  void update();

  // FreeRTOS task entry — calls update() in a 10 ms loop, drains the gyro queue
  void jumpDetectionTask();

  // Jump counts per timing config for the vertical axis
  void getCounts(uint32_t countsZ[NUM_TIMING_CONFIGS]);

  // Retrieve rise/fall durations for a given config index
  void getTimingConfig(int configIndex,
                       uint32_t &riseDuration,
                       uint32_t &fallDuration);

  // Best-config jump total and rate (jumps/min) for the vertical axis
  void  getTotalJumps(uint32_t &totalZ) const;
  void  getAverageRates(float &rateZ) const;

  void  resetSession();
  bool  isCalibrated() const;

private:
  SensorReading *_sensor;
  float    _thresholdFactor;
  uint32_t _minIntervalMs;

  AxisDetector _axisVertical;   // Operates on vertical_accel_g from the filter

  float    _avgJump;            // Adaptive threshold baseline (g)
  bool     _calibrationComplete;
  uint32_t _calibrationJumps;

  // ── Helpers ──────────────────────────────────────────────────────────────
  uint32_t getMillis();
  void     updateAxis  (AxisDetector &axis, float value, uint32_t now);
  void     updateConfig(JumpConfig   &cfg,  float value, uint32_t now);
  int      getConfigIndex(JumpConfig *cfg);
  uint32_t getAxisTotal(const AxisDetector &axis) const;
  float    getAxisRate (const AxisDetector &axis) const;
};

#endif // JUMP_H