#pragma once
#include "driver/i2c.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <cstdint>

struct mpu_data_t
{
    // World-frame vertical acceleration (gravity removed, orientation-independent).
    // 0 at rest. Positive = accelerating upward. Negative = falling / downward push.
    float verticalAccelGM;

    // Current complementary-filter gravity estimate in the sensor frame (units: g).
    // Useful for debugging or feeding into a higher-level detector.
    float gravXM;
    float gravYM;
    float gravZM;

    // Raw gyro in degrees-per-second
    float gxDpsM;
    float gyDpsM;
    float gzDpsM;
};

class SensorReading
{
public:
    // Singleton
    static SensorReading& getInstance()
    {
        static SensorReading instance;
        return instance;
    }

    // ── Public API ────────────────────────────────────────────────────────────
    void startTask();
    QueueHandle_t getQueue() const
    {
        return dataQueueM;
    }

    esp_err_t readRawAccel(int16_t& rAxP, int16_t& rAyP, int16_t& rAzP);
    esp_err_t readRawGyro(int16_t& rGxP, int16_t& rGyP, int16_t& rGzP);

    bool isInitialized() const
    {
        return _initializedM;
    }
    bool isCalibrated() const
    {
        return _calibratedM;
    }

    // Re-seed the gravity estimate from a still-phase average.
    // Device must be stationary for ~(samples x 10 ms).
    void calibrateGravity(int samplesP = 50);

    // Tune the complementary filter blend at runtime (default alpha = 0.98).
    //   Higher alpha -> trust gyro more (smoother, drifts slowly over minutes).
    //   Lower  alpha -> trust accel more (corrects drift faster, noisier during motion).
    void setAlpha(float alphaP)
    {
        _alphaM = alphaP;
    }

    SensorReading();

private:
    static uint8_t constexpr MPU_ADDR{0x68};
    SensorReading(SensorReading const&)            = delete;
    SensorReading& operator=(SensorReading const&) = delete;

    // ── Sensor config ─────────────────────────────────────────────────────────
    bool _initializedM;
    bool _calibratedM;
    float accelSensitivityM;  // raw LSB per g
    float gyroSensitivityM;   // raw LSB per deg/s

    // ── Complementary filter state ────────────────────────────────────────────
    // Running gravity estimate in the sensor frame (units: g).
    // Seeded by calibrateGravity(), then updated every tick by rotateGravity()
    // and blended with the raw accelerometer reading.
    float _gravXM;
    float _gravYM;
    float _gravZM;

    float _alphaM;      // gyro trust weight  (0 < alpha < 1)
    int64_t _lastUsM;  // timestamp of previous loop tick (microseconds)

    // ── Internal helpers ──────────────────────────────────────────────────────
    QueueHandle_t dataQueueM;

    void init();
    void readSensitivity();

    // Rotate the stored gravity estimate by (gx,gy,gz) [rad/s] over dt [s].
    // Uses the Rodrigues small-angle approximation - O(1) and accurate for
    // the short dt values produced by our 100 Hz loop.
    void rotateGravity(float gxRpsP, float gyRpsP, float gzRpsP, float dtP);

    // Project the raw accel vector onto the current gravity estimate,
    // subtract the static 1 g offset, and return signed vertical acceleration.
    float computeVerticalAccel(float axGP, float ayGP, float azGP) const;

    void taskLoop();
    static void taskEntry(void* pParamP);
};
