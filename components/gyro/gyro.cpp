#include "gyro.h"
#include "esp_log.h"
#include "esp_timer.h"  // esp_timer_get_time() -> microseconds
#include "i2cInit.h"
#include "mutex.h"
#include <cmath>

namespace
{
char const* TAG_S{"G&A sensor"};

// ─────────────────────────────────────────────────────────────────────────────
// Complementary filter alpha:
//
//   gravity_new = alpha * (gyro_rotated_gravity) + (1-alpha) * accel_reading
//
// At 100 Hz (dt = 0.01 s) an alpha of 0.98 gives the gyro a time-constant of
// roughly 0.5 s before the accelerometer can pull it back.  That is long
// enough to ride through a jump (~0.3-0.5 s airborne) without the free-fall
// accel signal corrupting the gravity estimate, yet short enough to correct
// gyro drift over several seconds of normal use.
// ─────────────────────────────────────────────────────────────────────────────
float constexpr DEFAULT_ALPHA_S{0.98f};
}  // namespace

// ── Constructor ───────────────────────────────────────────────────────────────
SensorReading::SensorReading()
    : _initializedM{false}
    , _calibratedM{false}
    , accelSensitivityM{16384.0f}
    , gyroSensitivityM{131.0f}
    , _gravXM{0.0f}
    , _gravYM{0.0f}
    , _gravZM{1.0f}
    , _alphaM{DEFAULT_ALPHA_S}
    , _lastUsM{0}
    , dataQueueM{xQueueCreate(10, sizeof(mpu_data_t))}
{
    init();
}

// ── Hardware init ─────────────────────────────────────────────────────────────
void SensorReading::init()
{
    I2CManager& i2c{I2CManager::getInstance()};
    if (!i2c.isInitialized())
    {
        ESP_LOGE(TAG_S, "I2C Manager not initialized!");
        return;
    }

    ESP_LOGI(TAG_S, "Initializing MPU6050...");
    {
        MutexGuard lock(i2c.getMutex());

        // Wake the chip (clear SLEEP bit in PWR_MGMT_1)
        uint8_t wake[2]{0x6B, 0x00};
        esp_err_t err{
            i2c_master_write_to_device(i2c.getPort(), MPU_ADDR, wake, 2, pdMS_TO_TICKS(100))
        };
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG_S, "Failed to wake MPU: %s", esp_err_to_name(err));
            return;
        }
        readSensitivity();
    }

    _initializedM = true;
    ESP_LOGI(TAG_S, "MPU6050 initialized successfully");

    // Seed the gravity estimate while the device is presumably still at boot.
    calibrateGravity(50);
}

// ── Sensitivity registers ─────────────────────────────────────────────────────
void SensorReading::readSensitivity()
{
    I2CManager& i2c{I2CManager::getInstance()};
    uint8_t aCfg{}, gCfg{};

    i2c_master_write_read_device(
        i2c.getPort(), MPU_ADDR, (uint8_t*) "\x1C", 1, &aCfg, 1, pdMS_TO_TICKS(100)
    );
    i2c_master_write_read_device(
        i2c.getPort(), MPU_ADDR, (uint8_t*) "\x1B", 1, &gCfg, 1, pdMS_TO_TICKS(100)
    );

    switch ((aCfg >> 3) & 0x03)
    {
        case 0:
            accelSensitivityM = 16384.0f;
            break;
        case 1:
            accelSensitivityM = 8192.0f;
            break;
        case 2:
            accelSensitivityM = 4096.0f;
            break;
        case 3:
            accelSensitivityM = 2048.0f;
            break;
    }

    switch ((gCfg >> 3) & 0x03)
    {
        case 0:
            gyroSensitivityM = 131.0f;
            break;
        case 1:
            gyroSensitivityM = 65.5f;
            break;
        case 2:
            gyroSensitivityM = 32.8f;
            break;
        case 3:
            gyroSensitivityM = 16.4f;
            break;
    }

    ESP_LOGI(
        TAG_S,
        "Accel sensitivity: %.1f LSB/g   Gyro sensitivity: %.1f LSB/(deg/s)",
        accelSensitivityM,
        gyroSensitivityM
    );
}

// ── Raw sensor reads ──────────────────────────────────────────────────────────
esp_err_t SensorReading::readRawAccel(int16_t& rAxP, int16_t& rAyP, int16_t& rAzP)
{
    esp_err_t result{ESP_FAIL};

    if (_initializedM)
    {
        I2CManager& i2c{I2CManager::getInstance()};
        MutexGuard lock(i2c.getMutex());

        uint8_t reg{0x3B};
        uint8_t raw[6]{};
        result =
            i2c_master_write_read_device(i2c.getPort(), MPU_ADDR, &reg, 1, raw, 6, pdMS_TO_TICKS(100));
        if (result == ESP_OK)
        {
            rAxP = (int16_t) ((raw[0] << 8) | raw[1]);
            rAyP = (int16_t) ((raw[2] << 8) | raw[3]);
            rAzP = (int16_t) ((raw[4] << 8) | raw[5]);
        }
    }

    return result;
}

esp_err_t SensorReading::readRawGyro(int16_t& rGxP, int16_t& rGyP, int16_t& rGzP)
{
    esp_err_t result{ESP_FAIL};

    if (_initializedM)
    {
        I2CManager& i2c{I2CManager::getInstance()};
        MutexGuard lock(i2c.getMutex());

        uint8_t reg{0x43};
        uint8_t raw[6]{};
        result =
            i2c_master_write_read_device(i2c.getPort(), MPU_ADDR, &reg, 1, raw, 6, pdMS_TO_TICKS(100));
        if (result == ESP_OK)
        {
            rGxP = (int16_t) ((raw[0] << 8) | raw[1]);
            rGyP = (int16_t) ((raw[2] << 8) | raw[3]);
            rGzP = (int16_t) ((raw[4] << 8) | raw[5]);
        }
    }

    return result;
}

// ── Gravity calibration ───────────────────────────────────────────────────────
// Averages N still accel readings to find the initial gravity direction.
// This seeds _grav_xyz so the complementary filter starts from a good position
// rather than the generic [0,0,1] default.
void SensorReading::calibrateGravity(int samplesP)
{
    if (!_initializedM)
    {
        ESP_LOGW(TAG_S, "Cannot calibrate: sensor not initialized");
        return;
    }

    ESP_LOGI(TAG_S, "Calibrating gravity vector (%d samples, device must be still)...", samplesP);

    double sx{0.0}, sy{0.0}, sz{0.0};
    int valid{0};

    for (int i{0}; i < samplesP; i++)
    {
        int16_t ax{}, ay{}, az{};
        if (readRawAccel(ax, ay, az) == ESP_OK)
        {
            sx += ax / accelSensitivityM;
            sy += ay / accelSensitivityM;
            sz += az / accelSensitivityM;
            valid++;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (valid == 0)
    {
        ESP_LOGE(TAG_S, "Gravity calibration failed: no valid samples");
        return;
    }

    _gravXM = (float) (sx / valid);
    _gravYM = (float) (sy / valid);
    _gravZM = (float) (sz / valid);

    float mag{sqrtf(_gravXM * _gravXM + _gravYM * _gravYM + _gravZM * _gravZM)};
    ESP_LOGI(TAG_S, "Gravity seed: [%.3f, %.3f, %.3f]  |g| = %.3f g", _gravXM, _gravYM, _gravZM, mag);

    if (mag < 0.8f || mag > 1.2f)
    {
        ESP_LOGW(
            TAG_S,
            "Gravity magnitude %.2f g is out of expected range - "
            "was the device moving during calibration?",
            mag
        );
    }

    // Reset the timer so the first loop tick computes a clean dt
    _lastUsM    = esp_timer_get_time();
    _calibratedM = true;
}

// ── Complementary filter: gyro rotation step ──────────────────────────────────
//
// The device has rotated by (gx,gy,gz) rad/s over dt seconds.
// We apply that same rotation to the stored gravity vector so it tracks
// how "down" has moved relative to the sensor axes.
//
// Rodrigues small-angle formula for rotating vector v by omega * dt:
//
//   v' = v + (omega x v) * dt
//
// This is accurate to first order in |omega| * dt.  At 100 Hz and typical
// human-motion rates (<< 10 rad/s) the error is < 0.05 % per step.
// ─────────────────────────────────────────────────────────────────────────────
void SensorReading::rotateGravity(float gxRpsP, float gyRpsP, float gzRpsP, float dtP)
{
    // Cross product  omega x g
    float cx{gyRpsP * _gravZM - gzRpsP * _gravYM};
    float cy{gzRpsP * _gravXM - gxRpsP * _gravZM};
    float cz{gxRpsP * _gravYM - gyRpsP * _gravXM};

    // Integrate: g' = g + (omega x g) * dt
    _gravXM += cx * dtP;
    _gravYM += cy * dtP;
    _gravZM += cz * dtP;
}

// ── Complementary filter: full update ────────────────────────────────────────
//
// Each tick we:
//   1. Propagate the gravity estimate through the gyro rotation  (gyro step)
//   2. Blend that prediction with the raw accel reading           (accel correction)
//   3. Re-normalise to keep |grav| = 1 g                         (normalise)
//
// The blend weight alpha controls the trade-off:
//   - Gyro step alone is smooth but drifts (integrates bias over time).
//   - Accel reading alone is accurate long-term but noisy and wrong mid-jump.
//   - alpha = 0.98 keeps the gyro dominant for ~0.5 s, long enough to span a jump.
//
// We expose the final gravity vector in the output struct so callers can
// visualise or log it for tuning.
// ─────────────────────────────────────────────────────────────────────────────
float SensorReading::computeVerticalAccel(float axGP, float ayGP, float azGP) const
{
    float result{0.0f};
    float mag{sqrtf(_gravXM * _gravXM + _gravYM * _gravYM + _gravZM * _gravZM)};

    if (mag >= 1e-6f)
    {
        // Unit vector pointing in the gravity direction (sensor frame)
        float ux{_gravXM / mag};
        float uy{_gravYM / mag};
        float uz{_gravZM / mag};

        // Scalar projection of the current accel onto the gravity axis
        float proj{axGP * ux + ayGP * uy + azGP * uz};

        // Subtract the static 1 g contribution -> 0 at rest,
        // positive upward, negative downward
        result = proj - mag;
    }

    return result;
}

// ── Task plumbing ─────────────────────────────────────────────────────────────
void SensorReading::taskEntry(void* pParamP)
{
    static_cast<SensorReading*>(pParamP)->taskLoop();
}

void SensorReading::startTask()
{
    xTaskCreate(taskEntry, "mpu_reader", 4096, this, 5, nullptr);
}

// ── Main sensor loop ──────────────────────────────────────────────────────────
void SensorReading::taskLoop()
{
    // Seed the timer on first entry (calibrateGravity may have set it already)
    if (_lastUsM == 0)
        _lastUsM = esp_timer_get_time();

    while (true)
    {
        // ── 1. Compute real dt ────────────────────────────────────────────────
        int64_t nowUs{esp_timer_get_time()};
        float dt{(nowUs - _lastUsM) * 1e-6f};  // convert us -> seconds
        _lastUsM       = nowUs;

        // Guard against spurious large dt (e.g. first tick after a stall)
        if (dt <= 0.0f || dt > 0.5f)
            dt = 0.01f;

        // ── 2. Read sensors ───────────────────────────────────────────────────
        int16_t ax16{}, ay16{}, az16{}, gx16{}, gy16{}, gz16{};
        if (readRawAccel(ax16, ay16, az16) != ESP_OK || readRawGyro(gx16, gy16, gz16) != ESP_OK)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // Convert to physical units
        float axG{ax16 / accelSensitivityM};  // g
        float ayG{ay16 / accelSensitivityM};
        float azG{az16 / accelSensitivityM};

        float gxDps{gx16 / gyroSensitivityM};  // degrees / second
        float gyDps{gy16 / gyroSensitivityM};
        float gzDps{gz16 / gyroSensitivityM};

        // ── 3. Complementary filter ───────────────────────────────────────────
        if (_calibratedM)
        {
            // Convert gyro to rad/s for the rotation step
            float constexpr DEG2RAD{0.017453293f};
            float gxRps{gxDps * DEG2RAD};
            float gyRps{gyDps * DEG2RAD};
            float gzRps{gzDps * DEG2RAD};

            // Step A - Gyro prediction: rotate gravity estimate by omega * dt
            rotateGravity(gxRps, gyRps, gzRps, dt);

            // Step B - Accel correction: blend gyro prediction with raw accel.
            // During free-fall/jump the accel magnitude is far from 1 g, so we
            // reduce the accel trust proportionally to its deviation from 1 g.
            // This prevents a mid-jump free-fall reading from pulling the gravity
            // estimate toward zero.
            float accelMag{sqrtf(axG * axG + ayG * ayG + azG * azG)};
            float deviation{fabsf(accelMag - 1.0f)};  // 0 at rest, ~1 in free-fall

            // Scale accel weight down when the device is clearly in motion.
            // deviation < 0.1 g  -> full accel correction  (weight = 1-alpha)
            // deviation > 0.5 g  -> nearly zero correction (trust gyro only)
            float accelWeight{(1.0f - _alphaM) * fmaxf(0.0f, 1.0f - deviation / 0.5f)};

            float gyroWeight{1.0f - accelWeight};

            _gravXM = gyroWeight * _gravXM + accelWeight * axG;
            _gravYM = gyroWeight * _gravYM + accelWeight * ayG;
            _gravZM = gyroWeight * _gravZM + accelWeight * azG;

            // Step C - Re-normalise to 1 g so magnitude never drifts
            float mag{sqrtf(_gravXM * _gravXM + _gravYM * _gravYM + _gravZM * _gravZM)};
            if (mag > 1e-6f)
            {
                _gravXM /= mag;
                _gravYM /= mag;
                _gravZM /= mag;
            }
        }

        // ── 4. Build output ───────────────────────────────────────────────────
        mpu_data_t data{};
        data.gxDpsM = gxDps;
        data.gyDpsM = gyDps;
        data.gzDpsM = gzDps;
        data.gravXM = _gravXM;
        data.gravYM = _gravYM;
        data.gravZM = _gravZM;

        if (_calibratedM)
        {
            data.verticalAccelGM = computeVerticalAccel(axG, ayG, azG);
        }
        else
        {
            // Pre-calibration fallback: assume device is flat, Z axis is up
            data.verticalAccelGM = azG - 1.0f;
        }

        xQueueSend(dataQueueM, &data, 0);

        vTaskDelay(pdMS_TO_TICKS(10));  // 100 Hz loop
    }
}
