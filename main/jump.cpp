#include "jump.h"
#include "gyro.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include <climits>
#include <cmath>
#include <cstring>

namespace
{

char const* TAG_S = "JumpDetector";

int constexpr TIMING_TOLERANCE_MS   = 40;
int constexpr MAX_PHASE_DURATION_MS = 800;

constexpr bool isValidTransition(DetectorState fromP, DetectorState toP)
{
    switch (fromP)
    {
        case DetectorState::Idle:
            return toP == DetectorState::Rising;

        case DetectorState::Rising:
            return toP == DetectorState::Falling || toP == DetectorState::Idle;

        case DetectorState::Falling:
            return toP == DetectorState::Idle;
    }

    return false;
}
float constexpr FILTER_ALPHA_FAST = 0.4f;
float constexpr FILTER_ALPHA_SLOW = 0.15f;
int constexpr CALIBRATION_JUMPS   = 5;

float constexpr PEAK_DROP_THRESHOLD = 0.85f;
float constexpr MIN_PEAK_VALUE      = 0.15f;

float constexpr MIN_THRESHOLD         = 0.15f;
float constexpr MAX_THRESHOLD         = 4.0f;
float constexpr INITIAL_THRESHOLD     = 0.4f;
float constexpr JUMP_THRESHOLD_FACTOR = 1.3f;

int constexpr MIN_JUMP_INTERVAL_MS = 300;
int constexpr TASK_LOOP_MS         = 10;

struct TimingConfig
{
    uint32_t riseM;
    uint32_t fallM;
};

TimingConfig const TIMING_CONFIGS_S[NUM_TIMING_CONFIGS] = {
    { 180, 180 },
    { 190, 190 },
    { 200, 200 },
    { 210, 210 },
};

}  // namespace

JumpDetector::JumpDetector(SensorReading* pSensorP, float thresholdFactorP, uint32_t minIntervalMsP)
    : _pSensorM{ pSensorP }
    , _thresholdFactorM{ thresholdFactorP }
    , _minIntervalMsM{ minIntervalMsP }
    , _avgJumpM{ INITIAL_THRESHOLD }
    , _calibrationCompleteM{ false }
    , _calibrationJumpsM{ 0 }
{
    std::strcpy(_axisVerticalM.nameM, "Vert");

    for (uint32_t i{ 0 }; i < NUM_TIMING_CONFIGS; i++)
    {
        JumpConfig& rC       = _axisVerticalM.configsM[i];
        rC.minRiseDurationM  = TIMING_CONFIGS_S[i].riseM;
        rC.minFallDurationM  = TIMING_CONFIGS_S[i].fallM;
        rC.jumpCountM        = 0;
        rC.lastJumpTimeM     = 0;
        rC.stateM            = DetectorState::Idle;
        rC.peakM             = 0.0f;
        rC.valleyM           = 0.0f;
        rC.risingStartTimeM  = 0;
        rC.fallingStartTimeM = 0;
        rC.lastValueM        = 0.0f;
        rC.filteredFastM     = 0.0f;
        rC.filteredSlowM     = 0.0f;
    }
}

uint32_t JumpDetector::getMillis()
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

void JumpDetector::update()
{
    configASSERT(_pSensorM != nullptr);

    if (!_pSensorM->isInitialized())
    {
        return;
    }

    mpu_data_t data{};
    if (xQueueReceive(_pSensorM->getQueue(), &data, 0) != pdTRUE)
    {
        return;
    }

    uint32_t const now = getMillis();
    updateAxis(_axisVerticalM, data.verticalAccelGM, now);
}

void JumpDetector::jumpDetectionTask()
{
    while (true)
    {
        update();
        vTaskDelay(pdMS_TO_TICKS(TASK_LOOP_MS));
    }
}

void JumpDetector::updateAxis(AxisDetector& rAxisP, float valueP, uint32_t nowP)
{
    for (uint32_t i{ 0 }; i < NUM_TIMING_CONFIGS; i++)
    {
        updateConfig(rAxisP.configsM[i], valueP, nowP);
    }
}

void JumpDetector::updateConfig(JumpConfig& rCfgP, float valueP, uint32_t nowP)
{
    rCfgP.filteredFastM =
        FILTER_ALPHA_FAST * valueP + (1.0f - FILTER_ALPHA_FAST) * rCfgP.filteredFastM;
    rCfgP.filteredSlowM =
        FILTER_ALPHA_SLOW * valueP + (1.0f - FILTER_ALPHA_SLOW) * rCfgP.filteredSlowM;

    float const fv           = rCfgP.filteredFastM;
    DetectorState const prev = rCfgP.stateM;
    DetectorState next       = prev;

    switch (prev)
    {
        case DetectorState::Idle:
            if (fv > rCfgP.lastValueM && fv > MIN_PEAK_VALUE)
            {
                next                   = DetectorState::Rising;
                rCfgP.risingStartTimeM = nowP;
                rCfgP.peakM            = fv;
            }
            break;

        case DetectorState::Rising:
            if (fv > rCfgP.peakM)
            {
                rCfgP.peakM = fv;
            }

            if (fv < rCfgP.peakM * PEAK_DROP_THRESHOLD)
            {
                uint32_t const riseDuration = nowP - rCfgP.risingStartTimeM;

                if (riseDuration >= rCfgP.minRiseDurationM - TIMING_TOLERANCE_MS &&
                    riseDuration <= rCfgP.minRiseDurationM + TIMING_TOLERANCE_MS)
                {
                    next                    = DetectorState::Falling;
                    rCfgP.fallingStartTimeM = nowP;
                    rCfgP.valleyM           = fv;
                }
                else
                {
                    next = DetectorState::Idle;
                }
            }

            if (next == prev && nowP - rCfgP.risingStartTimeM > MAX_PHASE_DURATION_MS)
            {
                next = DetectorState::Idle;
            }
            break;

        case DetectorState::Falling:
        {
            if (fv < rCfgP.valleyM)
            {
                rCfgP.valleyM = fv;
            }

            uint32_t const fallDuration = nowP - rCfgP.fallingStartTimeM;

            if (fallDuration >= rCfgP.minFallDurationM - TIMING_TOLERANCE_MS &&
                fallDuration <= rCfgP.minFallDurationM + TIMING_TOLERANCE_MS)
            {
                float const diff = fabsf(rCfgP.peakM - rCfgP.valleyM);
                float threshold  = _avgJumpM * _thresholdFactorM;

                if (!_calibrationCompleteM)
                {
                    threshold *= 0.7f;
                }

                if (diff > threshold && nowP - rCfgP.lastJumpTimeM > _minIntervalMsM)
                {
                    configASSERT(rCfgP.jumpCountM < UINT32_MAX);
                    rCfgP.jumpCountM++;
                    rCfgP.lastJumpTimeM = nowP;

                    _avgJumpM = 0.92f * _avgJumpM + 0.08f * diff;
                    _avgJumpM = fmaxf(MIN_THRESHOLD, fminf(MAX_THRESHOLD, _avgJumpM));

                    if (!_calibrationCompleteM)
                    {
                        if (++_calibrationJumpsM >= CALIBRATION_JUMPS)
                        {
                            _calibrationCompleteM = true;
                        }
                    }

                    ESP_LOGD(
                        TAG_S,
                        "[%s] Jump! diff=%.3fg threshold=%.3fg count=%lu",
                        _axisVerticalM.nameM,
                        diff,
                        threshold,
                        (unsigned long) rCfgP.jumpCountM
                    );
                }

                next = DetectorState::Idle;
            }

            if (next == prev && fallDuration > MAX_PHASE_DURATION_MS)
            {
                next = DetectorState::Idle;
            }
            break;
        }
    }

    configASSERT(isValidTransition(prev, next));
    configASSERT(next != prev || next == DetectorState::Rising);

    rCfgP.stateM     = next;
    rCfgP.lastValueM = fv;
}

int JumpDetector::getConfigIndex(JumpConfig* pCfgP)
{
    int result{ -1 };

    for (uint32_t i{ 0 }; i < NUM_TIMING_CONFIGS; i++)
    {
        if (&_axisVerticalM.configsM[i] == pCfgP)
        {
            result = static_cast<int>(i);
            break;
        }
    }

    return result;
}

void JumpDetector::getCounts(uint32_t* pCountsZP)
{
    if (pCountsZP == nullptr)
    {
        return;
    }

    for (uint32_t i{ 0 }; i < NUM_TIMING_CONFIGS; i++)
    {
        pCountsZP[i] = _axisVerticalM.configsM[i].jumpCountM;
    }
}

void JumpDetector::getTimingConfig(
    int configIndexP, uint32_t& rRiseDurationP, uint32_t& rFallDurationP
)
{
    if (configIndexP >= 0 && static_cast<uint32_t>(configIndexP) < NUM_TIMING_CONFIGS)
    {
        rRiseDurationP = TIMING_CONFIGS_S[configIndexP].riseM;
        rFallDurationP = TIMING_CONFIGS_S[configIndexP].fallM;
    }
}

uint32_t JumpDetector::getAxisTotal(AxisDetector const& rAxisP) const
{
    return rAxisP.configsM[2].jumpCountM;
}

float JumpDetector::getAxisRate(AxisDetector const& rAxisP) const
{
    uint32_t maxJumps{ 0 };
    uint32_t lastJumpTime{ 0 };

    for (uint32_t i{ 0 }; i < NUM_TIMING_CONFIGS; i++)
    {
        if (rAxisP.configsM[i].jumpCountM > maxJumps)
        {
            maxJumps     = rAxisP.configsM[i].jumpCountM;
            lastJumpTime = rAxisP.configsM[i].lastJumpTimeM;
        }
    }

    float result{ 0.0f };

    if (maxJumps >= 2 && lastJumpTime != 0)
    {
        float const timeMinutes = static_cast<float>(lastJumpTime) / 60000.0f;
        result                  = static_cast<float>(maxJumps) / timeMinutes;
    }

    return result;
}

void JumpDetector::getTotalJumps(uint32_t& rTotalZP) const
{
    rTotalZP = getAxisTotal(_axisVerticalM);
}

void JumpDetector::getAverageRates(float& rRateZP) const
{
    rRateZP = getAxisRate(_axisVerticalM);
}

void JumpDetector::resetSession()
{
    _avgJumpM             = INITIAL_THRESHOLD;
    _calibrationCompleteM = false;
    _calibrationJumpsM    = 0;

    for (uint32_t i{ 0 }; i < NUM_TIMING_CONFIGS; i++)
    {
        JumpConfig& rC       = _axisVerticalM.configsM[i];
        rC.jumpCountM        = 0;
        rC.lastJumpTimeM     = 0;
        rC.stateM            = DetectorState::Idle;
        rC.peakM             = 0.0f;
        rC.valleyM           = 0.0f;
        rC.risingStartTimeM  = 0;
        rC.fallingStartTimeM = 0;
        rC.lastValueM        = 0.0f;
        rC.filteredFastM     = 0.0f;
        rC.filteredSlowM     = 0.0f;
    }
}

bool JumpDetector::isCalibrated() const
{
    return _calibrationCompleteM;
}
