#pragma once

#include "gyro.h"
#include <cstdint>

constexpr uint32_t NUM_TIMING_CONFIGS{ 4 };

enum class DetectorState : uint8_t
{
    Idle,
    Rising,
    Falling
};

struct JumpConfig
{
    uint32_t minRiseDurationM;
    uint32_t minFallDurationM;
    uint32_t jumpCountM;
    uint32_t lastJumpTimeM;
    DetectorState stateM;
    float peakM;
    float valleyM;
    uint32_t risingStartTimeM;
    uint32_t fallingStartTimeM;
    float lastValueM;
    float filteredFastM;
    float filteredSlowM;
};

struct AxisDetector
{
    char nameM[8];
    JumpConfig configsM[NUM_TIMING_CONFIGS];
};

class JumpDetector
{
public:
    JumpDetector(
        SensorReading* pSensorP, float thresholdFactorP = 1.3f, uint32_t minIntervalMsP = 300
    );

    void update();
    void jumpDetectionTask();
    void getCounts(uint32_t* pCountsZP);
    void getTimingConfig(int configIndexP, uint32_t& rRiseDurationP, uint32_t& rFallDurationP);
    void getTotalJumps(uint32_t& rTotalZP) const;
    void getAverageRates(float& rRateZP) const;
    void resetSession();
    bool isCalibrated() const;

private:
    SensorReading* _pSensorM;
    float _thresholdFactorM;
    uint32_t _minIntervalMsM;

    AxisDetector _axisVerticalM;

    float _avgJumpM;
    bool _calibrationCompleteM;
    uint32_t _calibrationJumpsM;

    uint32_t getMillis();
    void updateAxis(AxisDetector& rAxisP, float valueP, uint32_t nowP);
    void updateConfig(JumpConfig& rCfgP, float valueP, uint32_t nowP);
    int getConfigIndex(JumpConfig* pCfgP);
    uint32_t getAxisTotal(AxisDetector const& rAxisP) const;
    float getAxisRate(AxisDetector const& rAxisP) const;
};
