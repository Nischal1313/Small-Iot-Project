#include "display_controller.h"
#include <cstdio>

DisplayController::DisplayController(OledDisplay &rDisplayP, JumpDetector &rDetectorP)
    : rDisplayM(rDisplayP)
    , rDetectorM(rDetectorP)
{
}

void DisplayController::renderCalibrationSplash(uint32_t remainingSecP)
{
    rDisplayM.clear();
    rDisplayM.drawString(10, 10, "CALIBRATING...");
    rDisplayM.drawString(5, 25, "Start jumping!");
    snprintf(lineBufferM, sizeof(lineBufferM), "%lu seconds", remainingSecP);
    rDisplayM.drawString(30, 40, lineBufferM);
    rDisplayM.commit();
}

void DisplayController::renderConfigPage(uint32_t const *pCountsP)
{
    rDisplayM.clear();
    rDisplayM.drawString(0, 0, "VERTICAL JUMPS:");
    for (int i = 0; i < NUM_TIMING_CONFIGS; i++)
    {
        uint32_t rise, fall;
        rDetectorM.getTimingConfig(i, rise, fall);
        snprintf(lineBufferM, sizeof(lineBufferM), "[%lums] %lu", rise, pCountsP[i]);
        rDisplayM.drawString(0, 12 + i * 12, lineBufferM);
    }
    rDisplayM.commit();
}

void DisplayController::renderTotalPage()
{
    rDisplayM.clear();
    rDisplayM.drawString(20, 0, "JUMP TOTAL");

    uint32_t totalZ;
    float rateZ;
    rDetectorM.getTotalJumps(totalZ);
    rDetectorM.getAverageRates(rateZ);

    snprintf(lineBufferM, sizeof(lineBufferM), "Jumps: %lu", totalZ);
    rDisplayM.drawString(15, 25, lineBufferM);
    snprintf(lineBufferM, sizeof(lineBufferM), "Rate:  %.0f/min", rateZ);
    rDisplayM.drawString(15, 42, lineBufferM);
    rDisplayM.commit();
}
