#pragma once

#include "display.h"
#include "jump.h"
#include <cstdint>

class DisplayController
{
public:
    explicit DisplayController(OledDisplay &rDisplayP, JumpDetector &rDetectorP);

    void renderCalibrationSplash(uint32_t remainingSecP);
    void renderConfigPage(uint32_t const *pCountsP);
    void renderTotalPage();

private:
    OledDisplay &rDisplayM;
    JumpDetector &rDetectorM;
    char lineBufferM[32];
};
