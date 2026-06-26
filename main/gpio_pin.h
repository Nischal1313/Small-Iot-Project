#pragma once

#include "driver/gpio.h"
#include "esp_timer.h"

enum class GPIOMode
{
    INPUT,
    OUTPUT
};

enum class GPIOPull
{
    NONE,
    PULLUP,
    PULLDOWN
};

class GPIOPin
{
public:
    explicit GPIOPin(
        gpio_num_t pinP, GPIOMode modeP = GPIOMode::INPUT, GPIOPull pullP = GPIOPull::PULLUP,
        bool invertP = false, uint32_t debounceMsP = 50
    );

    bool read() const;
    void write(bool valueP) const;
    int getPin() const;

    void update();
    bool pressed();
    bool held();
    void setHoldTime(uint32_t msP);
    void setDebounceTime(uint32_t msP);

private:
    gpio_num_t pinNumberM;
    GPIOMode modeM;
    GPIOPull pullM;
    bool isInvertedM;

    bool lastReadingM;
    bool stableStateM;
    bool pressEventM;
    bool holdEventM;

    int64_t lastChangeTimeM;
    int64_t pressStartTimeM;
    uint32_t debounceMsM;
    uint32_t holdMsM;
};
