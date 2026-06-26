#include "gpio_pin.h"
#include <cstdio>

GPIOPin::GPIOPin(gpio_num_t pinP, GPIOMode modeP, GPIOPull pullP, bool invertP, uint32_t debounceMsP)
    : pinNumberM{pinP}
    , modeM{modeP}
    , pullM{pullP}
    , isInvertedM{invertP}
    , lastReadingM{false}
    , stableStateM{false}
    , pressEventM{false}
    , holdEventM{false}
    , debounceMsM{debounceMsP}
    , holdMsM{1000}
{
    gpio_config_t cfg{};
    cfg.pin_bit_mask  = (1ULL << pinP);
    cfg.intr_type     = GPIO_INTR_DISABLE;

    if (modeP == GPIOMode::OUTPUT)
    {
        cfg.mode         = GPIO_MODE_OUTPUT;
        cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
    }
    else
    {
        cfg.mode = GPIO_MODE_INPUT;
        cfg.pull_up_en   = (pullP == GPIOPull::PULLUP) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
        cfg.pull_down_en = (pullP == GPIOPull::PULLDOWN) ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
    }

    gpio_config(&cfg);

    lastChangeTimeM = esp_timer_get_time();
    pressStartTimeM = esp_timer_get_time();
}

bool GPIOPin::read() const
{
    bool result{false};

    if (modeM == GPIOMode::INPUT)
    {
        bool val = gpio_get_level(pinNumberM);
        result   = isInvertedM ? !val : val;
    }
    else
    {
        printf("[GPIO WARNING] Tried to read OUTPUT pin %d\n", pinNumberM);
    }

    return result;
}

void GPIOPin::write(bool valueP) const
{
    if (modeM == GPIOMode::OUTPUT)
    {
        gpio_set_level(pinNumberM, isInvertedM ? !valueP : valueP);
    }
    else
    {
        printf("[GPIO WARNING] Tried to write INPUT pin %d\n", pinNumberM);
    }
}

int GPIOPin::getPin() const
{
    return pinNumberM;
}

void GPIOPin::update()
{
    if (modeM != GPIOMode::INPUT)
    {
        return;
    }

    bool reading = gpio_get_level(pinNumberM);
    if (isInvertedM)
    {
        reading = !reading;
    }

    int64_t const now = esp_timer_get_time();

    if (reading != lastReadingM)
    {
        lastChangeTimeM = now;
    }

    if ((now - lastChangeTimeM) > static_cast<int64_t>(debounceMsM) * 1000)
    {
        if (reading != stableStateM)
        {
            stableStateM = reading;

            if (stableStateM)
            {
                pressEventM     = true;
                pressStartTimeM = now;
                holdEventM      = false;
            }
            else
            {
                holdEventM = false;
            }
        }
    }

    if (stableStateM && !holdEventM && (now - pressStartTimeM) > static_cast<int64_t>(holdMsM) * 1000)
    {
        holdEventM = true;
    }

    lastReadingM = reading;
}

bool GPIOPin::pressed()
{
    bool result{false};

    if (pressEventM)
    {
        pressEventM = false;
        result      = true;
    }

    return result;
}

bool GPIOPin::held()
{
    bool result{false};

    if (holdEventM)
    {
        holdEventM = false;
        result     = true;
    }

    return result;
}

void GPIOPin::setHoldTime(uint32_t msP)
{
    holdMsM = msP;
}

void GPIOPin::setDebounceTime(uint32_t msP)
{
    debounceMsM = msP;
}
