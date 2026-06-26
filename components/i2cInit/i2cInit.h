#pragma once

#include "driver/i2c.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <cstdint>

class I2CManager
{
public:
    static I2CManager& getInstance() noexcept;

    esp_err_t init() noexcept;

    i2c_port_t getPort() const noexcept
    {
        return i2cPortM;
    }
    SemaphoreHandle_t getMutex() const noexcept
    {
        return mutexM;
    }
    bool isInitialized() const noexcept
    {
        return initializedM;
    }

    I2CManager(const I2CManager&)            = delete;
    I2CManager& operator=(const I2CManager&) = delete;

private:
    I2CManager() noexcept;
    ~I2CManager() noexcept;

    static i2c_port_t constexpr DEFAULT_I2C_PORT   = I2C_NUM_0;
    static uint8_t constexpr DEFAULT_SDA_PIN       = 19;
    static uint8_t constexpr DEFAULT_SCL_PIN       = 20;
    static uint32_t constexpr DEFAULT_CLK_SPEED_HZ = 400'000;

    i2c_port_t i2cPortM;
    int sdaPinM;
    int sclPinM;
    uint32_t clkSpeedM;
    bool initializedM;
    SemaphoreHandle_t mutexM;
};
