#pragma once

#include "driver/i2c.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <cstdint>

class I2CManager {
public:
    // Singleton accessor
    static I2CManager& getInstance() noexcept;

    // Initialize the I2C bus
    esp_err_t init() noexcept;

    // Accessors
    i2c_port_t getPort() const noexcept { return _i2c_port; }
    SemaphoreHandle_t getMutex() const noexcept { return _mutex; }
    bool isInitialized() const noexcept { return _initialized; }

    // Delete copy/move
    I2CManager(const I2CManager&) = delete;
    I2CManager& operator=(const I2CManager&) = delete;

private:
    I2CManager() noexcept;
    ~I2CManager() noexcept;

    // Configuration constants
    static constexpr i2c_port_t DEFAULT_I2C_PORT = I2C_NUM_0;
    static constexpr uint8_t DEFAULT_SDA_PIN = 19;
    static constexpr uint8_t DEFAULT_SCL_PIN = 20;
    static constexpr uint32_t DEFAULT_CLK_SPEED_HZ = 400'000;

    // Internal state
    i2c_port_t _i2c_port;
    int _sda_pin;
    int _scl_pin;
    uint32_t _clk_speed;
    bool _initialized;
    SemaphoreHandle_t _mutex;
};