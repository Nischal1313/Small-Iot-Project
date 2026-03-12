#include "i2cInit.h"
#include "esp_log.h"
#include <cassert>

static constexpr const char* TAG = "I2C_MANAGER";

I2CManager::I2CManager() noexcept
    : _i2c_port(DEFAULT_I2C_PORT),
      _sda_pin(DEFAULT_SDA_PIN),
      _scl_pin(DEFAULT_SCL_PIN),
      _clk_speed(DEFAULT_CLK_SPEED_HZ),
      _initialized(false),
      _mutex(nullptr) {}

I2CManager::~I2CManager() noexcept {
    if (_mutex) {
        vSemaphoreDelete(_mutex);
        _mutex = nullptr;
    }
}

I2CManager& I2CManager::getInstance() noexcept {
    static I2CManager instance;
    return instance;
}

esp_err_t I2CManager::init() noexcept {
    if (_initialized) {
        ESP_LOGW(TAG, "I2C already initialized");
        return ESP_OK;
    }

    _mutex = xSemaphoreCreateMutex();
    if (!_mutex) {
        ESP_LOGE(TAG, "Failed to create I2C mutex");
        return ESP_FAIL;
    }

    i2c_config_t config = {};
    config.mode = I2C_MODE_MASTER;
    config.sda_io_num = _sda_pin;
    config.scl_io_num = _scl_pin;
    config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    config.master.clk_speed = _clk_speed;

    esp_err_t ret = i2c_param_config(_i2c_port, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(_i2c_port, config.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    _initialized = true;
    ESP_LOGI(TAG,
             "I2C initialized on port %d (SDA=%d, SCL=%d, Speed=%u Hz)",
             _i2c_port, _sda_pin, _scl_pin, _clk_speed);

    return ESP_OK;
}