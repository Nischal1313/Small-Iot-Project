#include "i2cInit.h"
#include "esp_log.h"
#include <cassert>

namespace
{

char const *TAG_S = "I2C_MANAGER";

}  // namespace

I2CManager::I2CManager() noexcept
    : i2cPortM{DEFAULT_I2C_PORT}
    , sdaPinM{DEFAULT_SDA_PIN}
    , sclPinM{DEFAULT_SCL_PIN}
    , clkSpeedM{DEFAULT_CLK_SPEED_HZ}
    , initializedM{false}
    , mutexM{nullptr}
{
}

I2CManager::~I2CManager() noexcept
{
    if (mutexM != nullptr)
    {
        vSemaphoreDelete(mutexM);
        mutexM = nullptr;
    }
}

I2CManager& I2CManager::getInstance() noexcept
{
    static I2CManager instance;
    return instance;
}

esp_err_t I2CManager::init() noexcept
{
    if (initializedM)
    {
        ESP_LOGW(TAG_S, "I2C already initialized");
        return ESP_OK;
    }

    esp_err_t result{ESP_FAIL};

    mutexM = xSemaphoreCreateMutex();
    if (mutexM != nullptr)
    {
        i2c_config_t config{};
        config.mode             = I2C_MODE_MASTER;
        config.sda_io_num       = sdaPinM;
        config.scl_io_num       = sclPinM;
        config.sda_pullup_en    = GPIO_PULLUP_ENABLE;
        config.scl_pullup_en    = GPIO_PULLUP_ENABLE;
        config.master.clk_speed = clkSpeedM;

        result = i2c_param_config(i2cPortM, &config);
        if (result == ESP_OK)
        {
            result = i2c_driver_install(i2cPortM, config.mode, 0, 0, 0);
            if (result == ESP_OK)
            {
                initializedM = true;
                ESP_LOGI(
                    TAG_S,
                    "I2C initialized on port %d (SDA=%d, SCL=%d, Speed=%u Hz)",
                    i2cPortM,
                    sdaPinM,
                    sclPinM,
                    clkSpeedM
                );
            }
            else
            {
                ESP_LOGE(TAG_S, "I2C driver install failed: %s", esp_err_to_name(result));
            }
        }
        else
        {
            ESP_LOGE(TAG_S, "I2C param config failed: %s", esp_err_to_name(result));
        }
    }
    else
    {
        ESP_LOGE(TAG_S, "Failed to create I2C mutex");
    }

    return result;
}
