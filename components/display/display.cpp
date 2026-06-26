#include "display.h"
#include "esp_log.h"
#include "font5x7.h"
#include "i2cInit.h"
#include "mutex.h"

namespace
{
    char const *TAG_S = "OLED_DISPLAY";
}

OledDisplay::OledDisplay()
    : initializedM{false}
    , i2cAddressM{DISPLAY_ADDR}
    , pPanelIoM{nullptr}
    , pPanelM{nullptr}
{
    clear();
    initializeDisplay();
}

OledDisplay::~OledDisplay()
{
    cleanupResources();
}

void OledDisplay::initializeDisplay()
{
    bool result{false};

    I2CManager &rI2cManager = I2CManager::getInstance();

    if (!rI2cManager.isInitialized())
    {
        ESP_LOGE(TAG_S, "I2C Manager not initialized!");
    }
    else
    {
        initSSD1306Driver();

        ESP_LOGI(TAG_S, "Initializing OLED display at 0x%02X...", i2cAddressM);

        {
            MutexGuard lock(rI2cManager.getMutex());
            ESP_LOGI(TAG_S, "[DISPLAY] Acquired I2C lock for initialization");

            esp_lcd_panel_io_i2c_config_t panelIoConfiguration{};
            panelIoConfiguration.dev_addr            = i2cAddressM;
            panelIoConfiguration.control_phase_bytes = 1;
            panelIoConfiguration.lcd_cmd_bits        = 8;
            panelIoConfiguration.lcd_param_bits      = 8;

            esp_err_t err{
                esp_lcd_new_panel_io_i2c(rI2cManager.getPort(), &panelIoConfiguration, &pPanelIoM)
            };

            if (err == ESP_OK)
            {
                esp_lcd_panel_dev_config_t panelConfiguration{};
                panelConfiguration.bits_per_pixel   = 1;
                panelConfiguration.reset_gpio_num   = -1;

                err = esp_lcd_new_panel_ssd1306(pPanelIoM, &panelConfiguration, &pPanelM);
                if (err == ESP_OK)
                {
                    if (esp_lcd_panel_reset(pPanelM) == ESP_OK &&
                        esp_lcd_panel_init(pPanelM) == ESP_OK)
                    {
                        if (esp_lcd_panel_disp_on_off(pPanelM, true) == ESP_OK)
                        {
                            clear();
                            commit();
                            ESP_LOGI(TAG_S, "[DISPLAY] Display initialized successfully");
                            result = true;
                        }
                        else
                        {
                            ESP_LOGE(TAG_S, "Failed to turn on display");
                            cleanupResources();
                        }
                    }
                    else
                    {
                        ESP_LOGE(TAG_S, "Panel reset/init failed");
                        cleanupResources();
                    }
                }
                else
                {
                    ESP_LOGE(TAG_S, "Failed to create SSD1306 panel: %s", esp_err_to_name(err));
                    if (pPanelIoM)
                    {
                        esp_lcd_panel_io_del(pPanelIoM);
                        pPanelIoM = nullptr;
                    }
                }
            }
            else
            {
                ESP_LOGE(TAG_S, "Failed to create panel IO: %s", esp_err_to_name(err));
            }
        }
    }

    if (result)
    {
        initializedM = true;
    }
}

void OledDisplay::cleanupResources()
{
    if (pPanelM)
    {
        esp_lcd_panel_del(pPanelM);
        pPanelM = nullptr;
    }
    if (pPanelIoM)
    {
        esp_lcd_panel_io_del(pPanelIoM);
        pPanelIoM = nullptr;
    }
}

void OledDisplay::clear()
{
    std::memset(framebufferM, 0x00, sizeof(framebufferM));
}

void OledDisplay::commit()
{
    if (initializedM)
    {
        uint8_t columnCommand[]{0x21, 0, uint8_t(DISPLAY_WIDTH - 1)};
        uint8_t pageCommand[]{0x22, 0, uint8_t(DISPLAY_HEIGHT / 8 - 1)};

        sendCommand(columnCommand[0]);
        sendCommand(columnCommand[1]);
        sendCommand(columnCommand[2]);
        sendCommand(pageCommand[0]);
        sendCommand(pageCommand[1]);
        sendCommand(pageCommand[2]);

        uint8_t buffer[DISPLAY_WIDTH * DISPLAY_HEIGHT / 8 + 1];
        buffer[0] = 0x40;
        std::memcpy(buffer + 1, framebufferM, DISPLAY_WIDTH * DISPLAY_HEIGHT / 8);

        I2CManager &rI2cManager = I2CManager::getInstance();
        MutexGuard lock(rI2cManager.getMutex());
        esp_err_t result = i2c_master_write_to_device(
            rI2cManager.getPort(), i2cAddressM, buffer, sizeof(buffer), pdMS_TO_TICKS(1000)
        );
        if (result != ESP_OK)
        {
            ESP_LOGE(TAG_S, "I2C write failed: %s", esp_err_to_name(result));
        }
    }
}

void OledDisplay::sendCommand(uint8_t commandP)
{
    uint8_t buffer[2] = {0x80, commandP};

    I2CManager &rI2cManager = I2CManager::getInstance();
    MutexGuard lock(rI2cManager.getMutex());
    esp_err_t result = i2c_master_write_to_device(
        rI2cManager.getPort(), i2cAddressM, buffer, sizeof(buffer), pdMS_TO_TICKS(1000)
    );
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG_S, "I2C write failed: %s", esp_err_to_name(result));
    }
}

void OledDisplay::initSSD1306Driver()
{
    sendCommand(0xAE);
    sendCommand(0x20);
    sendCommand(0x00);
    sendCommand(0x40);
    sendCommand(0xA1);
    sendCommand(0xC0);
    sendCommand(0xA8);
    sendCommand(DISPLAY_HEIGHT - 1);
    sendCommand(0xD3);
    sendCommand(0x00);
    sendCommand(0xDA);
    sendCommand(0x12);
    sendCommand(0x81);
    sendCommand(0xFF);
    sendCommand(0xD9);
    sendCommand(0xF1);
    sendCommand(0xDB);
    sendCommand(0x30);
    sendCommand(0xA4);
    sendCommand(0xA6);
    sendCommand(0x8D);
    sendCommand(0x14);
    sendCommand(0xAF);
}

void OledDisplay::drawChar(int positionXP, int positionYP, char characterP)
{
    if (characterP <= 127)
    {
        uint8_t const *pGlyph = reinterpret_cast<uint8_t const *>(
            font8x8_basic[uint8_t(characterP)]
        );

        for (int row = 0; row < FONT_HEIGHT; ++row)
        {
            uint8_t rowBits = pGlyph[row];

            rowBits = ((rowBits & 0xF0) >> 4) | ((rowBits & 0x0F) << 4);
            rowBits = ((rowBits & 0xCC) >> 2) | ((rowBits & 0x33) << 2);
            rowBits = ((rowBits & 0xAA) >> 1) | ((rowBits & 0x55) << 1);

            for (int col = 0; col < FONT_WIDTH; ++col)
            {
                if (rowBits & (1 << (7 - col)))
                {
                    int pixelX = positionXP + col;
                    int pixelY = positionYP + row;
                    if (pixelX >= 0 && pixelX < DISPLAY_WIDTH &&
                        pixelY >= 0 && pixelY < DISPLAY_HEIGHT)
                    {
                        int index = pixelX + (pixelY / 8) * DISPLAY_WIDTH;
                        framebufferM[index] |= (1 << (pixelY % 8));
                    }
                }
            }
        }
    }
}

void OledDisplay::drawString(int positionXP, int positionYP, char const *pTextP)
{
    int cursorX{positionXP};
    while (*pTextP)
    {
        drawChar(cursorX, positionYP, *pTextP);
        cursorX += FONT_WIDTH;
        ++pTextP;
    }
}
