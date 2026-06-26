#pragma once

#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_lcd_io_i2c.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_ssd1306.h"
#include "esp_lcd_panel_vendor.h"
#include <cstdint>
#include <cstring>

class OledDisplay
{
public:
    OledDisplay();
    ~OledDisplay();

    void clear();
    void commit();

    void drawChar(int positionXP, int positionYP, char characterP);
    void drawString(int positionXP, int positionYP, char const *pTextP);

    bool isInitialized() const
    {
        return initializedM;
    }

private:
    void sendCommand(uint8_t commandP);
    void initSSD1306Driver();
    void initializeDisplay();
    void cleanupResources();

    bool initializedM;
    uint8_t const i2cAddressM;

    esp_lcd_panel_io_handle_t pPanelIoM;
    esp_lcd_panel_handle_t pPanelM;

    static uint8_t constexpr DISPLAY_WIDTH  = 128;
    static uint8_t constexpr DISPLAY_HEIGHT = 64;
    static uint8_t constexpr FONT_WIDTH     = 8;
    static uint8_t constexpr FONT_HEIGHT    = 8;
    static uint8_t constexpr DISPLAY_ADDR   = 0x3C;

    uint8_t framebufferM[DISPLAY_WIDTH * DISPLAY_HEIGHT / 8];
};
