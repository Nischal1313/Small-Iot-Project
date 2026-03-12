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

class OledDisplay {
public:
    OledDisplay();               // Constructor auto-initializes
    ~OledDisplay();              // Destructor cleans up resources

    void clear();                // Clears the framebuffer
    void commit();               // Sends framebuffer to the display

    void drawChar(int position_x, int position_y, char character);
    void drawString(int position_x, int position_y, const char* text);

    bool isInitialized() const { return _initialized; }

private:
    void sendCommand(uint8_t command);
    void initSSD1306Driver();
    void initializeDisplay();
    void cleanupResources();

    bool _initialized;
    const uint8_t _i2c_address;

    esp_lcd_panel_io_handle_t _panel_io;
    esp_lcd_panel_handle_t _panel;

    static constexpr uint8_t DISPLAY_WIDTH = 128;
    static constexpr uint8_t DISPLAY_HEIGHT = 64;
    static constexpr uint8_t FONT_WIDTH = 8;
    static constexpr uint8_t FONT_HEIGHT = 8;
    static constexpr uint8_t DISPLAY_ADDR = 0x3C;

    uint8_t _framebuffer[DISPLAY_WIDTH * DISPLAY_HEIGHT / 8];
};