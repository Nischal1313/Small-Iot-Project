#pragma once

#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_lcd_io_i2c.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_ssd1306.h"
#include "esp_lcd_panel_vendor.h"
#include <cstdint>

constexpr uint8_t DISPLAY_ADDR = 0x3C;

class OledDisplay {
public:
  OledDisplay(); // Auto-initializes on construction

  void clear();
  void commit();

  void drawChar(int x, int y, char c);
  bool isInitialized() const { return _initialized; }

  private:
    void sendCommand(uint8_t cmd);
    void const initSSD1306();
    void init();
    void cleanup();

    bool _initialized;
    const uint8_t _i2c_addr;

    esp_lcd_panel_io_handle_t _io;
    esp_lcd_panel_handle_t _panel;

    static constexpr int WIDTH = 128;
    static constexpr int HEIGHT = 64;
    static constexpr uint8_t FONT_WIDTH = 8;
    static constexpr uint8_t FONT_HEIGHT = FONT_WIDTH;

    uint8_t _framebuffer[WIDTH * HEIGHT / 8];
  };
