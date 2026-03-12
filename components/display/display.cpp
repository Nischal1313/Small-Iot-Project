#include "display.h"
#include "esp_log.h"
#include "font5x7.h"
#include "i2cInit.h"
#include "mutex.h"

static const char* TAG = "OLED_DISPLAY";

OledDisplay::OledDisplay()
    : _initialized(false),
      _i2c_address(DISPLAY_ADDR),
      _panel_io(nullptr),
      _panel(nullptr)
{
    clear();
    initializeDisplay();
}

OledDisplay::~OledDisplay() {
    cleanupResources();
}

void OledDisplay::initializeDisplay() {
    I2CManager& i2c_manager = I2CManager::getInstance();
    initSSD1306Driver();

    if (!i2c_manager.isInitialized()) {
        ESP_LOGE(TAG, "I2C Manager not initialized!");
        return;
    }

    ESP_LOGI(TAG, "Initializing OLED display at 0x%02X...", _i2c_address);

    {
        MutexGuard lock(i2c_manager.getMutex());
        ESP_LOGI(TAG, "[DISPLAY] Acquired I2C lock for initialization");

        esp_err_t result;

        // Panel IO configuration
        esp_lcd_panel_io_i2c_config_t panel_io_configuration = {};
        panel_io_configuration.dev_addr = _i2c_address;
        panel_io_configuration.control_phase_bytes = 1;
        panel_io_configuration.lcd_cmd_bits = 8;
        panel_io_configuration.lcd_param_bits = 8;

        result = esp_lcd_new_panel_io_i2c(i2c_manager.getPort(),
                                          &panel_io_configuration,
                                          &_panel_io);
        if (result != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create panel IO: %s", esp_err_to_name(result));
            return;
        }

        // Panel configuration
        esp_lcd_panel_dev_config_t panel_configuration = {};
        panel_configuration.bits_per_pixel = 1;
        panel_configuration.reset_gpio_num = -1;

        result = esp_lcd_new_panel_ssd1306(_panel_io, &panel_configuration, &_panel);
        if (result != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create SSD1306 panel: %s", esp_err_to_name(result));
            if (_panel_io) {
                esp_lcd_panel_io_del(_panel_io);
                _panel_io = nullptr;
            }
            return;
        }

        // Reset and initialize panel
        if (esp_lcd_panel_reset(_panel) != ESP_OK ||
            esp_lcd_panel_init(_panel) != ESP_OK) {
            ESP_LOGE(TAG, "Panel reset/init failed");
            cleanupResources();
            return;
        }

        // Turn on display
        if (esp_lcd_panel_disp_on_off(_panel, true) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to turn on display");
            cleanupResources();
            return;
        }

        clear();
        commit();

        ESP_LOGI(TAG, "[DISPLAY] Display initialized successfully");
    }

    _initialized = true;
}

void OledDisplay::cleanupResources() {
    if (_panel) {
        esp_lcd_panel_del(_panel);
        _panel = nullptr;
    }
    if (_panel_io) {
        esp_lcd_panel_io_del(_panel_io);
        _panel_io = nullptr;
    }
}

void OledDisplay::clear() {
    std::memset(_framebuffer, 0x00, sizeof(_framebuffer));
}

void OledDisplay::commit() {
    if (!_initialized) return;

    uint8_t column_command[] = {0x21, 0, uint8_t(DISPLAY_WIDTH - 1)};
    uint8_t page_command[] = {0x22, 0, uint8_t(DISPLAY_HEIGHT / 8 - 1)};

    sendCommand(column_command[0]);
    sendCommand(column_command[1]);
    sendCommand(column_command[2]);
    sendCommand(page_command[0]);
    sendCommand(page_command[1]);
    sendCommand(page_command[2]);

    uint8_t buffer[DISPLAY_WIDTH * DISPLAY_HEIGHT / 8 + 1];
    buffer[0] = 0x40;
    std::memcpy(buffer + 1, _framebuffer, DISPLAY_WIDTH * DISPLAY_HEIGHT / 8);

    I2CManager& i2c_manager = I2CManager::getInstance();
    MutexGuard lock(i2c_manager.getMutex());
    esp_err_t result = i2c_master_write_to_device(i2c_manager.getPort(),
                                                  _i2c_address,
                                                  buffer,
                                                  sizeof(buffer),
                                                  pdMS_TO_TICKS(1000));
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "I2C write failed: %s", esp_err_to_name(result));
    }
}

void OledDisplay::sendCommand(uint8_t command) {
    uint8_t buffer[2] = {0x80, command};

    I2CManager& i2c_manager = I2CManager::getInstance();
    MutexGuard lock(i2c_manager.getMutex());
    esp_err_t result = i2c_master_write_to_device(i2c_manager.getPort(),
                                                  _i2c_address,
                                                  buffer,
                                                  sizeof(buffer),
                                                  pdMS_TO_TICKS(1000));
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "I2C write failed: %s", esp_err_to_name(result));
    }
}

void OledDisplay::initSSD1306Driver() {
    // Initialization sequence per SSD1306 datasheet
    sendCommand(0xAE); // Display off
    sendCommand(0x20); sendCommand(0x00); // Memory mode = horizontal
    sendCommand(0x40); // Start line
    sendCommand(0xA1); // Segment remap
    sendCommand(0xC0); // COM scan direction
    sendCommand(0xA8); sendCommand(DISPLAY_HEIGHT - 1); // Multiplex ratio
    sendCommand(0xD3); sendCommand(0x00); // Display offset
    sendCommand(0xDA); sendCommand(0x12); // COM pins
    sendCommand(0x81); sendCommand(0xFF); // Contrast
    sendCommand(0xD9); sendCommand(0xF1); // Pre-charge
    sendCommand(0xDB); sendCommand(0x30); // VCOMH deselect
    sendCommand(0xA4); // Entire display on follow RAM
    sendCommand(0xA6); // Normal display
    sendCommand(0x8D); sendCommand(0x14); // Charge pump
    sendCommand(0xAF); // Display on
}

void OledDisplay::drawChar(int position_x, int position_y, char character) {
    if (character > 127) return;

    const uint8_t* glyph = reinterpret_cast<const uint8_t*>(font8x8_basic[uint8_t(character)]);

    for (int row = 0; row < FONT_HEIGHT; ++row) {
        uint8_t row_bits = glyph[row];

        // Horizontal bit reverse
        row_bits = ((row_bits & 0xF0) >> 4) | ((row_bits & 0x0F) << 4);
        row_bits = ((row_bits & 0xCC) >> 2) | ((row_bits & 0x33) << 2);
        row_bits = ((row_bits & 0xAA) >> 1) | ((row_bits & 0x55) << 1);

        for (int col = 0; col < FONT_WIDTH; ++col) {
            if (row_bits & (1 << (7 - col))) {
                int pixel_x = position_x + col;
                int pixel_y = position_y + row;
                if (pixel_x < 0 || pixel_x >= DISPLAY_WIDTH || pixel_y < 0 || pixel_y >= DISPLAY_HEIGHT)
                    continue;

                int index = pixel_x + (pixel_y / 8) * DISPLAY_WIDTH;
                _framebuffer[index] |= (1 << (pixel_y % 8));
            }
        }
    }
}

void OledDisplay::drawString(int position_x, int position_y, const char* text) {
    int cursor_x = position_x;
    while (*text) {
        drawChar(cursor_x, position_y, *text);
        cursor_x += FONT_WIDTH;
        ++text;
    }
}