#include "algorithm_by_RF.h"
#include "display.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "i2cInit.h"
#include "jr_ble.h"
#include "jump.h"
#include "max30102.h"
#include "mutex.h"
#include <cstdint>
#include <cstdio>
#include <inttypes.h>

// Jump detector tuning
constexpr float    JUMP_THRESHOLD_FACTOR = 1.3f;
constexpr uint32_t MIN_JUMP_INTERVAL_MS  = 300;   
constexpr int      JUMP_UPDATE_HZ        = 100;
constexpr int      DISPLAY_UPDATE_HZ     = 4;
constexpr uint32_t CALIBRATION_TIME_MS   = 3000;  
// SpO2 / HR
constexpr int SPO2_BUFFER_SIZE = 100;         
constexpr int SPO2_SAMPLE_HZ   = 100;
constexpr int BLE_JUMP_CONFIG_INDEX = 2;

static const char *TAG = "JUMP_TEST";

static OledDisplay   *display       = nullptr;
static JumpDetector  *accelDetector = nullptr;
static SensorReading *sensor        = nullptr;

// Protects accelDetector, accelCountsZ, and the calibration flags
static SemaphoreHandle_t dataMutex = nullptr;

// Calibration state — only ever written inside dataMutex
static uint32_t calibrationStartTime = 0;
static bool     calibrationPhase     = true;

// Jump count snapshot for the display — written + read under dataMutex
static uint32_t accelCountsZ[NUM_TIMING_CONFIGS];

void jumpDetectionTask(void *param) {
  ESP_LOGI(TAG, "Jump detection task started");
  bool wasStreaming = false;

  while (true) {
    const bool isStreaming = jr_ble_is_streaming();

    {
      MutexGuard lock(dataMutex);

      // New BLE workout session started: reset counts and restart calibration
      if (isStreaming && !wasStreaming) {
        accelDetector->resetSession();
        calibrationStartTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
        calibrationPhase     = true;
        ESP_LOGI(TAG, "BLE stream started — session reset");
      }

      // Drive the detector (drains the gyro queue, updates state machines)
      accelDetector->update();

      // Once enough jumps have been seen the detector self-calibrates;
      // mirror that into our display-side calibration flag.
      if (calibrationPhase && accelDetector->isCalibrated()) {
        calibrationPhase = false;
        ESP_LOGI(TAG, "Jump detector calibrated");
      }
    }

    wasStreaming = isStreaming;
    vTaskDelay(pdMS_TO_TICKS(1000 / JUMP_UPDATE_HZ));
  }
}

void displayTask(void *param) {
  ESP_LOGI(TAG, "Display task started");

  constexpr uint8_t  NUM_PAGES       = 2;   // update this when HR page is re-enabled
  constexpr uint16_t  PAGE_DWELL_MS   = 3000;
  int      displayPage    = 0;
  uint32_t lastPageChange = 0;

  while (true) {
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // ── Calibration splash ──────────────────────────────────────────────────
    bool inCalibration;
    uint32_t calStart;
    {
      MutexGuard lock(dataMutex);
      inCalibration = calibrationPhase;
      calStart      = calibrationStartTime;
    }

    if (inCalibration) {
      uint32_t elapsed   = now - calStart;
      uint32_t remaining = (elapsed < CALIBRATION_TIME_MS)
                               ? (CALIBRATION_TIME_MS - elapsed) / 1000
                               : 0;
      char buf[32];
      display->clear();
      display->drawString(10, 10, "CALIBRATING...");
      display->drawString(5,  25, "Start jumping!");
      snprintf(buf, sizeof(buf), "%lu seconds", remaining + 1);
      display->drawString(30, 40, buf);
      display->commit();
      vTaskDelay(pdMS_TO_TICKS(1000 / DISPLAY_UPDATE_HZ));
      continue;
    }
    {
      MutexGuard lock(dataMutex);
      accelDetector->getCounts(accelCountsZ);
    }

    if (now - lastPageChange > PAGE_DWELL_MS) {
      displayPage     = (displayPage + 1) % NUM_PAGES;
      lastPageChange  = now;
    }
    display->clear();
    char line[32];

    if (displayPage == 0) {
      display->drawString(0, 0, "VERTICAL JUMPS:");
      for (int i = 0; i < NUM_TIMING_CONFIGS; i++) {
        uint32_t rise, fall;
        accelDetector->getTimingConfig(i, rise, fall);
        display->drawString(0, 12 + i * 12, line);
      }

    } else { // displayPage == 1
      display->drawString(20, 0, "JUMP TOTAL");
      uint32_t totalZ;
      float    rateZ;
      {
        MutexGuard lock(dataMutex);
        accelDetector->getTotalJumps(totalZ);
        accelDetector->getAverageRates(rateZ);
      }
      snprintf(line, sizeof(line), "Jumps: %lu",    totalZ);
      display->drawString(15, 25, line);
      snprintf(line, sizeof(line), "Rate:  %.0f/min", rateZ);
      display->drawString(15, 42, line);
    }
    display->commit();
    vTaskDelay(pdMS_TO_TICKS(1000 / DISPLAY_UPDATE_HZ));
  }
}

void bleUpdateTask(void *param) {
  while (true) {
    uint32_t selectedJumpCount;
    {
      MutexGuard lock(dataMutex);
      accelDetector->getCounts(accelCountsZ);
      selectedJumpCount = accelCountsZ[BLE_JUMP_CONFIG_INDEX];
    }
    jr_ble_set_sensor_snapshot(selectedJumpCount, 1, 2, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

/* =============================================================================
   INIT TASK
   ============================================================================= */
void initTask(void *param) {
  ESP_LOGI(TAG, "=== Vertical Jump Monitor ===");

  dataMutex = xSemaphoreCreateMutex();
  configASSERT(dataMutex != nullptr);

  I2CManager &i2c = I2CManager::getInstance();
  i2c.init();
  configASSERT(i2c.isInitialized());
  vTaskDelay(pdMS_TO_TICKS(100));

  jr_ble_init();

  display = new OledDisplay();
  display->clear();
  display->drawString(15, 20, "Initializing");
  display->drawString(30, 35, "Sensors...");
  display->commit();
  vTaskDelay(pdMS_TO_TICKS(1500));

  // Sensor starts its own FreeRTOS task that pushes mpu_data_t to a queue
  // at 100 Hz. The jump detector drains that queue in jumpDetectionTask.
  sensor = new SensorReading();
  sensor->startTask();
  vTaskDelay(pdMS_TO_TICKS(100)); // let gyro task seed the queue

  accelDetector = new JumpDetector(sensor, JUMP_THRESHOLD_FACTOR,
                                   MIN_JUMP_INTERVAL_MS);

  calibrationStartTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
  calibrationPhase     = true;

  xTaskCreate(jumpDetectionTask, "jump_task",    4096, nullptr, 5, nullptr);
  xTaskCreate(displayTask,       "display_task", 3072, nullptr, 4, nullptr);
  xTaskCreate(bleUpdateTask,     "ble_task",     3072, nullptr, 3, nullptr);
  // xTaskCreate(heartRateTask,  "hr_task",      3072, nullptr, 3, nullptr);

  ESP_LOGI(TAG, "All tasks started");
  vTaskDelete(nullptr);
}

/* =============================================================================
   APP MAIN
   ============================================================================= */
extern "C" void app_main(void) {
  ESP_LOGI(TAG, "Jump Rope Monitor starting...");
  xTaskCreate(initTask, "init_task", 4096, nullptr, 6, nullptr);
}