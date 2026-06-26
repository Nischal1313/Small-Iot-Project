#include "display_controller.h"
#include "display.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "i2cInit.h"
#include "jr_ble.h"
#include "jump.h"
#include "mutex.h"
#include <cstdint>
#include <memory>

namespace
{

enum class SystemState : uint8_t
{
    Calibrating,
    Running
};

constexpr bool isValidTransition(SystemState fromP, SystemState toP)
{
    switch (fromP)
    {
        case SystemState::Calibrating:
            return toP == SystemState::Running;
        case SystemState::Running:
            return toP == SystemState::Calibrating;
    }

    return false;
}

constexpr float JUMP_THRESHOLD_FACTOR   = 1.3f;
constexpr uint32_t MIN_JUMP_INTERVAL_MS = 300;
constexpr int JUMP_UPDATE_HZ            = 100;
constexpr int DISPLAY_UPDATE_HZ         = 4;
constexpr uint32_t CALIBRATION_TIME_MS  = 3000;
constexpr int BLE_JUMP_CONFIG_INDEX     = 2;

char const* TAG_S = "JUMP_TEST";
std::unique_ptr<OledDisplay> pDisplayS;
std::unique_ptr<JumpDetector> pAccelDetectorS;
std::unique_ptr<SensorReading> pSensorS;
SemaphoreHandle_t pDataMutexS  = nullptr;
uint32_t calibrationStartTimeS = 0;
SystemState systemStateS       = SystemState::Calibrating;
uint32_t accelCountsZS[NUM_TIMING_CONFIGS];

}  // namespace

void jumpDetectionTask(void* pParamP)
{
    ESP_LOGI(TAG_S, "Jump detection task started");
    bool wasStreaming = false;

    while (true)
    {
        bool const isStreaming = jrBleIsStreaming();

        {
            MutexGuard lock(pDataMutexS);

            if (isStreaming && !wasStreaming)
            {
                pAccelDetectorS->resetSession();
                calibrationStartTimeS  = static_cast<uint32_t>(esp_timer_get_time() / 1000);
                SystemState const prev = systemStateS;
                systemStateS           = SystemState::Calibrating;
                configASSERT(isValidTransition(prev, systemStateS));
                ESP_LOGI(TAG_S, "BLE stream started — session reset");
            }

            pAccelDetectorS->update();

            if (systemStateS == SystemState::Calibrating && pAccelDetectorS->isCalibrated())
            {
                SystemState const prev = systemStateS;
                systemStateS           = SystemState::Running;
                configASSERT(isValidTransition(prev, systemStateS));
                ESP_LOGI(TAG_S, "Jump detector calibrated");
            }
        }

        wasStreaming = isStreaming;
        vTaskDelay(pdMS_TO_TICKS(1000 / JUMP_UPDATE_HZ));
    }
}

void displayTask(void* pParamP)
{
    ESP_LOGI(TAG_S, "Display task started");
    auto* pController = static_cast<DisplayController*>(pParamP);

    constexpr uint8_t NUM_PAGES      = 2;
    constexpr uint16_t PAGE_DWELL_MS = 3000;
    int displayPage                  = 0;
    uint32_t lastPageChange          = 0;

    while (true)
    {
        uint32_t now      = static_cast<uint32_t>(esp_timer_get_time() / 1000);
        SystemState st    = SystemState::Calibrating;
        uint32_t calStart = 0;

        {
            MutexGuard lock(pDataMutexS);
            st       = systemStateS;
            calStart = calibrationStartTimeS;
            pAccelDetectorS->getCounts(accelCountsZS);
        }

        if (st == SystemState::Calibrating)
        {
            uint32_t remaining = (now - calStart < CALIBRATION_TIME_MS)
                                     ? (CALIBRATION_TIME_MS - (now - calStart)) / 1000 + 1
                                     : 0;
            pController->renderCalibrationSplash(remaining);
            vTaskDelay(pdMS_TO_TICKS(1000 / DISPLAY_UPDATE_HZ));
            continue;
        }

        configASSERT(st == SystemState::Running);

        if (now - lastPageChange > PAGE_DWELL_MS)
        {
            displayPage    = (displayPage + 1) % NUM_PAGES;
            lastPageChange = now;
        }

        if (displayPage == 0)
        {
            pController->renderConfigPage(accelCountsZS);
        }
        else
        {
            pController->renderTotalPage();
        }

        vTaskDelay(pdMS_TO_TICKS(1000 / DISPLAY_UPDATE_HZ));
    }
}

void bleUpdateTask(void* pParamP)
{
    while (true)
    {
        uint32_t selectedJumpCount;
        {
            MutexGuard lock(pDataMutexS);
            pAccelDetectorS->getCounts(accelCountsZS);
            selectedJumpCount = accelCountsZS[BLE_JUMP_CONFIG_INDEX];
        }
        jrBleSetSensorSnapshot(selectedJumpCount, 0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void initTask(void* pParamP)
{
    ESP_LOGI(TAG_S, "=== Vertical Jump Monitor ===");

    pDataMutexS = xSemaphoreCreateMutex();
    configASSERT(pDataMutexS != nullptr);

    I2CManager& rI2c = I2CManager::getInstance();
    rI2c.init();
    configASSERT(rI2c.isInitialized());
    vTaskDelay(pdMS_TO_TICKS(100));

    jrBleInit();

    pDisplayS = std::make_unique<OledDisplay>();
    pDisplayS->clear();
    pDisplayS->drawString(15, 20, "Initializing");
    pDisplayS->drawString(30, 35, "Sensors...");
    pDisplayS->commit();
    vTaskDelay(pdMS_TO_TICKS(1500));

    pSensorS = std::make_unique<SensorReading>();
    pSensorS->startTask();
    vTaskDelay(pdMS_TO_TICKS(100));

    pAccelDetectorS =
        std::make_unique<JumpDetector>(pSensorS.get(), JUMP_THRESHOLD_FACTOR, MIN_JUMP_INTERVAL_MS);

    calibrationStartTimeS = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    systemStateS          = SystemState::Calibrating;

    auto pDisplayCtrl = std::make_unique<DisplayController>(*pDisplayS, *pAccelDetectorS);

    xTaskCreate(jumpDetectionTask, "jump_task", 4096, nullptr, 5, nullptr);
    xTaskCreate(displayTask, "display_task", 3072, pDisplayCtrl.release(), 4, nullptr);
    xTaskCreate(bleUpdateTask, "ble_task", 3072, nullptr, 3, nullptr);

    ESP_LOGI(TAG_S, "All tasks started");
    vTaskDelete(nullptr);
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG_S, "Jump Rope Monitor starting...");
    xTaskCreate(initTask, "init_task", 4096, nullptr, 6, nullptr);
}
