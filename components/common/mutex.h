#pragma once

#include "esp_assert.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class MutexGuard
{
public:
    explicit MutexGuard(SemaphoreHandle_t pMutexP)
        : pMutexM{pMutexP}
        , lockedM{false}
    {
        configASSERT(pMutexM != nullptr && "Mutex handle is null");

        BaseType_t taken = xSemaphoreTake(pMutexM, portMAX_DELAY);
        configASSERT(taken == pdTRUE && "Failed to acquire mutex");

        lockedM = true;
    }

    ~MutexGuard()
    {
        if (lockedM)
        {
            xSemaphoreGive(pMutexM);
        }
    }

    MutexGuard(const MutexGuard&)            = delete;
    MutexGuard& operator=(const MutexGuard&) = delete;

    MutexGuard(MutexGuard &&rOtherP) noexcept
        : pMutexM{rOtherP.pMutexM}
        , lockedM{rOtherP.lockedM}
    {
        rOtherP.lockedM = false;
    }

    MutexGuard& operator=(MutexGuard &&rOtherP) noexcept
    {
        if (this != &rOtherP)
        {
            if (lockedM)
            {
                xSemaphoreGive(pMutexM);
            }
            pMutexM         = rOtherP.pMutexM;
            lockedM         = rOtherP.lockedM;
            rOtherP.lockedM = false;
        }
        return *this;
    }

private:
    SemaphoreHandle_t pMutexM;
    bool lockedM;
};
