#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct __attribute__((packed))
    {
        uint32_t timestampMsM;
        uint32_t jumpCountM;
        uint8_t heartRateBpmM;
        uint16_t accelMagM;
        uint8_t flagsM;
    } jr_packet_v1_t;

#ifdef __cplusplus
    static_assert(sizeof(jr_packet_v1_t) == 12, "jr_packet_v1_t must be 12 bytes");
#else
    _Static_assert(sizeof(jr_packet_v1_t) == 12, "jr_packet_v1_t must be 12 bytes");
#endif

    void jrBleInit(void);

    void jrBleSetSensorSnapshot(
        uint32_t jumpCountTotalP, uint8_t heartRateBpmP, uint16_t accelMagP, uint8_t flagsP
    );

    void jrBleSetResetOnStart(bool enableP);

    bool jrBleIsStreaming(void);

    bool jrBleIsConnected(void);

#ifdef __cplusplus
}
#endif
