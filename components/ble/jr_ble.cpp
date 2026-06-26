#include "jr_ble.h"

#include <cstring>
#include <inttypes.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

namespace
{

char const *TAG_S = "JR_BLE";

ble_uuid128_t const JR_SVC_UUID = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e
);

ble_uuid128_t const JR_CTRL_UUID = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e
);

ble_uuid128_t const JR_DATA_UUID = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e
);

TickType_t notifyPeriodTicks(void)
{
    uint32_t const periodMs = static_cast<uint32_t>(1000 / JR_BLE_NOTIFY_HZ);
    uint32_t const safeMs   = (periodMs == 0) ? 1 : periodMs;
    return pdMS_TO_TICKS(safeMs);
}

uint8_t gOwnAddrTypeS    = 0;
uint16_t gConnHandleS    = BLE_HS_CONN_HANDLE_NONE;
uint16_t gDataValHandleS = 0;

bool gStreamingS          = false;
bool gResetOnStartS       = true;
uint32_t gJumpBaselineS   = 0;
uint32_t gJumpTotalS      = 0;
uint8_t gHrBpmS           = 0;
uint16_t gAccelMagS       = 0;
uint8_t gFlagsS           = 0;

portMUX_TYPE gLockS = portMUX_INITIALIZER_UNLOCKED;

uint32_t uptimeMs(void)
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
}

void buildPacket(jr_packet_v1_t *pOutP)
{
    portENTER_CRITICAL(&gLockS);
    uint32_t jumpTotal   = gJumpTotalS;
    uint8_t hr           = gHrBpmS;
    uint16_t accel       = gAccelMagS;
    uint8_t flags        = gFlagsS;
    uint32_t baseline    = gJumpBaselineS;
    bool resetOnStart    = gResetOnStartS;
    portEXIT_CRITICAL(&gLockS);

    uint32_t jumpToSend = (resetOnStart && jumpTotal >= baseline)
                              ? (jumpTotal - baseline)
                              : jumpTotal;

    pOutP->timestampMsM  = uptimeMs();
    pOutP->jumpCountM    = jumpToSend;
    pOutP->heartRateBpmM = hr;
    pOutP->accelMagM     = accel;
    pOutP->flagsM        = flags;
}

int ctrlAccessCb(
    uint16_t connHandleP, uint16_t attrHandleP,
    struct ble_gatt_access_ctxt *pCtxtP, void *pArgP
)
{
    (void) connHandleP;
    (void) attrHandleP;
    (void) pArgP;

    if (pCtxtP->op != BLE_GATT_ACCESS_OP_WRITE_CHR)
    {
        return BLE_ATT_ERR_UNLIKELY;
    }

    uint8_t cmd = 0;
    if (OS_MBUF_PKTLEN(pCtxtP->om) < 1)
    {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    if (ble_hs_mbuf_to_flat(pCtxtP->om, &cmd, sizeof(cmd), NULL) != 0)
    {
        return BLE_ATT_ERR_UNLIKELY;
    }

    if (cmd == 0x01)
    {
        portENTER_CRITICAL(&gLockS);
        gStreamingS = true;
        gJumpBaselineS = gJumpTotalS;
        portEXIT_CRITICAL(&gLockS);

        ESP_LOGI(
            TAG_S,
            "Streaming START (reset_on_start=%d, baseline=%" PRIu32 ")",
            (int) gResetOnStartS,
            (uint32_t) gJumpBaselineS
        );
    }
    else if (cmd == 0x00)
    {
        portENTER_CRITICAL(&gLockS);
        gStreamingS = false;
        portEXIT_CRITICAL(&gLockS);
        ESP_LOGI(TAG_S, "Streaming STOP");
    }
    else
    {
        ESP_LOGW(TAG_S, "Unknown control cmd: 0x%02X", cmd);
    }

    return 0;
}

int dataAccessCb(
    uint16_t connHandleP, uint16_t attrHandleP,
    struct ble_gatt_access_ctxt *pCtxtP, void *pArgP
)
{
    (void) connHandleP;
    (void) attrHandleP;
    (void) pArgP;

    if (pCtxtP->op != BLE_GATT_ACCESS_OP_READ_CHR)
    {
        return BLE_ATT_ERR_UNLIKELY;
    }

    jr_packet_v1_t pkt;
    buildPacket(&pkt);

    return (os_mbuf_append(pCtxtP->om, &pkt, sizeof(pkt)) == 0)
               ? 0
               : BLE_ATT_ERR_INSUFFICIENT_RES;
}

struct ble_gatt_chr_def gatt_charsS[] = {
    {
        (ble_uuid_t *) &JR_CTRL_UUID,
        ctrlAccessCb,
        NULL,
        NULL,
        static_cast<uint16_t>(BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP),
        0,
        NULL,
        NULL,
    },
    {
        (ble_uuid_t *) &JR_DATA_UUID,
        dataAccessCb,
        NULL,
        NULL,
        static_cast<uint16_t>(BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_READ),
        0,
        &gDataValHandleS,
        NULL,
    },
    { 0 },
};

struct ble_gatt_svc_def const gatt_svcsS[] = {
    {
        BLE_GATT_SVC_TYPE_PRIMARY,
        (ble_uuid_t *) &JR_SVC_UUID,
        NULL,
        gatt_charsS,
    },
    { 0 },
};

void startAdvertising(void);

int gapEventCb(struct ble_gap_event *pEventP, void *pArgP)
{
    (void) pArgP;

    switch (pEventP->type)
    {
        case BLE_GAP_EVENT_CONNECT:
            if (pEventP->connect.status == 0)
            {
                gConnHandleS = pEventP->connect.conn_handle;
                ESP_LOGI(TAG_S, "Connected (handle=%d)", (int) gConnHandleS);
            }
            else
            {
                ESP_LOGW(TAG_S, "Connect failed; status=%d", pEventP->connect.status);
                startAdvertising();
            }
            return 0;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG_S, "Disconnected; reason=%d", pEventP->disconnect.reason);
            gConnHandleS = BLE_HS_CONN_HANDLE_NONE;
            gStreamingS  = false;
            startAdvertising();
            return 0;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            startAdvertising();
            return 0;

        case BLE_GAP_EVENT_SUBSCRIBE:
            ESP_LOGI(
                TAG_S,
                "Subscribe: conn=%d attr=%d notify=%d indicate=%d",
                (int) pEventP->subscribe.conn_handle,
                (int) pEventP->subscribe.attr_handle,
                (int) pEventP->subscribe.cur_notify,
                (int) pEventP->subscribe.cur_indicate
            );
            return 0;

        default:
            return 0;
    }
}

void startAdvertising(void)
{
    struct ble_gap_adv_params adv;
    struct ble_hs_adv_fields fields;

    memset(&adv, 0, sizeof(adv));
    memset(&fields, 0, sizeof(fields));

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    char const *pName           = ble_svc_gap_device_name();
    fields.name                 = reinterpret_cast<uint8_t *>(const_cast<char *>(pName));
    fields.name_len             = static_cast<uint8_t>(strlen(pName));
    fields.name_is_complete     = 1;

    fields.uuids128             = const_cast<ble_uuid128_t *>(&JR_SVC_UUID);
    fields.num_uuids128         = 1;
    fields.uuids128_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0)
    {
        ESP_LOGE(TAG_S, "ble_gap_adv_set_fields failed: rc=%d", rc);
    }

    adv.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(gOwnAddrTypeS, NULL, BLE_HS_FOREVER, &adv, gapEventCb, NULL);
    if (rc != 0)
    {
        ESP_LOGE(TAG_S, "ble_gap_adv_start failed: rc=%d", rc);
    }
    else
    {
        ESP_LOGI(TAG_S, "Advertising...");
    }
}

void bleOnSync(void)
{
    ble_hs_id_infer_auto(0, &gOwnAddrTypeS);
    ble_svc_gap_device_name_set("JRope-C6");
    startAdvertising();
}

void bleOnReset(int reasonP)
{
    ESP_LOGE(TAG_S, "BLE reset; reason=%d", reasonP);
}

void notifyTask(void *pParamP)
{
    (void) pParamP;

    TickType_t const period = notifyPeriodTicks();
    jr_packet_v1_t pkt;

    while (true)
    {
        bool streaming;
        portENTER_CRITICAL(&gLockS);
        streaming = gStreamingS;
        portEXIT_CRITICAL(&gLockS);

        if (streaming && gConnHandleS != BLE_HS_CONN_HANDLE_NONE && gDataValHandleS != 0)
        {
            buildPacket(&pkt);

            struct os_mbuf *pOm = ble_hs_mbuf_from_flat(&pkt, sizeof(pkt));
            if (pOm != nullptr)
            {
                int rc = ble_gatts_notify_custom(gConnHandleS, gDataValHandleS, pOm);
                if (rc != 0)
                {
                    ESP_LOGD(TAG_S, "notify rc=%d", rc);
                }
            }
        }
        vTaskDelay(period);
    }
}

void hostTask(void *pParamP)
{
    (void) pParamP;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

}  // namespace

void jrBleInit(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    else
    {
        ESP_ERROR_CHECK(ret);
    }

    nimble_port_init();

    ble_svc_gap_init();
    ble_svc_gatt_init();

    ble_gatts_count_cfg(gatt_svcsS);
    ble_gatts_add_svcs(gatt_svcsS);

    ble_hs_cfg.sync_cb  = bleOnSync;
    ble_hs_cfg.reset_cb = bleOnReset;

    xTaskCreate(notifyTask, "jr_notify", 4096, NULL, 5, NULL);
    nimble_port_freertos_init(hostTask);

    ESP_LOGI(TAG_S, "jrBleInit ok (notify_hz=%d)", (int) JR_BLE_NOTIFY_HZ);
}

void jrBleSetSensorSnapshot(
    uint32_t jumpCountTotalP, uint8_t heartRateBpmP, uint16_t accelMagP, uint8_t flagsP
)
{
    portENTER_CRITICAL(&gLockS);
    gJumpTotalS = jumpCountTotalP;
    gHrBpmS     = heartRateBpmP;
    gAccelMagS  = accelMagP;
    gFlagsS     = flagsP;
    portEXIT_CRITICAL(&gLockS);
}

void jrBleSetResetOnStart(bool enableP)
{
    portENTER_CRITICAL(&gLockS);
    gResetOnStartS = enableP;
    portEXIT_CRITICAL(&gLockS);
}

bool jrBleIsStreaming(void)
{
    bool result;
    portENTER_CRITICAL(&gLockS);
    result = gStreamingS;
    portEXIT_CRITICAL(&gLockS);
    return result;
}

bool jrBleIsConnected(void)
{
    return gConnHandleS != BLE_HS_CONN_HANDLE_NONE;
}
