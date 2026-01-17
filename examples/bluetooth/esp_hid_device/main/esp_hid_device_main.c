/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_bt_defs.h"
#if CONFIG_BT_BLE_ENABLED
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#endif
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#if CONFIG_BT_SDP_COMMON_ENABLED
#include "esp_sdp_api.h"
#endif /* CONFIG_BT_SDP_COMMON_ENABLED */

#include "esp_hidd.h"
#include "esp_hid_gap.h"

static const char *TAG = "HID_DEV_DEMO";
extern void init_gpio_led_pm(void);
extern void switch_gpio(bool on);

typedef struct
{
    TaskHandle_t task_hdl;
    esp_hidd_dev_t *hid_dev;
    uint8_t protocol_mode;
    uint8_t *buffer;
} local_param_t;

#if CONFIG_BT_BLE_ENABLED
static local_param_t s_ble_hid_param = {0};

const unsigned char mediaReportMap[] = {
    0x05,
    0x0C, // Usage Page (Consumer)
    0x09,
    0x01, // Usage (Consumer Control)
    0xA1,
    0x01, // Collection (Application)
    0x85,
    0x03, //   Report ID (3)
    0x09,
    0x02, //   Usage (Numeric Key Pad)
    0xA1,
    0x02, //   Collection (Logical)
    0x05,
    0x09, //     Usage Page (Button)
    0x19,
    0x01, //     Usage Minimum (0x01)
    0x29,
    0x0A, //     Usage Maximum (0x0A)
    0x15,
    0x01, //     Logical Minimum (1)
    0x25,
    0x0A, //     Logical Maximum (10)
    0x75,
    0x04, //     Report Size (4)
    0x95,
    0x01, //     Report Count (1)
    0x81,
    0x00, //     Input (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0, //   End Collection
    0x05,
    0x0C, //   Usage Page (Consumer)
    0x09,
    0x86, //   Usage (Channel)
    0x15,
    0xFF, //   Logical Minimum (-1)
    0x25,
    0x01, //   Logical Maximum (1)
    0x75,
    0x02, //   Report Size (2)
    0x95,
    0x01, //   Report Count (1)
    0x81,
    0x46, //   Input (Data,Var,Rel,No Wrap,Linear,Preferred State,Null State)
    0x09,
    0xE9, //   Usage (Volume Increment)
    0x09,
    0xEA, //   Usage (Volume Decrement)
    0x15,
    0x00, //   Logical Minimum (0)
    0x75,
    0x01, //   Report Size (1)
    0x95,
    0x02, //   Report Count (2)
    0x81,
    0x02, //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x09,
    0xE2, //   Usage (Mute)
    0x09,
    0x30, //   Usage (Power)
    0x09,
    0x83, //   Usage (Recall Last)
    0x09,
    0x81, //   Usage (Assign Selection)
    0x09,
    0xB0, //   Usage (Play)
    0x09,
    0xB1, //   Usage (Pause)
    0x09,
    0xB2, //   Usage (Record)
    0x09,
    0xB3, //   Usage (Fast Forward)
    0x09,
    0xB4, //   Usage (Rewind)
    0x09,
    0xB5, //   Usage (Scan Next Track)
    0x09,
    0xB6, //   Usage (Scan Previous Track)
    0x09,
    0xB7, //   Usage (Stop)
    0x15,
    0x01, //   Logical Minimum (1)
    0x25,
    0x0C, //   Logical Maximum (12)
    0x75,
    0x04, //   Report Size (4)
    0x95,
    0x01, //   Report Count (1)
    0x81,
    0x00, //   Input (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x09,
    0x80, //   Usage (Selection)
    0xA1,
    0x02, //   Collection (Logical)
    0x05,
    0x09, //     Usage Page (Button)
    0x19,
    0x01, //     Usage Minimum (0x01)
    0x29,
    0x03, //     Usage Maximum (0x03)
    0x15,
    0x01, //     Logical Minimum (1)
    0x25,
    0x03, //     Logical Maximum (3)
    0x75,
    0x02, //     Report Size (2)
    0x81,
    0x00, //     Input (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0, //   End Collection
    0x81,
    0x03, //   Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0, // End Collection
};

static esp_hid_raw_report_map_t ble_report_maps[] = {
    /* This block is compiled for bluedroid as well */
    {
        .data = mediaReportMap,
        .len = sizeof(mediaReportMap)}};

static esp_hid_device_config_t ble_hid_config = {
    .vendor_id = 0x16C0,
    .product_id = 0x05DF,
    .version = 0x0100,
    .device_name = "ESP BLE ZXQ",
    .manufacturer_name = "Espressif",
    .serial_number = "1234567890",
    .report_maps = ble_report_maps,
    .report_maps_len = 1};

#if !CONFIG_BT_NIMBLE_ENABLED || CONFIG_EXAMPLE_HID_DEVICE_ROLE == 1
void ble_hid_demo_task(void *pvParameters)
{
    switch_gpio(true);
    vTaskDelete(NULL); // 修复：正确地注销该任务
}
#endif

void ble_hid_task_start_up(void)
{
    xTaskCreate(ble_hid_demo_task, "ble_hid_demo_task", 4 * 1024, NULL, configMAX_PRIORITIES - 3,
                &s_ble_hid_param.task_hdl);
}

void ble_hid_task_shut_down(void)
{
    switch_gpio(false);
}

static void ble_hidd_event_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidd_event_t event = (esp_hidd_event_t)id;
    esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;
    static const char *TAG = "HID_DEV_BLE";
    ESP_LOGI(TAG, "Free heap: %u bytes", esp_get_free_heap_size());
    switch (event)
    {
    case ESP_HIDD_START_EVENT:
    {
        ESP_LOGI(TAG, "START");
        esp_hid_ble_gap_adv_start();
        break;
    }
    case ESP_HIDD_CONNECT_EVENT:
    {
        ESP_LOGI(TAG, "Connected, updating params for ultra-low power...");
        break;
    }
    case ESP_HIDD_PROTOCOL_MODE_EVENT:
    {
        ESP_LOGI(TAG, "PROTOCOL MODE[%u]: %s", param->protocol_mode.map_index, param->protocol_mode.protocol_mode ? "REPORT" : "BOOT");
        break;
    }
    case ESP_HIDD_CONTROL_EVENT:
    {
        ESP_LOGI(TAG, "CONTROL[%u]: %sSUSPEND", param->control.map_index, param->control.control ? "EXIT_" : "");
        break;
    }
    case ESP_HIDD_OUTPUT_EVENT:
    {
        ESP_LOGI(TAG, "OUTPUT[%u]: %8s ID: %2u, Len: %d, Data:", param->output.map_index, esp_hid_usage_str(param->output.usage), param->output.report_id, param->output.length);
        ESP_LOG_BUFFER_HEX(TAG, param->output.data, param->output.length);
        break;
    }
    case ESP_HIDD_FEATURE_EVENT:
    {
        ESP_LOGI(TAG, "FEATURE[%u]: %8s ID: %2u, Len: %d, Data:", param->feature.map_index, esp_hid_usage_str(param->feature.usage), param->feature.report_id, param->feature.length);
        ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
        break;
    }
    case ESP_HIDD_DISCONNECT_EVENT:
    {
        ESP_LOGI(TAG, "DISCONNECT: %s", esp_hid_disconnect_reason_str(esp_hidd_dev_transport_get(param->disconnect.dev), param->disconnect.reason));
        ble_hid_task_shut_down();
        esp_hid_ble_gap_adv_start();
        break;
    }
    case ESP_HIDD_STOP_EVENT:
    {
        ESP_LOGI(TAG, "STOP");
        break;
    }
    default:
        break;
    }
    return;
}
#endif

void app_main(void)
{
    esp_err_t ret;
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "setting hid gap, mode:%d", HID_DEV_MODE);
    ret = esp_hid_gap_init(HID_DEV_MODE);
    ESP_ERROR_CHECK(ret);

    ret = esp_hid_ble_gap_adv_init(ESP_HID_APPEARANCE_GENERIC, ble_hid_config.device_name);
    ESP_ERROR_CHECK(ret);

    if ((ret = esp_ble_gatts_register_callback(esp_hidd_gatts_event_handler)) != ESP_OK)
    {
        ESP_LOGE(TAG, "GATTS register callback failed: %d", ret);
        return;
    }

    ESP_LOGI(TAG, "setting ble device");
    ESP_ERROR_CHECK(
        esp_hidd_dev_init(&ble_hid_config, ESP_HID_TRANSPORT_BLE, ble_hidd_event_callback, &s_ble_hid_param.hid_dev));

    init_gpio_led_pm();
}
