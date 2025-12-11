#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "esp_log.h"

// BLE Includes
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"

// Zigbee Includes
#include "esp_zigbee_core.h"
#include "zcl/esp_zigbee_zcl_common.h"

static const char *TAG = "SMART_CTRL_C6";

// --- Configuration Constants ---
#define SERVICE_UUID        0x00FF
#define CHAR_ON_DUR_UUID    0xFF01
#define CHAR_OFF_DUR_UUID   0xFF02
#define APP_ID              0

// --- Global State ---
static uint32_t g_on_duration = 30;  // Seconds
static uint32_t g_off_duration = 60; // Seconds
static uint16_t g_zb_short_addr = 0xFFFF; // Target Sonoff Address
static bool g_device_paired = false;

// --- NVS Storage Helper ---
void save_settings_to_nvs() {
    nvs_handle_t handle;
    if (nvs_open("storage", NVS_READWRITE, &handle) == ESP_OK) {
        nvs_set_u32(handle, "on_dur", g_on_duration);
        nvs_set_u32(handle, "off_dur", g_off_duration);
        nvs_commit(handle);
        nvs_close(handle);
        ESP_LOGI(TAG, "Settings saved to NVS: ON=%lus, OFF=%lus", g_on_duration, g_off_duration);
    }
}

void load_settings_from_nvs() {
    nvs_handle_t handle;
    if (nvs_open("storage", NVS_READONLY, &handle) == ESP_OK) {
        nvs_get_u32(handle, "on_dur", &g_on_duration);
        nvs_get_u32(handle, "off_dur", &g_off_duration);
        nvs_close(handle);
    }
}

// --- Zigbee Logic ---
static void send_zb_on_off(bool power_on) {
    if (!g_device_paired) return;

    esp_zb_zcl_on_off_cmd_req_t cmd_req = {
        .zcl_basic_cmd.dst_addr_u.addr_16 = g_zb_short_addr,
        .zcl_basic_cmd.dst_endpoint = 1, // Standard Sonoff EP
        .zcl_basic_cmd.src_endpoint = 1,
        .zcl_basic_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
    };

    if (power_on) {
        ESP_LOGI(TAG, "Sending Zigbee ON Command...");
        esp_zb_zcl_on_off_cmd_req(&cmd_req, ESP_ZB_ZCL_ON_OFF_CMD_ON);
    } else {
        ESP_LOGI(TAG, "Sending Zigbee OFF Command...");
        esp_zb_zcl_on_off_cmd_req(&cmd_req, ESP_ZB_ZCL_ON_OFF_CMD_OFF);
    }
}

void zigbee_loop_task(void *pvParameters) {
    while (1) {
        if (g_device_paired) {
            send_zb_on_off(true);
            vTaskDelay(pdMS_TO_TICKS(g_on_duration * 1000));

            send_zb_on_off(false);
            vTaskDelay(pdMS_TO_TICKS(g_off_duration * 1000));
        } else {
            ESP_LOGI(TAG, "Waiting for Sonoff device to pair...");
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask) {
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee stack initialized");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (status == ESP_OK) {
            ESP_LOGI(TAG, "Network steering started. Search for device...");
        } else {
            ESP_LOGW(TAG, "Steering failed, retrying...");
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE: {
        esp_zb_zdo_signal_device_annce_params_t *dev_annce_params = (esp_zb_zdo_signal_device_annce_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        ESP_LOGI(TAG, "New device joined! Short Addr: 0x%x", dev_annce_params->device_short_addr);
        g_zb_short_addr = dev_annce_params->device_short_addr;
        g_device_paired = true;
        break;
    }
    default:
        ESP_LOGD(TAG, "Zigbee signal: 0x%x, status: %s", sig_type, esp_err_to_name(status));
        break;
    }
}

// --- BLE Logic ---
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    static uint16_t char_on_handle, char_off_handle;

    switch (event) {
    case ESP_GATTS_REG_EVT: {
        esp_gatt_srvc_id_t service_id = {.is_primary = true, .id.uuid.len = ESP_UUID_LEN_16, .id.uuid.uuid.uuid16 = SERVICE_UUID};
        esp_ble_gatts_create_service(gatts_if, &service_id, 8);
        break;
    }
    case ESP_GATTS_CREATE_EVT: {
        esp_ble_gatts_start_service(param->create.service_handle);
        // Add Characteristic 1 (ON Duration)
        esp_bt_uuid_t c1_uuid = {.len = ESP_UUID_LEN_16, .uuid.uuid16 = CHAR_ON_DUR_UUID};
        esp_ble_gatts_add_char(param->create.service_handle, &c1_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                               ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE, NULL, NULL);
        // Add Characteristic 2 (OFF Duration)
        esp_bt_uuid_t c2_uuid = {.len = ESP_UUID_LEN_16, .uuid.uuid16 = CHAR_OFF_DUR_UUID};
        esp_ble_gatts_add_char(param->create.service_handle, &c2_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                               ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE, NULL, NULL);
        break;
    }
    case ESP_GATTS_ADD_CHAR_EVT:
        if (param->add_char.char_uuid.uuid.uuid16 == CHAR_ON_DUR_UUID) char_on_handle = param->add_char.attr_handle;
        if (param->add_char.char_uuid.uuid.uuid16 == CHAR_OFF_DUR_UUID) char_off_handle = param->add_char.attr_handle;
        break;
    case ESP_GATTS_READ_EVT: {
        esp_gatt_rsp_t rsp = {0};
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = 4;
        uint32_t val = (param->read.handle == char_on_handle) ? g_on_duration : g_off_duration;
        memcpy(rsp.attr_value.value, &val, 4);
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
        break;
    }
    case ESP_GATTS_WRITE_EVT: {
        if (param->write.len == 4) {
            uint32_t new_val;
            memcpy(&new_val, param->write.value, 4);
            if (param->write.handle == char_on_handle) g_on_duration = new_val;
            else if (param->write.handle == char_off_handle) g_off_duration = new_val;
            ESP_LOGI(TAG, "BLE Updated Timer: ON=%lu, OFF=%lu", g_on_duration, g_off_duration);
            save_settings_to_nvs();
        }
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        break;
    }
    case ESP_GATTS_CONNECT_EVT: {
        esp_ble_gap_stop_advertising(); // Stop to save radio time during connection if needed
        break;
    }
    case ESP_GATTS_DISCONNECT_EVT: {
        esp_ble_gap_start_advertising(&(esp_ble_adv_params_t){
            .adv_int_min = 0x20,
            .adv_int_max = 0x40,
            .adv_type = ADV_TYPE_IND,
            .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
            .channel_map = ADV_CHNL_ALL,
            .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
        });
        break;
    }
    default: break;
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    if (event == ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT) {
        esp_ble_gap_start_advertising(&(esp_ble_adv_params_t){
            .adv_int_min = 0x20, // Critical constraint fix
            .adv_int_max = 0x40,
            .adv_type = ADV_TYPE_IND,
            .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
            .channel_map = ADV_CHNL_ALL,
            .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
        });
    }
}

// --- Initialization ---
void app_main(void) {
    // 1. Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    load_settings_from_nvs();

    // 2. Initialize Zigbee
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZC_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    // Create standard On/Off Endpoint
    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_on_off_cluster_cfg_t on_off_cfg = { .on_off = false };
    esp_zb_cluster_list_add_on_off_cluster(cluster_list, esp_zb_on_off_cluster_create(&on_off_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_endpoint_config_t ep_config = {.endpoint = 1, .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID, .app_device_id = ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID};
    esp_zb_ep_list_add_ep(esp_zb_ep_list, cluster_list, ep_config);
    esp_zb_device_register(esp_zb_ep_list);

    esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);

    // 3. Initialize BLE
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_bluedroid_init();
    esp_bluedroid_enable();
    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(APP_ID);

    static esp_ble_adv_data_t adv_data = {
        .set_scan_rsp = false,
        .include_name = true,
        .include_txpower = false,
        .min_interval = 0x0006,
        .max_interval = 0x0010,
        .appearance = 0x00,
        .manufacturer_len = 0,
        .service_data_len = 0,
        .service_uuid_len = 0,
        .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
    };
    esp_ble_gap_set_device_name("C6_ZIGBEE_CONFIG");
    esp_ble_gap_config_adv_data(&adv_data);

    // 4. Start Zigbee and Timer Tasks
    xTaskCreate(zigbee_loop_task, "zb_loop", 4096, NULL, 5, NULL);
    esp_zb_main_loop_iteration(); // Starts the Zigbee scheduler
}