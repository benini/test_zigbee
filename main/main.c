#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"

// Zigbee Includes
#include "esp_zigbee_core.h"
#include "zcl/esp_zigbee_zcl_common.h"
#include "zcl/esp_zigbee_zcl_command.h"

#define TAG "PRESEPE_CTRL"

// --- BLE UUIDs ---
#define SERVICE_UUID           0x00FF
#define CHAR_ON_UUID           0xFF01
#define CHAR_OFF_UUID          0xFF02

// --- VARIABILI GLOBALI ---
static uint32_t duration_on_sec = 30;
static uint32_t duration_off_sec = 60;
static bool zigbee_device_connected = false;
static uint16_t target_short_addr = 0xFFFF;
static uint8_t  target_endpoint = 0;

// --- ZIGBEE DEFINITIONS ---
#define HA_ONOFF_SWITCH_ENDPOINT 1
#define ESP_ZB_PRIMARY_CHANNEL_MASK ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK

// --- BLE 5.0 EXTENDED ADVERTISING CONFIG ---
// Definiamo l'handle manualmente (indice 0)
#define EXT_ADV_HANDLE_MAIN 0

// Parametri Extended Advertising
esp_ble_gap_ext_adv_params_t ext_adv_params_legacy = {
    .type = ESP_BLE_GAP_SET_EXT_ADV_PROP_CONNECTABLE | ESP_BLE_GAP_SET_EXT_ADV_PROP_SCANNABLE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    .primary_phy = ESP_BLE_GAP_PHY_1M,
    .max_skip = 0,
    .secondary_phy = ESP_BLE_GAP_PHY_1M,
    .sid = 0,
    .scan_req_notif = false,
};

// Raw Advertising Data
static uint8_t raw_adv_data[] = {
    0x02, 0x01, 0x06,
    0x08, 0x09, 'P', 'R', 'E', 'S', 'E', 'P', 'E',
    0x03, 0x03, 0xFF, 0x00
};

// Struttura necessaria per l'avvio dell'advertising (NUOVA API)
static esp_ble_gap_ext_adv_t ext_adv_start_param[] = {
    {
        .instance = EXT_ADV_HANDLE_MAIN,
        .duration = 0,   // 0 = per sempre
        .max_events = 0,
    }
};

// --- LOGICA DI CONTROLLO (TASK) ---
void presepe_logic_task(void *pvParameters) {
    bool state_on = false;
    const uint8_t CMD_ID_OFF = 0x00;
    const uint8_t CMD_ID_ON  = 0x01;

    while(1) {
        if (!zigbee_device_connected) {
            ESP_LOGW(TAG, "In attesa del Sonoff Zigbee...");
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        uint32_t current_wait = state_on ? duration_on_sec : duration_off_sec;
        ESP_LOGI(TAG, "Stato: %s (%lu s)", state_on ? "ACCESO" : "SPENTO", current_wait);

        esp_zb_zcl_on_off_cmd_t cmd_req;
        memset(&cmd_req, 0, sizeof(esp_zb_zcl_on_off_cmd_t));

        cmd_req.zcl_basic_cmd.src_endpoint = HA_ONOFF_SWITCH_ENDPOINT;
        cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
        cmd_req.zcl_basic_cmd.dst_endpoint = target_endpoint;
        cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = target_short_addr;
        cmd_req.on_off_cmd_id = state_on ? CMD_ID_OFF : CMD_ID_ON;

        esp_zb_zcl_on_off_cmd_req(&cmd_req);

        state_on = !state_on;
        vTaskDelay(pdMS_TO_TICKS(current_wait * 1000));
    }
}

// --- BLE CALLBACKS ---
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT:
            // Configurazione BLE 5.0 Params
            esp_ble_gap_ext_adv_set_params(EXT_ADV_HANDLE_MAIN, &ext_adv_params_legacy);
            esp_ble_gatts_create_service(gatts_if, &param->reg.app_id, 40);
            break;
        case ESP_GATTS_CREATE_EVT:
            ESP_LOGI(TAG, "Servizio BLE creato");
            esp_bt_uuid_t uuid_srv = { .len = ESP_UUID_LEN_16, .uuid = { .uuid16 = SERVICE_UUID } };
            esp_ble_gatts_start_service(param->create.service_handle);

            esp_bt_uuid_t uuid_on = { .len = ESP_UUID_LEN_16, .uuid = { .uuid16 = CHAR_ON_UUID } };
            esp_ble_gatts_add_char(param->create.service_handle, &uuid_on, ESP_GATT_PERM_WRITE | ESP_GATT_PERM_READ,
                               ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ, NULL, NULL);

            esp_bt_uuid_t uuid_off = { .len = ESP_UUID_LEN_16, .uuid = { .uuid16 = CHAR_OFF_UUID } };
            esp_ble_gatts_add_char(param->create.service_handle, &uuid_off, ESP_GATT_PERM_WRITE | ESP_GATT_PERM_READ,
                               ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ, NULL, NULL);
            break;
        case ESP_GATTS_WRITE_EVT:
            if (param->write.len == 4) {
                uint32_t val = *((uint32_t*)param->write.value);
                ESP_LOGI(TAG, "BLE Write: %lu", val);
                duration_on_sec = val;
            }
            break;
        default: break;
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_EXT_ADV_SET_PARAMS_COMPLETE_EVT:
            // Params settati, ora configuriamo i dati raw
            esp_ble_gap_config_ext_adv_data_raw(EXT_ADV_HANDLE_MAIN, sizeof(raw_adv_data), raw_adv_data);
            break;
        case ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT:
            // Dati settati, avviamo l'advertising passando la struct corretta
            esp_ble_gap_ext_adv_start(1, ext_adv_start_param);
            ESP_LOGI(TAG, "BLE Extended Advertising avviato");
            break;
        default: break;
    }
}

// --- ZIGBEE CALLBACKS ---
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee Stack Init");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Rete Zigbee Aperta");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE: {
        esp_zb_zdo_signal_device_annce_params_t *dev_annce_params = (esp_zb_zdo_signal_device_annce_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        ESP_LOGI(TAG, "Dispositivo Rilevato! Addr: 0x%x", dev_annce_params->device_short_addr);
        target_short_addr = dev_annce_params->device_short_addr;
        target_endpoint = 1;
        zigbee_device_connected = true;
        break;
    }
    default: break;
    }
}

static void esp_zb_task(void *pvParameters) {
    esp_zb_cfg_t zb_nwk_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_COORDINATOR,
        .install_code_policy = false,
        // CORREZIONE 1: zc_cfg -> zczr_cfg (Unificato)
        .nwk_cfg.zczr_cfg = {
            .max_children = 32,
        },
    };
    esp_zb_init(&zb_nwk_cfg);

    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_attribute_list_t *basic_attr = esp_zb_basic_cluster_create(NULL);
    esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_attr, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_attribute_list_t *identify_attr = esp_zb_identify_cluster_create(NULL);
    esp_zb_cluster_list_add_identify_cluster(cluster_list, identify_attr, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_on_off_cluster(cluster_list, esp_zb_on_off_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = HA_ONOFF_SWITCH_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID,
        .app_device_version = 0
    };

    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_ep_list_add_ep(ep_list, cluster_list, endpoint_config);
    esp_zb_device_register(ep_list);

    esp_zb_core_action_handler_register(esp_zb_app_signal_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);

    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_main_loop_iteration();
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_bluedroid_init();
    esp_bluedroid_enable();

    esp_ble_gatts_register_callback(gatts_profile_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(0);

    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
    xTaskCreate(presepe_logic_task, "Presepe_Logic", 4096, NULL, 5, NULL);
}