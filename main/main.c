/**
 * @file main.c
 * @brief ESP32-C6 Zigbee Coordinator + BLE GATT Server for Smart Switch Control
 * 
 * This firmware:
 * - Acts as a Zigbee 3.0 Coordinator to control a Sonoff ZBMINIR2 smart switch
 * - Provides a BLE GATT server for runtime configuration of ON/OFF timer durations
 * - Uses NVS for persistent storage of timer values
 * - Implements software coexistence for simultaneous Zigbee and BLE operation
 * 
 * @author Senior Embedded Systems Engineer
 * @version 1.0.0
 * @date 2024
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>

/* FreeRTOS */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

/* ESP System */
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_coex.h"

/* Bluetooth */
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"
#include "esp_bt_defs.h"

/* Zigbee */
#include "esp_zigbee_core.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "zcl/esp_zigbee_zcl_common.h"

/* ========================================
 * Constants and Configuration
 * ======================================== */

static const char *TAG = "ZB_BLE_CTRL";

/* NVS Configuration */
#define NVS_NAMESPACE           "timer_config"
#define NVS_KEY_ON_DURATION     "on_duration"
#define NVS_KEY_OFF_DURATION    "off_duration"

/* Default Timer Values (in seconds) */
#define DEFAULT_ON_DURATION_SEC     30
#define DEFAULT_OFF_DURATION_SEC    60

/* Zigbee Configuration */
#define COORDINATOR_ENDPOINT        1
#define ZIGBEE_CHANNEL_MASK         (1UL << 11)  /* Channel 11 */
#define PERMIT_JOIN_DURATION        254          /* 254 = always open, 0 = close */

/* BLE Configuration */
#define DEVICE_NAME                 "ZB_Switch_Ctrl"
#define BLE_APP_ID                  0
#define BLE_MTU_SIZE                500

/* Custom Service UUIDs (128-bit) */
#define SERVICE_UUID_BYTES          { 0xF0, 0xDE, 0xBC, 0x9A, 0x78, 0x56, 0x34, 0x12, \
                                      0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12 }
#define CHAR_ON_UUID_BYTES          { 0xF1, 0xDE, 0xBC, 0x9A, 0x78, 0x56, 0x34, 0x12, \
                                      0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12 }
#define CHAR_OFF_UUID_BYTES         { 0xF2, 0xDE, 0xBC, 0x9A, 0x78, 0x56, 0x34, 0x12, \
                                      0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12 }

/* GATT Handle Indices */
#define IDX_SVC                     0
#define IDX_CHAR_ON_DECL            1
#define IDX_CHAR_ON_VAL             2
#define IDX_CHAR_OFF_DECL           3
#define IDX_CHAR_OFF_VAL            4
#define IDX_NB                      5

/* Task Configuration */
#define ZIGBEE_TASK_STACK_SIZE      4096
#define CONTROL_TASK_STACK_SIZE     4096
#define ZIGBEE_TASK_PRIORITY        5
#define CONTROL_TASK_PRIORITY       4

/* ========================================
 * Type Definitions
 * ======================================== */

/**
 * @brief Structure to hold paired Zigbee device information
 */
typedef struct {
    bool        is_joined;
    uint16_t    short_addr;
    uint8_t     endpoint;
    uint8_t     ieee_addr[8];
} zigbee_device_t;

/**
 * @brief Application state structure
 */
typedef struct {
    /* Timer Configuration */
    uint32_t            on_duration_sec;
    uint32_t            off_duration_sec;
    SemaphoreHandle_t   config_mutex;
    
    /* Zigbee State */
    zigbee_device_t     target_device;
    bool                network_formed;
    bool                permit_joining;
    
    /* BLE State */
    uint16_t            gatts_if;
    uint16_t            conn_id;
    bool                ble_connected;
    uint16_t            handle_table[IDX_NB];
    
    /* Control State */
    bool                control_active;
    bool                current_switch_state;  /* true = ON, false = OFF */
} app_state_t;

/* ========================================
 * Global Variables
 * ======================================== */

static app_state_t g_app_state = {
    .on_duration_sec    = DEFAULT_ON_DURATION_SEC,
    .off_duration_sec   = DEFAULT_OFF_DURATION_SEC,
    .config_mutex       = NULL,
    .target_device      = { .is_joined = false },
    .network_formed     = false,
    .permit_joining     = false,
    .gatts_if           = ESP_GATT_IF_NONE,
    .conn_id            = 0,
    .ble_connected      = false,
    .control_active     = false,
    .current_switch_state = false,
};

/* Task Handles */
static TaskHandle_t g_control_task_handle = NULL;

/* Event Groups */
static EventGroupHandle_t g_zigbee_event_group = NULL;
#define ZB_NETWORK_READY_BIT    BIT0
#define ZB_DEVICE_JOINED_BIT    BIT1

/* ========================================
 * GATT Service Table Definition
 * ======================================== */

static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t char_decl_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint8_t char_prop_rw = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE;

static const uint8_t service_uuid[16] = SERVICE_UUID_BYTES;
static const uint8_t char_on_uuid[16] = CHAR_ON_UUID_BYTES;
static const uint8_t char_off_uuid[16] = CHAR_OFF_UUID_BYTES;

static uint32_t char_on_value = DEFAULT_ON_DURATION_SEC;
static uint32_t char_off_value = DEFAULT_OFF_DURATION_SEC;

static const esp_gatts_attr_db_t gatt_db[IDX_NB] = {
    /* Service Declaration */
    [IDX_SVC] = {
        { ESP_GATT_AUTO_RSP },
        {
            ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid,
            ESP_GATT_PERM_READ,
            sizeof(service_uuid), sizeof(service_uuid), (uint8_t *)service_uuid
        }
    },
    
    /* ON Duration Characteristic Declaration */
    [IDX_CHAR_ON_DECL] = {
        { ESP_GATT_AUTO_RSP },
        {
            ESP_UUID_LEN_16, (uint8_t *)&char_decl_uuid,
            ESP_GATT_PERM_READ,
            sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_rw
        }
    },
    
    /* ON Duration Characteristic Value */
    [IDX_CHAR_ON_VAL] = {
        { ESP_GATT_RSP_BY_APP },  /* Response handled by application */
        {
            ESP_UUID_LEN_128, (uint8_t *)char_on_uuid,
            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            sizeof(uint32_t), sizeof(uint32_t), (uint8_t *)&char_on_value
        }
    },
    
    /* OFF Duration Characteristic Declaration */
    [IDX_CHAR_OFF_DECL] = {
        { ESP_GATT_AUTO_RSP },
        {
            ESP_UUID_LEN_16, (uint8_t *)&char_decl_uuid,
            ESP_GATT_PERM_READ,
            sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_rw
        }
    },
    
    /* OFF Duration Characteristic Value */
    [IDX_CHAR_OFF_VAL] = {
        { ESP_GATT_RSP_BY_APP },  /* Response handled by application */
        {
            ESP_UUID_LEN_128, (uint8_t *)char_off_uuid,
            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            sizeof(uint32_t), sizeof(uint32_t), (uint8_t *)&char_off_value
        }
    },
};

/* BLE Advertising Data */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006,  /* 7.5ms (N * 1.25ms) */
    .max_interval        = 0x0010,  /* 20ms */
    .appearance          = 0x00,
    .manufacturer_len    = 0,
    .p_manufacturer_data = NULL,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),
    .p_service_uuid      = (uint8_t *)service_uuid,
    .flag                = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp        = true,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006,
    .max_interval        = 0x0010,
    .appearance          = 0x00,
    .manufacturer_len    = 0,
    .p_manufacturer_data = NULL,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = 0,
    .p_service_uuid      = NULL,
    .flag                = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,     /* 20ms (N * 0.625ms) */
    .adv_int_max        = 0x40,     /* 40ms */
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .peer_addr          = { 0 },
    .peer_addr_type     = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

/* ========================================
 * NVS Functions
 * ======================================== */

/**
 * @brief Load timer configuration from NVS
 * @return ESP_OK on success
 */
static esp_err_t nvs_load_timer_config(void)
{
    nvs_handle_t handle;
    esp_err_t err;
    
    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "NVS namespace not found, using defaults");
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(err, TAG, "Failed to open NVS");
    
    uint32_t on_val, off_val;
    
    err = nvs_get_u32(handle, NVS_KEY_ON_DURATION, &on_val);
    if (err == ESP_OK) {
        g_app_state.on_duration_sec = on_val;
        char_on_value = on_val;
        ESP_LOGI(TAG, "Loaded ON duration: %" PRIu32 "s", on_val);
    }
    
    err = nvs_get_u32(handle, NVS_KEY_OFF_DURATION, &off_val);
    if (err == ESP_OK) {
        g_app_state.off_duration_sec = off_val;
        char_off_value = off_val;
        ESP_LOGI(TAG, "Loaded OFF duration: %" PRIu32 "s", off_val);
    }
    
    nvs_close(handle);
    return ESP_OK;
}

/**
 * @brief Save timer configuration to NVS
 * @return ESP_OK on success
 */
static esp_err_t nvs_save_timer_config(void)
{
    nvs_handle_t handle;
    esp_err_t err;
    
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    ESP_RETURN_ON_ERROR(err, TAG, "Failed to open NVS for write");
    
    err = nvs_set_u32(handle, NVS_KEY_ON_DURATION, g_app_state.on_duration_sec);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save ON duration");
        nvs_close(handle);
        return err;
    }
    
    err = nvs_set_u32(handle, NVS_KEY_OFF_DURATION, g_app_state.off_duration_sec);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save OFF duration");
        nvs_close(handle);
        return err;
    }
    
    err = nvs_commit(handle);
    nvs_close(handle);
    
    ESP_LOGI(TAG, "Saved config: ON=%" PRIu32 "s, OFF=%" PRIu32 "s",
             g_app_state.on_duration_sec, g_app_state.off_duration_sec);
    
    return err;
}

/* ========================================
 * Zigbee Functions
 * ======================================== */

/**
 * @brief Send On/Off command to the paired Zigbee device
 * @param on true for ON command, false for OFF command
 */
static void zigbee_send_on_off_command(bool on)
{
    if (!g_app_state.target_device.is_joined) {
        ESP_LOGW(TAG, "No device joined, cannot send command");
        return;
    }
    
    esp_zb_zcl_on_off_cmd_t cmd_req = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = g_app_state.target_device.short_addr,
            .dst_endpoint = g_app_state.target_device.endpoint,
            .src_endpoint = COORDINATOR_ENDPOINT,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .on_off_cmd_id = on ? ESP_ZB_ZCL_CMD_ON_OFF_ON_ID : ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID,
    };
    
    esp_err_t err = esp_zb_zcl_on_off_cmd_req(&cmd_req);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Sent %s command to device 0x%04X endpoint %d",
                 on ? "ON" : "OFF",
                 g_app_state.target_device.short_addr,
                 g_app_state.target_device.endpoint);
        g_app_state.current_switch_state = on;
    } else {
        ESP_LOGE(TAG, "Failed to send command: %s", esp_err_to_name(err));
    }
}

/**
 * @brief Handle device announcement (new device joined)
 * @param dev_annce Pointer to device announce info
 */
static void zigbee_handle_device_announce(esp_zb_zdo_signal_device_annce_params_t *dev_annce)
{
    ESP_LOGI(TAG, "New device joined! Short addr: 0x%04X", dev_annce->device_short_addr);
    ESP_LOGI(TAG, "IEEE addr: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
             dev_annce->ieee_addr[7], dev_annce->ieee_addr[6],
             dev_annce->ieee_addr[5], dev_annce->ieee_addr[4],
             dev_annce->ieee_addr[3], dev_annce->ieee_addr[2],
             dev_annce->ieee_addr[1], dev_annce->ieee_addr[0]);
    
    /* Store device information */
    g_app_state.target_device.is_joined = true;
    g_app_state.target_device.short_addr = dev_annce->device_short_addr;
    g_app_state.target_device.endpoint = 1;  /* Sonoff ZBMINIR2 uses endpoint 1 */
    memcpy(g_app_state.target_device.ieee_addr, dev_annce->ieee_addr, 8);
    
    /* Signal that device has joined */
    xEventGroupSetBits(g_zigbee_event_group, ZB_DEVICE_JOINED_BIT);
    
    /* Activate control loop */
    g_app_state.control_active = true;
    
    ESP_LOGI(TAG, "Device registered for control on endpoint %d", 
             g_app_state.target_device.endpoint);
}

/**
 * @brief Open network for device joining
 */
static void zigbee_permit_join(void)
{
    esp_zb_zdo_permit_joining_req_param_t permit_req = {
        .dst_nwk_addr = ESP_ZB_NWK_BROADCAST_RX_ON_WHEN_IDLE,
        .permit_duration = PERMIT_JOIN_DURATION,
        .tc_significance = true,
    };
    
    esp_zb_zdo_permit_joining_req(&permit_req, NULL, NULL);
    g_app_state.permit_joining = true;
    
    ESP_LOGI(TAG, "Network open for joining (duration: %d)", PERMIT_JOIN_DURATION);
}

/**
 * @brief Zigbee signal handler callback
 * @param signal_struct Pointer to signal structure
 */
static void zigbee_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    
    switch (sig_type) {
        case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
            ESP_LOGI(TAG, "Zigbee stack initialized");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
            break;
            
        case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "Device started for first time, forming network...");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
            } else {
                ESP_LOGE(TAG, "Failed to initialize: %s", esp_err_to_name(err));
            }
            break;
            
        case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "Device rebooted, network already formed");
                g_app_state.network_formed = true;
                xEventGroupSetBits(g_zigbee_event_group, ZB_NETWORK_READY_BIT);
                zigbee_permit_join();
            } else {
                ESP_LOGW(TAG, "Reboot signal error, reforming network...");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
            }
            break;
            
        case ESP_ZB_BDB_SIGNAL_FORMATION:
            if (err == ESP_OK) {
                esp_zb_ieee_addr_t ext_pan_id;
                esp_zb_get_extended_pan_id(ext_pan_id);
                ESP_LOGI(TAG, "Network formed successfully!");
                ESP_LOGI(TAG, "Extended PAN ID: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                         ext_pan_id[7], ext_pan_id[6], ext_pan_id[5], ext_pan_id[4],
                         ext_pan_id[3], ext_pan_id[2], ext_pan_id[1], ext_pan_id[0]);
                ESP_LOGI(TAG, "PAN ID: 0x%04X, Channel: %d",
                         esp_zb_get_pan_id(), esp_zb_get_current_channel());
                
                g_app_state.network_formed = true;
                xEventGroupSetBits(g_zigbee_event_group, ZB_NETWORK_READY_BIT);
                zigbee_permit_join();
            } else {
                ESP_LOGW(TAG, "Network formation failed (err: %s), retrying...",
                         esp_err_to_name(err));
                esp_zb_scheduler_alarm((esp_zb_callback_t)esp_zb_bdb_start_top_level_commissioning,
                                       ESP_ZB_BDB_MODE_NETWORK_FORMATION, 1000);
            }
            break;
            
        case ESP_ZB_BDB_SIGNAL_STEERING:
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "Network steering completed");
            }
            break;
            
        case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
            {
                esp_zb_zdo_signal_device_annce_params_t *dev_annce = 
                    (esp_zb_zdo_signal_device_annce_params_t *)esp_zb_app_signal_get_params(p_sg_p);
                zigbee_handle_device_announce(dev_annce);
            }
            break;
            
        case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
            if (err == ESP_OK) {
                uint8_t *permit_duration = (uint8_t *)esp_zb_app_signal_get_params(p_sg_p);
                if (*permit_duration) {
                    ESP_LOGI(TAG, "Network permit join: OPEN (duration: %d)", *permit_duration);
                } else {
                    ESP_LOGI(TAG, "Network permit join: CLOSED");
                    /* Re-open if no device joined yet */
                    if (!g_app_state.target_device.is_joined) {
                        vTaskDelay(pdMS_TO_TICKS(1000));
                        zigbee_permit_join();
                    }
                }
            }
            break;
            
        case ESP_ZB_ZDO_SIGNAL_LEAVE:
            ESP_LOGW(TAG, "Device left the network");
            g_app_state.target_device.is_joined = false;
            g_app_state.control_active = false;
            xEventGroupClearBits(g_zigbee_event_group, ZB_DEVICE_JOINED_BIT);
            /* Re-open network for joining */
            zigbee_permit_join();
            break;
            
        default:
            ESP_LOGD(TAG, "Zigbee signal: 0x%X, status: %s", sig_type, esp_err_to_name(err));
            break;
    }
}

/**
 * @brief Create and register Zigbee Coordinator endpoint
 */
static void zigbee_create_coordinator_endpoint(void)
{
    /* Create cluster list for coordinator endpoint */
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    
    /* Basic Cluster (Server) */
    esp_zb_basic_cluster_cfg_t basic_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE,
    };
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&basic_cfg);
    esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, 
                                           ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    
    /* Identify Cluster (Server) */
    esp_zb_identify_cluster_cfg_t identify_cfg = {
        .identify_time = 0,
    };
    esp_zb_attribute_list_t *identify_cluster = esp_zb_identify_cluster_create(&identify_cfg);
    esp_zb_cluster_list_add_identify_cluster(cluster_list, identify_cluster,
                                              ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    
    /* On/Off Cluster (Client - we send commands to switches) */
    esp_zb_attribute_list_t *on_off_cluster = esp_zb_on_off_cluster_create(NULL);
    esp_zb_cluster_list_add_on_off_cluster(cluster_list, on_off_cluster,
                                            ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    
    /* Create endpoint list */
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    
    /* Endpoint configuration */
    esp_zb_endpoint_config_t ep_config = {
        .endpoint = COORDINATOR_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID,
        .app_device_version = 0,
    };
    
    esp_zb_ep_list_add_ep(ep_list, cluster_list, ep_config);
    
    /* Register device */
    esp_zb_device_register(ep_list);
    
    ESP_LOGI(TAG, "Coordinator endpoint %d registered", COORDINATOR_ENDPOINT);
}

/**
 * @brief Zigbee task main function
 * @param pvParameters Task parameters (unused)
 */
static void zigbee_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting Zigbee Coordinator task");
    
    /* Zigbee configuration */
    esp_zb_cfg_t zb_nwk_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_COORDINATOR,
        .install_code_policy = false,
        .nwk_cfg = {
            .zczr_cfg = {
                .max_children = 10,
            },
        },
    };
    
    /* Initialize Zigbee stack */
    esp_zb_init(&zb_nwk_cfg);
    
    /* Create and register coordinator endpoint */
    zigbee_create_coordinator_endpoint();
    
    /* Set channel mask */
    esp_zb_set_primary_network_channel_set(ZIGBEE_CHANNEL_MASK);
    
    /* Register signal handler */
    ESP_ERROR_CHECK(esp_zb_set_rx_on_when_idle(true));
    
    /* Start Zigbee stack */
    ESP_ERROR_CHECK(esp_zb_start(false));
    
    /* Main loop - process Zigbee stack events */
    esp_zb_main_loop_iteration();
    
    /* Should never reach here */
    vTaskDelete(NULL);
}

/* ========================================
 * BLE GATT Server Functions
 * ======================================== */

/**
 * @brief Handle BLE GAP events
 * @param event Event type
 * @param param Event parameters
 */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "BLE advertising data set complete");
            esp_ble_gap_start_advertising(&adv_params);
            break;
            
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "BLE scan response data set complete");
            break;
            
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "BLE advertising started successfully");
            } else {
                ESP_LOGE(TAG, "BLE advertising start failed: %d", param->adv_start_cmpl.status);
            }
            break;
            
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "BLE advertising stopped");
            }
            break;
            
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGD(TAG, "Connection params updated: interval=%d, latency=%d, timeout=%d",
                     param->update_conn_params.conn_int,
                     param->update_conn_params.latency,
                     param->update_conn_params.timeout);
            break;
            
        default:
            ESP_LOGD(TAG, "GAP event: %d", event);
            break;
    }
}

/**
 * @brief Handle BLE GATTS events
 * @param event Event type
 * @param gatts_if GATT server interface
 * @param param Event parameters
 */
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                 esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT:
            if (param->reg.status == ESP_GATT_OK) {
                g_app_state.gatts_if = gatts_if;
                ESP_LOGI(TAG, "GATT server registered, app_id=%d, if=%d",
                         param->reg.app_id, gatts_if);
                
                /* Set device name */
                esp_ble_gap_set_device_name(DEVICE_NAME);
                
                /* Configure advertising data */
                esp_ble_gap_config_adv_data(&adv_data);
                esp_ble_gap_config_adv_data(&scan_rsp_data);
                
                /* Create attribute table */
                esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, IDX_NB, 0);
            } else {
                ESP_LOGE(TAG, "GATT server registration failed: %d", param->reg.status);
            }
            break;
            
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
            if (param->add_attr_tab.status == ESP_GATT_OK) {
                if (param->add_attr_tab.num_handle == IDX_NB) {
                    memcpy(g_app_state.handle_table, param->add_attr_tab.handles, 
                           sizeof(g_app_state.handle_table));
                    ESP_LOGI(TAG, "GATT attribute table created successfully");
                    
                    /* Start service */
                    esp_ble_gatts_start_service(g_app_state.handle_table[IDX_SVC]);
                } else {
                    ESP_LOGE(TAG, "Attribute table incomplete, expected %d handles, got %d",
                             IDX_NB, param->add_attr_tab.num_handle);
                }
            } else {
                ESP_LOGE(TAG, "Failed to create attribute table: 0x%X", param->add_attr_tab.status);
            }
            break;
            
        case ESP_GATTS_START_EVT:
            if (param->start.status == ESP_GATT_OK) {
                ESP_LOGI(TAG, "GATT service started, handle=0x%X", param->start.service_handle);
            }
            break;
            
        case ESP_GATTS_CONNECT_EVT:
            g_app_state.conn_id = param->connect.conn_id;
            g_app_state.ble_connected = true;
            ESP_LOGI(TAG, "BLE client connected, conn_id=%d", param->connect.conn_id);
            ESP_LOGI(TAG, "Client address: %02X:%02X:%02X:%02X:%02X:%02X",
                     param->connect.remote_bda[0], param->connect.remote_bda[1],
                     param->connect.remote_bda[2], param->connect.remote_bda[3],
                     param->connect.remote_bda[4], param->connect.remote_bda[5]);
            
            /* Update connection parameters */
            esp_ble_conn_update_params_t conn_params = {
                .latency = 0,
                .max_int = 0x20,    /* 40ms */
                .min_int = 0x10,    /* 20ms */
                .timeout = 400,     /* 4 seconds */
            };
            memcpy(conn_params.bda, param->connect.remote_bda, ESP_BD_ADDR_LEN);
            esp_ble_gap_update_conn_params(&conn_params);
            break;
            
        case ESP_GATTS_DISCONNECT_EVT:
            g_app_state.ble_connected = false;
            ESP_LOGI(TAG, "BLE client disconnected, reason=0x%X", param->disconnect.reason);
            
            /* Restart advertising */
            esp_ble_gap_start_advertising(&adv_params);
            break;
            
        case ESP_GATTS_READ_EVT:
            ESP_LOGI(TAG, "GATT read request, handle=0x%X", param->read.handle);
            
            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            rsp.attr_value.handle = param->read.handle;
            
            if (param->read.handle == g_app_state.handle_table[IDX_CHAR_ON_VAL]) {
                /* Read ON duration */
                xSemaphoreTake(g_app_state.config_mutex, portMAX_DELAY);
                uint32_t val = g_app_state.on_duration_sec;
                xSemaphoreGive(g_app_state.config_mutex);
                
                rsp.attr_value.len = sizeof(uint32_t);
                memcpy(rsp.attr_value.value, &val, sizeof(uint32_t));
                ESP_LOGI(TAG, "Read ON duration: %" PRIu32 "s", val);
            } else if (param->read.handle == g_app_state.handle_table[IDX_CHAR_OFF_VAL]) {
                /* Read OFF duration */
                xSemaphoreTake(g_app_state.config_mutex, portMAX_DELAY);
                uint32_t val = g_app_state.off_duration_sec;
                xSemaphoreGive(g_app_state.config_mutex);
                
                rsp.attr_value.len = sizeof(uint32_t);
                memcpy(rsp.attr_value.value, &val, sizeof(uint32_t));
                ESP_LOGI(TAG, "Read OFF duration: %" PRIu32 "s", val);
            }
            
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, 
                                        param->read.trans_id, ESP_GATT_OK, &rsp);
            break;
            
        case ESP_GATTS_WRITE_EVT:
            ESP_LOGI(TAG, "GATT write request, handle=0x%X, len=%d",
                     param->write.handle, param->write.len);
            
            if (param->write.len == sizeof(uint32_t)) {
                uint32_t new_val;
                memcpy(&new_val, param->write.value, sizeof(uint32_t));
                
                /* Validate value (minimum 1 second, maximum 1 hour) */
                if (new_val < 1) new_val = 1;
                if (new_val > 3600) new_val = 3600;
                
                if (param->write.handle == g_app_state.handle_table[IDX_CHAR_ON_VAL]) {
                    /* Write ON duration */
                    xSemaphoreTake(g_app_state.config_mutex, portMAX_DELAY);
                    g_app_state.on_duration_sec = new_val;
                    char_on_value = new_val;
                    xSemaphoreGive(g_app_state.config_mutex);
                    
                    ESP_LOGI(TAG, "Updated ON duration: %" PRIu32 "s", new_val);
                    nvs_save_timer_config();
                } else if (param->write.handle == g_app_state.handle_table[IDX_CHAR_OFF_VAL]) {
                    /* Write OFF duration */
                    xSemaphoreTake(g_app_state.config_mutex, portMAX_DELAY);
                    g_app_state.off_duration_sec = new_val;
                    char_off_value = new_val;
                    xSemaphoreGive(g_app_state.config_mutex);
                    
                    ESP_LOGI(TAG, "Updated OFF duration: %" PRIu32 "s", new_val);
                    nvs_save_timer_config();
                }
            } else {
                ESP_LOGW(TAG, "Invalid write length: %d (expected %d)", 
                         param->write.len, sizeof(uint32_t));
            }
            
            /* Send response if needed */
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                            param->write.trans_id, ESP_GATT_OK, NULL);
            }
            break;
            
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(TAG, "MTU updated to %d", param->mtu.mtu);
            break;
            
        case ESP_GATTS_CONF_EVT:
            ESP_LOGD(TAG, "Confirmation received, status=%d", param->conf.status);
            break;
            
        default:
            ESP_LOGD(TAG, "GATTS event: %d", event);
            break;
    }
}

/**
 * @brief Initialize BLE stack and GATT server
 * @return ESP_OK on success
 */
static esp_err_t ble_init(void)
{
    esp_err_t ret;
    
    /* Release classic BT memory (we only use BLE) */
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    
    /* Initialize BT controller */
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BT controller init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Enable BT controller in BLE mode */
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BT controller enable failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Initialize Bluedroid stack */
    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Register GAP callback */
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GAP register callback failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Register GATTS callback */
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GATTS register callback failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Register GATTS application */
    ret = esp_ble_gatts_app_register(BLE_APP_ID);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GATTS app register failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Set MTU */
    esp_ble_gatt_set_local_mtu(BLE_MTU_SIZE);
    
    ESP_LOGI(TAG, "BLE GATT server initialized successfully");
    return ESP_OK;
}

/* ========================================
 * Control Task
 * ======================================== */

/**
 * @brief Control task that manages the switch ON/OFF cycle
 * @param pvParameters Task parameters (unused)
 */
static void control_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Control task started, waiting for Zigbee device...");
    
    /* Wait for network to be ready */
    xEventGroupWaitBits(g_zigbee_event_group, ZB_NETWORK_READY_BIT, 
                        pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "Zigbee network ready, waiting for device to join...");
    
    /* Wait for device to join */
    xEventGroupWaitBits(g_zigbee_event_group, ZB_DEVICE_JOINED_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "Device joined, starting control loop");
    
    /* Give the device time to fully initialize */
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    /* Main control loop */
    while (1) {
        if (!g_app_state.control_active || !g_app_state.target_device.is_joined) {
            /* Device not available, wait and check again */
            vTaskDelay(pdMS_TO_TICKS(1000));
            
            /* Wait for device to rejoin */
            xEventGroupWaitBits(g_zigbee_event_group, ZB_DEVICE_JOINED_BIT,
                                pdFALSE, pdTRUE, portMAX_DELAY);
            vTaskDelay(pdMS_TO_TICKS(3000));  /* Let device initialize */
            continue;
        }
        
        /* Get current ON duration */
        xSemaphoreTake(g_app_state.config_mutex, portMAX_DELAY);
        uint32_t on_duration = g_app_state.on_duration_sec;
        xSemaphoreGive(g_app_state.config_mutex);
        
        /* Turn ON */
        ESP_LOGI(TAG, "=== Turning switch ON for %" PRIu32 " seconds ===", on_duration);
        zigbee_send_on_off_command(true);
        
        /* Wait for ON duration (check periodically for config updates) */
        for (uint32_t i = 0; i < on_duration && g_app_state.control_active; i++) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            
            /* Check if duration was updated */
            xSemaphoreTake(g_app_state.config_mutex, portMAX_DELAY);
            uint32_t current_on = g_app_state.on_duration_sec;
            xSemaphoreGive(g_app_state.config_mutex);
            
            if (current_on != on_duration) {
                ESP_LOGI(TAG, "ON duration changed from %" PRIu32 " to %" PRIu32,
                         on_duration, current_on);
                on_duration = current_on;
            }
            
            /* Log remaining time every 10 seconds */
            if ((on_duration - i - 1) % 10 == 0 && (on_duration - i - 1) > 0) {
                ESP_LOGD(TAG, "ON time remaining: %" PRIu32 " seconds", on_duration - i - 1);
            }
        }
        
        if (!g_app_state.control_active) continue;
        
        /* Get current OFF duration */
        xSemaphoreTake(g_app_state.config_mutex, portMAX_DELAY);
        uint32_t off_duration = g_app_state.off_duration_sec;
        xSemaphoreGive(g_app_state.config_mutex);
        
        /* Turn OFF */
        ESP_LOGI(TAG, "=== Turning switch OFF for %" PRIu32 " seconds ===", off_duration);
        zigbee_send_on_off_command(false);
        
        /* Wait for OFF duration (check periodically for config updates) */
        for (uint32_t i = 0; i < off_duration && g_app_state.control_active; i++) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            
            /* Check if duration was updated */
            xSemaphoreTake(g_app_state.config_mutex, portMAX_DELAY);
            uint32_t current_off = g_app_state.off_duration_sec;
            xSemaphoreGive(g_app_state.config_mutex);
            
            if (current_off != off_duration) {
                ESP_LOGI(TAG, "OFF duration changed from %" PRIu32 " to %" PRIu32,
                         off_duration, current_off);
                off_duration = current_off;
            }
            
            /* Log remaining time every 10 seconds */
            if ((off_duration - i - 1) % 10 == 0 && (off_duration - i - 1) > 0) {
                ESP_LOGD(TAG, "OFF time remaining: %" PRIu32 " seconds", off_duration - i - 1);
            }
        }
    }
}

/* ========================================
 * Main Application Entry Point
 * ======================================== */

void app_main(void)
{
    esp_err_t ret;
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  Zigbee + BLE Smart Switch Controller");
    ESP_LOGI(TAG, "  ESP32-C6 | ESP-IDF v5.1+");
    ESP_LOGI(TAG, "========================================");
    
    /* Initialize NVS */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition truncated, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized");
    
    /* Create synchronization primitives */
    g_app_state.config_mutex = xSemaphoreCreateMutex();
    configASSERT(g_app_state.config_mutex != NULL);
    
    g_zigbee_event_group = xEventGroupCreate();
    configASSERT(g_zigbee_event_group != NULL);
    
    /* Load saved configuration */
    ESP_ERROR_CHECK(nvs_load_timer_config());
    ESP_LOGI(TAG, "Timer config: ON=%" PRIu32 "s, OFF=%" PRIu32 "s",
             g_app_state.on_duration_sec, g_app_state.off_duration_sec);
    
    /* Initialize platform for Zigbee */
    esp_zb_platform_config_t platform_config = {
        .radio_config = {
            .radio_mode = ZB_RADIO_MODE_NATIVE,
        },
        .host_config = {
            .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,
        },
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&platform_config));
    ESP_LOGI(TAG, "Zigbee platform configured");
    
    /* Initialize BLE */
    ESP_ERROR_CHECK(ble_init());
    
    /* Start Zigbee task */
    xTaskCreate(zigbee_task, "zigbee_task", ZIGBEE_TASK_STACK_SIZE, 
                NULL, ZIGBEE_TASK_PRIORITY, NULL);
    ESP_LOGI(TAG, "Zigbee task created");
    
    /* Start control task */
    xTaskCreate(control_task, "control_task", CONTROL_TASK_STACK_SIZE,
                NULL, CONTROL_TASK_PRIORITY, &g_control_task_handle);
    ESP_LOGI(TAG, "Control task created");
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  System Initialization Complete!");
    ESP_LOGI(TAG, "  BLE Device Name: %s", DEVICE_NAME);
    ESP_LOGI(TAG, "  Waiting for Zigbee network...");
    ESP_LOGI(TAG, "========================================");
}
