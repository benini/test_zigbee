#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_check.h"
#include "esp_zigbee_core.h"

static const char *TAG = "ZIGBEE_TOGGLE";

/* Configurazione */
#define TOGGLE_INTERVAL_SEC         45
#define NETWORK_OPEN_DURATION_SEC   60
#define COORDINATOR_ENDPOINT        1
#define ESP_ZB_PRIMARY_CHANNEL_MASK (1l << 13)

/* Timer handle per il toggle periodico */
static TimerHandle_t s_toggle_timer = NULL;

/* Flag per sapere se la rete è formata */
static bool s_network_formed = false;

static void send_on_to_device(uint16_t short_addr, uint8_t endpoint)
{
    if (!s_network_formed) {
        ESP_LOGW(TAG, "Rete non ancora formata, skip ON");
        return;
    }

    ESP_LOGI(TAG, ">>> Invio ON a dispositivo 0x%04x (endpoint %d)", short_addr, endpoint);

    esp_zb_zcl_on_off_cmd_t cmd_req = {
        .zcl_basic_cmd = {
            .src_endpoint = COORDINATOR_ENDPOINT,
            .dst_addr_u = {
                .addr_short = short_addr,
            },
            .dst_endpoint = endpoint,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .on_off_cmd_id = ESP_ZB_ZCL_CMD_ON_OFF_ON_ID,   // comando ON
    };

    esp_err_t ret = esp_zb_zcl_on_off_cmd_req(&cmd_req);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Comando ON inviato con successo");
    } else {
        ESP_LOGE(TAG, "Errore invio ON: %s", esp_err_to_name(ret));
    }
}

/**
 * @brief Invia comando Toggle a tutti i dispositivi nella rete
 */
static void send_toggle_to_all(void)
{
    if (!s_network_formed) {
        ESP_LOGW(TAG, "Rete non ancora formata, skip toggle");
        return;
    }

    ESP_LOGI(TAG, ">>> Invio TOGGLE a tutti i dispositivi (broadcast 0xFFFF)");

    esp_zb_zcl_on_off_cmd_t cmd_req = {
        .zcl_basic_cmd = {
            .src_endpoint = COORDINATOR_ENDPOINT,
            .dst_addr_u = {
                .addr_short = 0xFFFF,
            },
            .dst_endpoint = 0xFF,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .on_off_cmd_id = ESP_ZB_ZCL_CMD_ON_OFF_TOGGLE_ID,
    };

    esp_err_t ret = esp_zb_zcl_on_off_cmd_req(&cmd_req);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Comando Toggle inviato con successo");
    } else {
        ESP_LOGE(TAG, "Errore invio Toggle: %s", esp_err_to_name(ret));
    }
}

/**
 * @brief Callback del timer per toggle periodico
 */
static void toggle_timer_callback(TimerHandle_t xTimer)
{
    send_toggle_to_all();
}

/**
 * @brief Apre la rete per permettere l'associazione di nuovi dispositivi
 */
static void open_network(uint8_t duration_sec)
{
    ESP_LOGI(TAG, "Apertura rete per %d secondi...", duration_sec);
    esp_zb_bdb_open_network(duration_sec);
}

/**
 * @brief Helper per avviare commissioning
 */
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

/**
 * @brief Handler principale dei segnali Zigbee
 */
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Inizializzazione stack Zigbee");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Device %s", 
                     sig_type == ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START ? 
                     "primo avvio" : "riavviato");
            
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Dispositivo nuovo, avvio formazione rete...");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
            } else {
                ESP_LOGI(TAG, "Rete già configurata, riconnessione...");
                s_network_formed = true;
                open_network(NETWORK_OPEN_DURATION_SEC);
                xTimerStart(s_toggle_timer, 0);
            }
        } else {
            ESP_LOGW(TAG, "Errore inizializzazione: %s", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_NETWORK_FORMATION, 1000);
        }
        break;

    case ESP_ZB_BDB_SIGNAL_FORMATION:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "=== RETE FORMATA ===");
            ESP_LOGI(TAG, "  PAN ID: 0x%04x", esp_zb_get_pan_id());
            ESP_LOGI(TAG, "  Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0]);
            ESP_LOGI(TAG, "  Canale: %d", esp_zb_get_current_channel());
            ESP_LOGI(TAG, "  Short Address: 0x%04x", esp_zb_get_short_address());
            
            s_network_formed = true;
            
            open_network(NETWORK_OPEN_DURATION_SEC);
            xTimerStart(s_toggle_timer, 0);
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            ESP_LOGW(TAG, "Formazione rete fallita, riprovo...");
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_NETWORK_FORMATION, 1000);
        }
        break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Steering completato");
        }
        break;

    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE: {
        esp_zb_zdo_signal_device_annce_params_t *dev_annce_params = 
            (esp_zb_zdo_signal_device_annce_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        ESP_LOGI(TAG, "*** NUOVO DISPOSITIVO ASSOCIATO ***");
        ESP_LOGI(TAG, "    Short Address: 0x%04x", dev_annce_params->device_short_addr);
        ESP_LOGI(TAG, "    IEEE Address: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
                 dev_annce_params->ieee_addr[7], dev_annce_params->ieee_addr[6],
                 dev_annce_params->ieee_addr[5], dev_annce_params->ieee_addr[4],
                 dev_annce_params->ieee_addr[3], dev_annce_params->ieee_addr[2],
                 dev_annce_params->ieee_addr[1], dev_annce_params->ieee_addr[0]);
        send_on_to_device(dev_annce_params->device_short_addr);
        break;
    }

    case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS: {
        uint8_t *permit_join_duration = (uint8_t *)esp_zb_app_signal_get_params(p_sg_p);
        if (*permit_join_duration) {
            ESP_LOGI(TAG, "Rete APERTA per %d secondi", *permit_join_duration);
        } else {
            ESP_LOGI(TAG, "Rete CHIUSA");
        }
        break;
    }

    default:
        ESP_LOGD(TAG, "Segnale ZDO: 0x%x, status: %s", sig_type, esp_err_to_name(err_status));
        break;
    }
}

/**
 * @brief Crea il data model del coordinatore
 */
static esp_zb_ep_list_t *create_coordinator_ep_list(void)
{
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = COORDINATOR_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID,
        .app_device_version = 0,
    };

    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

    /* Basic cluster (server) */
    esp_zb_basic_cluster_cfg_t basic_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE,
    };
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&basic_cfg);
    esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* Identify cluster (server) */
    esp_zb_identify_cluster_cfg_t identify_cfg = {
        .identify_time = 0,
    };
    esp_zb_attribute_list_t *identify_cluster = esp_zb_identify_cluster_create(&identify_cfg);
    esp_zb_cluster_list_add_identify_cluster(cluster_list, identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* On/Off cluster (client - per inviare comandi) */
    esp_zb_on_off_cluster_cfg_t on_off_cfg = {
        .on_off = ESP_ZB_ZCL_ON_OFF_ON_OFF_DEFAULT_VALUE,
    };
    esp_zb_attribute_list_t *on_off_cluster = esp_zb_on_off_cluster_create(&on_off_cfg);
    esp_zb_cluster_list_add_on_off_cluster(cluster_list, on_off_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_ep_list_add_ep(ep_list, cluster_list, endpoint_config);

    return ep_list;
}

/**
 * @brief Task principale Zigbee
 */
static void esp_zb_task(void *pvParameters)
{
    esp_zb_cfg_t zb_nwk_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_COORDINATOR,
        .install_code_policy = false,
        .nwk_cfg = {
            .zczr_cfg = {
                .max_children = 20,
            },
        },
    };
    esp_zb_init(&zb_nwk_cfg);

    esp_zb_ep_list_t *ep_list = create_coordinator_ep_list();
    esp_zb_device_register(ep_list);

    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    esp_zb_set_rx_on_when_idle(true);

    ESP_LOGI(TAG, "Avvio stack Zigbee...");
    ESP_ERROR_CHECK(esp_zb_start(false));

    /* Nuova API: usa esp_zb_stack_main_loop() invece di esp_zb_main_loop_iteration() */
    esp_zb_stack_main_loop();
}

void app_main(void)
{
    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "  Zigbee Toggle Broadcaster");
    ESP_LOGI(TAG, "  Toggle ogni %d secondi", TOGGLE_INTERVAL_SEC);
    ESP_LOGI(TAG, "  Pairing window: %d secondi", NETWORK_OPEN_DURATION_SEC);
    ESP_LOGI(TAG, "======================================");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Configurazione piattaforma per ESP32-C6 (radio nativa integrata) */
    esp_zb_platform_config_t config = {
        .radio_config = {
            .radio_mode = ZB_RADIO_MODE_NATIVE,
        },
        .host_config = {
            .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,
        },
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    s_toggle_timer = xTimerCreate(
        "toggle_timer",
        pdMS_TO_TICKS(TOGGLE_INTERVAL_SEC * 1000),
        pdTRUE,
        NULL,
        toggle_timer_callback
    );
    
    if (s_toggle_timer == NULL) {
        ESP_LOGE(TAG, "Errore creazione timer");
        return;
    }

    xTaskCreate(esp_zb_task, "zigbee_main", 4096, NULL, 5, NULL);
}

