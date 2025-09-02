#include "ble.h"
#include "esp_bt.h"
#include "esp_log.h"
#include "esp_bt_main.h"
#include "esp_gatts_api.h"
#include "esp_gap_ble_api.h"
#include <string.h>

#define TAG "BLE"

// Nordic UART Service UUIDs
#define NUS_SERVICE_UUID      0x6E400001
#define NUS_CHAR_UUID_RX      0x6E400002  // Write from phone to ESP
#define NUS_CHAR_UUID_TX      0x6E400003  // Notify from ESP to phone


static uint16_t service_handle = 0;
static esp_gatt_if_t gatt_if = 0;
static uint16_t conn_id = 0;
static bool device_connected = false;

static uint8_t nus_service_uuid128[16] = {
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E
};

static uint8_t nus_char_uuid_rx128[16] = {
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E
};

static uint8_t nus_char_uuid_tx128[16] = {
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E
};

static esp_gatt_char_prop_t prop = ESP_GATT_CHAR_PROP_BIT_NOTIFY | ESP_GATT_CHAR_PROP_BIT_READ;
static esp_attr_value_t gatts_demo_char_val = {
    .attr_max_len = 100,
    .attr_len = sizeof("Waiting..."),
    .attr_value = (uint8_t *)"Waiting..."
};

static uint16_t char_handle = 0;

// Forward declaration
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param);

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    if (event == ESP_GAP_BLE_ADV_START_COMPLETE_EVT) {
        ESP_LOGI(TAG, "Advertising started.");
    }
}

void ble_init(void) {
    esp_err_t ret;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(0));

    esp_ble_adv_data_t adv_data = {
        .set_scan_rsp = false,
        .include_name = true,
        .include_txpower = true,
        .min_interval = 0x20,
        .max_interval = 0x40,
        .appearance = 0x00,
        .manufacturer_len = 0,
        .p_manufacturer_data = NULL,
        .service_data_len = 0,
        .p_service_data = NULL,
        .service_uuid_len = sizeof(nus_service_uuid128),
        .p_service_uuid = nus_service_uuid128,
        .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
    };

    ESP_ERROR_CHECK(esp_ble_gap_set_device_name("ESP32_Orientation"));
    ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&adv_data));
}

void ble_send_orientation(float roll, float pitch, float yaw) {
    if (!device_connected) return;

    char msg[64];
    int len = snprintf(msg, sizeof(msg), "%.3f,%.3f,%.3f", roll, pitch, yaw);

    esp_ble_gatts_set_attr_value(char_handle, len, (uint8_t *)msg);
    esp_ble_gatts_send_indicate(gatt_if, conn_id, char_handle, len, (uint8_t *)msg, false);
}

bool ble_is_connected(void) {
    return device_connected;
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t iface,
                                esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT: {
            service_handle = param->create.service_handle; 

            gatt_if = iface;

            esp_gatt_srvc_id_t service_id = {
                .is_primary = true,
                .id.inst_id = 0x00,
                .id.uuid.len = ESP_UUID_LEN_128,
                .id.uuid.uuid.uuid128 = {
                    0x9E, 0xCA, 0xDC, 0x24,
                    0x0E, 0xE5, 0xA9, 0xE0,
                    0x93, 0xF3, 0xA3, 0xB5,
                    0x01, 0x00, 0x40, 0x6E
                }
            };
            esp_ble_gatts_create_service(iface, &service_id, 7);
            break;
        }
        case ESP_GATTS_CREATE_EVT:
            // Add RX characteristic (Write only)
            esp_bt_uuid_t rx_uuid = {
                .len = ESP_UUID_LEN_128,
                .uuid = {.uuid128 = {
                    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                    0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E
                }}
            };
            esp_attr_control_t control = {.auto_rsp = ESP_GATT_RSP_BY_APP};
            esp_attr_value_t rx_val = {
                .attr_max_len = 100,
                .attr_len = sizeof(""),
                .attr_value = (uint8_t *)""
            };
            esp_ble_gatts_add_char(service_handle, &rx_uuid,
                ESP_GATT_PERM_WRITE, ESP_GATT_CHAR_PROP_BIT_WRITE, &rx_val, &control);

            // Add TX characteristic (Notify)
            esp_bt_uuid_t tx_uuid = {
                .len = ESP_UUID_LEN_128,
                .uuid = {.uuid128 = {
                    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                    0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E
                }}
            };
            esp_attr_value_t tx_val = {
                .attr_max_len = 100,
                .attr_len = sizeof("Waiting..."),
                .attr_value = (uint8_t *)"Waiting..."
            };
            esp_ble_gatts_add_char(service_handle, &tx_uuid,
                ESP_GATT_PERM_READ, ESP_GATT_CHAR_PROP_BIT_NOTIFY, &tx_val, &control);

            break;

        case ESP_GATTS_ADD_CHAR_EVT:
            char_handle = param->add_char.attr_handle;
            esp_ble_gatts_start_service(service_handle);
            esp_ble_gap_start_advertising(&(esp_ble_adv_params_t){
                .adv_int_min = 0x20,
                .adv_int_max = 0x40,
                .adv_type = ADV_TYPE_IND,
                .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
                .channel_map = ADV_CHNL_ALL,
                .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
            });
            break;

        case ESP_GATTS_CONNECT_EVT:
            device_connected = true;
            conn_id = param->connect.conn_id;
            ESP_LOGI(TAG, "Client connected.");
            break;

        case ESP_GATTS_DISCONNECT_EVT:
            device_connected = false;
            ESP_LOGI(TAG, "Client disconnected.");
            esp_ble_gap_start_advertising(&(esp_ble_adv_params_t){
                .adv_int_min = 0x20,
                .adv_int_max = 0x40,
                .adv_type = ADV_TYPE_IND,
                .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
                .channel_map = ADV_CHNL_ALL,
                .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
            });
            break;

        default:
            break;
    }
}
