/*
 * GATT Client for STM32WB P2P Server (ESP-IDF)
 * Legacy GAP scanning (set_scan_params / start_scanning / stop_scanning)
 * Features:
 *  - Connect to "MyCST"
 *  - Discover service by 128-bit UUID
 *  - Find LED (write) and SWITCH (notify) characteristics
 *  - Enable notifications
 *  - Send LED ON command after CCCD is enabled
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>  // malloc/free
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/usb_serial_jtag.h"
#include "sensor.h"

#define GATTC_TAG "GATTC_CLEAN"
#define PROFILE_NUM        1
#define PROFILE_APP_ID     0
#define INVALID_HANDLE     0

// IMU → BLE TX settings
#define BLE_TX_INTERVAL_MS      50

// Target device name advertised by STM32WB
static char remote_device_name[] = "MyCST";

static bool is_connected = false;
static bool service_found = false;

static esp_gattc_char_elem_t *char_elem_result   = NULL;
static esp_gattc_descr_elem_t *descr_elem_result = NULL;

/* STM32WB P2P 128-bit UUIDs (little endian for ESP) */
static esp_bt_uuid_t p2p_service_uuid = {
    .len = ESP_UUID_LEN_128,
    .uuid.uuid128 = {
        0x8f, 0xe5, 0xb3, 0xd5, 0x2e, 0x7f, 0x4a, 0x98,
        0x2a, 0x48, 0x7a, 0xcc, 0x40, 0xfe, 0x00, 0x00
    }
};

static esp_bt_uuid_t led_char_uuid = {
    .len = ESP_UUID_LEN_128,
    .uuid.uuid128 = {
        0x19, 0xed, 0x82, 0xae, 0xed, 0x21, 0x4c, 0x9d,
        0x41, 0x45, 0x22, 0x8e, 0x41, 0xfe, 0x00, 0x00
    }
};

static esp_bt_uuid_t switch_char_uuid = {
    .len = ESP_UUID_LEN_128,
    .uuid.uuid128 = {
        0x19, 0xed, 0x82, 0xae, 0xed, 0x21, 0x4c, 0x9d,
        0x41, 0x45, 0x22, 0x8e, 0x42, 0xfe, 0x00, 0x00
    }
};

static esp_bt_uuid_t notify_descr_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG}
};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = 0x50,
    .scan_window = 0x30,
    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE
};

struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t led_handle;     // VALUE handle we will write to
    uint16_t switch_handle;  // DECLARATION handle (used to find CCCD)
    esp_bd_addr_t remote_bda;
};

static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_ID] = { .gattc_if = ESP_GATT_IF_NONE },
};

static bool uuid_equal(const esp_bt_uuid_t *a, const esp_bt_uuid_t *b) {
    if (a->len != b->len) { return false; }
    switch (a->len) {
        case ESP_UUID_LEN_16:  return a->uuid.uuid16 == b->uuid.uuid16;
        case ESP_UUID_LEN_32:  return a->uuid.uuid32 == b->uuid.uuid32;
        case ESP_UUID_LEN_128: return memcmp(a->uuid.uuid128, b->uuid.uuid128, ESP_UUID_LEN_128) == 0;
        default: return false;
    }
}

/* GAP callback */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        esp_ble_gap_start_scanning(30);
        break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = param;
        if (scan_result->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
            uint8_t adv_name_len = 0;

            uint8_t *adv_name = esp_ble_resolve_adv_data(
                scan_result->scan_rst.ble_adv,
                ESP_BLE_AD_TYPE_NAME_CMPL,
                &adv_name_len);

            if (adv_name && adv_name_len == strlen(remote_device_name) &&
                strncmp((char *)adv_name, remote_device_name, adv_name_len) == 0) {

                ESP_LOGI(GATTC_TAG, "Found target device %s", remote_device_name);
                esp_ble_gap_stop_scanning();

                esp_ble_gatt_creat_conn_params_t params = {0};
                memcpy(params.remote_bda, scan_result->scan_rst.bda, ESP_BD_ADDR_LEN);
                params.remote_addr_type = scan_result->scan_rst.ble_addr_type;
                params.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
                params.is_direct = true;

                esp_err_t err = esp_ble_gattc_enh_open(gl_profile_tab[PROFILE_APP_ID].gattc_if, &params);
                if (err != ESP_OK) {
                    ESP_LOGE(GATTC_TAG, "enh_open failed: %s", esp_err_to_name(err));
                }
            }
        }
        break;
    }

    default:
        break;
    }
}

/* GATTC events */
static void gattc_profile_event_handler(esp_gattc_cb_event_t event,
                                        esp_gatt_if_t gattc_if,
                                        esp_ble_gattc_cb_param_t *param) {
    switch (event) {
    case ESP_GATTC_REG_EVT:
        esp_ble_gap_set_scan_params(&ble_scan_params);
        break;

    case ESP_GATTC_CONNECT_EVT: {
        is_connected = true;
        gl_profile_tab[PROFILE_APP_ID].conn_id = param->connect.conn_id;
        memcpy(gl_profile_tab[PROFILE_APP_ID].remote_bda, param->connect.remote_bda, ESP_BD_ADDR_LEN);

        // Clean cache to avoid stale handles while iterating
        esp_err_t e = esp_ble_gattc_cache_clean(gl_profile_tab[PROFILE_APP_ID].remote_bda);
        ESP_LOGI(GATTC_TAG, "gattc_cache_clean: %s", esp_err_to_name(e));

        esp_ble_gattc_send_mtu_req(gattc_if, param->connect.conn_id);
        break;
    }

    case ESP_GATTC_DIS_SRVC_CMPL_EVT:
        esp_ble_gattc_search_service(gattc_if, param->dis_srvc_cmpl.conn_id, &p2p_service_uuid);
        break;

    case ESP_GATTC_SEARCH_RES_EVT:
        if (uuid_equal(&param->search_res.srvc_id.uuid, &p2p_service_uuid)) {
            service_found = true;
            gl_profile_tab[PROFILE_APP_ID].service_start_handle = param->search_res.start_handle;
            gl_profile_tab[PROFILE_APP_ID].service_end_handle   = param->search_res.end_handle;
            ESP_LOGI(GATTC_TAG, "P2P Service found: start=0x%04x end=0x%04x",
                     gl_profile_tab[PROFILE_APP_ID].service_start_handle,
                     gl_profile_tab[PROFILE_APP_ID].service_end_handle);
        }
        break;

    case ESP_GATTC_SEARCH_CMPL_EVT: {
        if (!service_found) {
            ESP_LOGW(GATTC_TAG, "Service not found — skipping characteristic discovery");
            return;
        }

        // Get capacity (IN) for our characteristics buffer
        uint16_t capacity = 0;
        esp_err_t err = esp_ble_gattc_get_attr_count(gattc_if, param->search_cmpl.conn_id,
                                                    ESP_GATT_DB_CHARACTERISTIC,
                                                    gl_profile_tab[PROFILE_APP_ID].service_start_handle,
                                                    gl_profile_tab[PROFILE_APP_ID].service_end_handle,
                                                    INVALID_HANDLE, &capacity);
        if (err != ESP_OK || capacity == 0) {
            ESP_LOGE(GATTC_TAG, "Failed to get characteristic count (err=%s, count=%d)",
                     esp_err_to_name(err), capacity);
            return;
        }

        char_elem_result = malloc(sizeof(esp_gattc_char_elem_t) * capacity);
        if (!char_elem_result) {
            ESP_LOGE(GATTC_TAG, "Malloc failed for char_elem_result");
            return;
        }

        // ---------- LED Characteristic (Write) ----------
        uint16_t found = capacity;  // IMPORTANT: pass capacity IN
        err = esp_ble_gattc_get_char_by_uuid(gattc_if,
                                            param->search_cmpl.conn_id,
                                            gl_profile_tab[PROFILE_APP_ID].service_start_handle,
                                            gl_profile_tab[PROFILE_APP_ID].service_end_handle,
                                            led_char_uuid,
                                            char_elem_result,
                                            &found);
        if (err == ESP_OK && found > 0) {
            // STM32WB: value handle = declaration handle + 1
            gl_profile_tab[PROFILE_APP_ID].led_handle = char_elem_result[0].char_handle;
            ESP_LOGI(GATTC_TAG, "LED value handle 0x%04x", gl_profile_tab[PROFILE_APP_ID].led_handle);
        } else {
            ESP_LOGE(GATTC_TAG, "LED characteristic not found (err=%s, found=%d)",
                     esp_err_to_name(err), found);
        }

        // ---------- SWITCH Characteristic (Notify) ----------
        found = capacity;  // reset capacity for second query
        err = esp_ble_gattc_get_char_by_uuid(gattc_if,
                                            param->search_cmpl.conn_id,
                                            gl_profile_tab[PROFILE_APP_ID].service_start_handle,
                                            gl_profile_tab[PROFILE_APP_ID].service_end_handle,
                                            switch_char_uuid,
                                            char_elem_result,
                                            &found);
        if (err == ESP_OK && found > 0) {
            gl_profile_tab[PROFILE_APP_ID].switch_handle = char_elem_result[0].char_handle; // decl handle
            ESP_LOGI(GATTC_TAG, "SWITCH decl handle 0x%04x", gl_profile_tab[PROFILE_APP_ID].switch_handle);

            err = esp_ble_gattc_register_for_notify(gattc_if,
                                                    gl_profile_tab[PROFILE_APP_ID].remote_bda,
                                                    gl_profile_tab[PROFILE_APP_ID].switch_handle);
            if (err != ESP_OK) {
                ESP_LOGE(GATTC_TAG, "Failed to register for notifications (err=%s)",
                         esp_err_to_name(err));
            }
        } else {
            ESP_LOGW(GATTC_TAG, "SWITCH characteristic not found (err=%s, found=%d)",
                     esp_err_to_name(err), found);
        }

        free(char_elem_result);
        char_elem_result = NULL;
        break;
    }

    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        // Find CCCD and enable notifications for SWITCH
        uint16_t capacity = 0;
        esp_err_t err = esp_ble_gattc_get_attr_count(gattc_if,
                                                    gl_profile_tab[PROFILE_APP_ID].conn_id,
                                                    ESP_GATT_DB_DESCRIPTOR,
                                                    gl_profile_tab[PROFILE_APP_ID].service_start_handle,
                                                    gl_profile_tab[PROFILE_APP_ID].service_end_handle,
                                                    gl_profile_tab[PROFILE_APP_ID].switch_handle,
                                                    &capacity);
        if (err != ESP_OK || capacity == 0) {
            ESP_LOGE(GATTC_TAG, "No CCCD descriptor found (err=%s, count=%d)",
                     esp_err_to_name(err), capacity);
            break;
        }

        descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * capacity);
        if (!descr_elem_result) {
            ESP_LOGE(GATTC_TAG, "Malloc failed for descr_elem_result");
            break;
        }

        uint16_t found = capacity; // IN = capacity
        err = esp_ble_gattc_get_descr_by_char_handle(gattc_if,
                                                    gl_profile_tab[PROFILE_APP_ID].conn_id,
                                                    gl_profile_tab[PROFILE_APP_ID].switch_handle,
                                                    notify_descr_uuid,
                                                    descr_elem_result,
                                                    &found);
        if (err != ESP_OK || found == 0) {
            ESP_LOGE(GATTC_TAG, "Failed to get descriptor handle (err=%s, found=%d)",
                     esp_err_to_name(err), found);
            free(descr_elem_result);
            break;
        }

        uint16_t notify_en = 1;
        err = esp_ble_gattc_write_char_descr(gattc_if,
                                            gl_profile_tab[PROFILE_APP_ID].conn_id,
                                            descr_elem_result[0].handle,
                                            sizeof(notify_en),
                                            (uint8_t *)&notify_en,
                                            ESP_GATT_WRITE_TYPE_RSP,
                                            ESP_GATT_AUTH_REQ_NONE);
        if (err != ESP_OK) {
            ESP_LOGE(GATTC_TAG, "Failed to write CCCD (err=%s)", esp_err_to_name(err));
        } else {
            ESP_LOGI(GATTC_TAG, "CCCD write request sent to handle=0x%04x",
                     descr_elem_result[0].handle);
        }

        free(descr_elem_result);
        descr_elem_result = NULL;
        break;
    }

    case ESP_GATTC_WRITE_DESCR_EVT:
        ESP_LOGI(GATTC_TAG, "Notifications enabled.");
        break;

    case ESP_GATTC_NOTIFY_EVT:
        ESP_LOGI(GATTC_TAG, "SWITCH Notification (%d bytes):", param->notify.value_len);
        ESP_LOG_BUFFER_HEX(GATTC_TAG, param->notify.value, param->notify.value_len);
        break;

    case ESP_GATTC_DISCONNECT_EVT:
        ESP_LOGI(GATTC_TAG, "Disconnected");
        is_connected = false;
        service_found = false;
        gl_profile_tab[PROFILE_APP_ID].led_handle = INVALID_HANDLE;
        gl_profile_tab[PROFILE_APP_ID].switch_handle = INVALID_HANDLE;
        break;

    case ESP_GATTC_CFG_MTU_EVT:
        ESP_LOGI(GATTC_TAG, "MTU configured to %d", param->cfg_mtu.mtu);
        esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, &p2p_service_uuid);
        break;

    default:
        break;
    }
}

/* Dispatcher */
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                         esp_ble_gattc_cb_param_t *param) {
    if (event == ESP_GATTC_REG_EVT)
        gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;

    if (gattc_if == ESP_GATT_IF_NONE ||
        gattc_if == gl_profile_tab[PROFILE_APP_ID].gattc_if)
        gattc_profile_event_handler(event, gattc_if, param);
}

// Shared state between IMU reader and BLE TX
static volatile uint16_t s_latest_pwm = 1500;  // Center position (0 degrees)
static volatile bool s_imu_ready = false;

/**
 * @brief Map angle (-90 to +90 degrees) to servo PWM (1000 to 2000 microseconds)
 */
static inline uint16_t angle_to_servo_pwm(float angle) {
    // Clamp angle to -90 to +90 range
    if (angle < -90.0f) angle = -90.0f;
    if (angle > 90.0f) angle = 90.0f;
    
    // Map: -90° -> 1000µs, 0° -> 1500µs, +90° -> 2000µs
    uint16_t pwm = (uint16_t)(1500.0f + (angle * 1000.0f / 180.0f));
    
    return pwm;
}

/* IMU reader: read sensor data and convert to servo PWM */
static void imu_reader_task(void *arg) {
    imu_data_t sensor_data;
    imu_orientation_t orientation;
    uint64_t last_time = esp_timer_get_time();

    ESP_LOGI(GATTC_TAG, "Initializing IMU...");
    
    if (imu_init() != ESP_OK) {
        ESP_LOGE(GATTC_TAG, "IMU initialization failed!");
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(GATTC_TAG, "IMU initialized, calibrating gyroscope...");
    imu_calibrate_gyro(1000);
    
    s_imu_ready = true;
    ESP_LOGI(GATTC_TAG, "IMU ready, starting orientation tracking");
    
    while (1) {
        // Calculate time delta
        uint64_t current_time = esp_timer_get_time();
        float dt = (current_time - last_time) / 1000000.0f;
        last_time = current_time;
        
        // Clamp dt to reasonable range
        if (dt <= 0.0f || dt > 0.1f) {
            dt = 0.01f;
        }
        
        // Read sensors and update fusion
        if (imu_read_sensors(&sensor_data) == ESP_OK) {
            imu_update_fusion(&sensor_data, dt);
            imu_get_orientation(&orientation);
            
            // Use ROLL for steering control (you can change to pitch or yaw if needed)
            s_latest_pwm = angle_to_servo_pwm(orientation.roll);
            
            // Log every second
            static int log_counter = 0;
            if (++log_counter >= 100) {
                ESP_LOGI(GATTC_TAG, "IMU: Roll=%.1f° -> PWM=%d µs", 
                         orientation.roll, s_latest_pwm);
                log_counter = 0;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));  // 100 Hz
    }
}

/* BLE TX task: periodically send PWM value (1000-2000) as 2 bytes */
static void ble_tx_task(void *arg) {
    uint16_t last_tx = 0xFFFF; // sentinel to log on first send
    ESP_LOGI(GATTC_TAG, "BLE TX task started (interval %d ms)", BLE_TX_INTERVAL_MS);
    
    while (1) {
        // Wait for IMU to be ready
        if (!s_imu_ready) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        uint16_t pwm_to_send = s_latest_pwm;

        if (is_connected &&
            gl_profile_tab[PROFILE_APP_ID].led_handle != INVALID_HANDLE &&
            gl_profile_tab[PROFILE_APP_ID].gattc_if != ESP_GATT_IF_NONE) {
            
            // Send PWM as 2 bytes (little-endian: LSB first, MSB second)
            uint8_t pwm_bytes[2];
            pwm_bytes[0] = (uint8_t)(pwm_to_send & 0xFF);         // LSB
            pwm_bytes[1] = (uint8_t)((pwm_to_send >> 8) & 0xFF);  // MSB
            
            esp_err_t err = esp_ble_gattc_write_char(
                gl_profile_tab[PROFILE_APP_ID].gattc_if,
                gl_profile_tab[PROFILE_APP_ID].conn_id,
                gl_profile_tab[PROFILE_APP_ID].led_handle,
                2,  // 2 bytes
                pwm_bytes,
                ESP_GATT_WRITE_TYPE_NO_RSP,
                ESP_GATT_AUTH_REQ_NONE);
            
            if (err != ESP_OK) {
                ESP_LOGW(GATTC_TAG, "BLE TX write failed: %s", esp_err_to_name(err));
            } else if (pwm_to_send != last_tx) {
                ESP_LOGI(GATTC_TAG, "BLE TX: PWM=%d µs [0x%02X 0x%02X]", 
                         pwm_to_send, pwm_bytes[0], pwm_bytes[1]);
                last_tx = pwm_to_send;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(BLE_TX_INTERVAL_MS));
    }
}

/* Main entry */
void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // Start IMU reader and BLE TX tasks
    xTaskCreate(imu_reader_task, "imu_reader", 4096, NULL, 5, NULL);
    xTaskCreate(ble_tx_task, "ble_tx", 3072, NULL, 6, NULL);

    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_bluedroid_init();
    esp_bluedroid_enable();

    esp_ble_gap_register_callback(esp_gap_cb);
    esp_ble_gattc_register_callback(esp_gattc_cb);
    esp_ble_gattc_app_register(PROFILE_APP_ID);
    esp_ble_gatt_set_local_mtu(500);
}
