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
#include <math.h>    // logf
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_common_api.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/usb_serial_jtag.h"
#include "driver/gpio.h"
#include "sensor.h"
#include "drv2605l.h"
#include "driver/i2c.h"

#define I2C_MASTER_SCL_IO    GPIO_NUM_6    // Xiao ESP32S3 default SCL
#define I2C_MASTER_SDA_IO    GPIO_NUM_5    // Xiao ESP32S3 default SDA
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   400000

// GPIO configuration for reverse button
#define GPIO_REVERSE_BUTTON    GPIO_NUM_1
#define GPIO_INPUT_PIN_SEL     (1ULL << GPIO_REVERSE_BUTTON)

#define GATTC_TAG "GATTC_CLEAN"
#define PROFILE_NUM        1
#define PROFILE_APP_ID     0
#define INVALID_HANDLE     0

// IMU → BLE TX settings
#define BLE_TX_INTERVAL_MS      50

static const char *TAG = "MAIN";

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

// Shared state between dual IMU readers and BLE TX
typedef struct {
    uint16_t icm_roll;   // ICM-20948 roll PWM (1500-1650 µs, inverted: 0°=1650, 90°=1500)
    uint16_t icm_pitch;  // ICM-20948 pitch PWM (1500-1650 µs, inverted: 0°=1650, 90°=1500)
    uint16_t mpu_roll;   // MPU-6050 roll PWM (1000-2000 µs range)
    uint16_t mpu_pitch;  // MPU-6050 pitch PWM (1000-2000 µs range)
} dual_imu_pwm_t;

static volatile uint8_t haptic_intensity;

static volatile dual_imu_pwm_t s_latest_pwm = {1580, 1580, 1500, 1500}; // ICM center=1650 (0°), MPU center=1500
static volatile bool s_icm_ready = false;
static volatile bool s_mpu_ready = false;

// Reverse flag controlled by GPIO button
static volatile bool s_reverse_mode = false;
static volatile uint32_t s_reverse_toggle_count = 0;  // Counter for button presses

/**
 * @brief Persist reverse flag to NVS
 */
static void reverse_state_save(bool value)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("app", NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        uint8_t v = value ? 1 : 0;
        nvs_set_u8(nvs, "reverse", v);
        nvs_commit(nvs);
        nvs_close(nvs);
        ESP_LOGI(GATTC_TAG, "Reverse state saved: %s", value ? "ON" : "OFF");
    } else {
        ESP_LOGW(GATTC_TAG, "NVS open failed for save: %s", esp_err_to_name(err));
    }
}

/**
 * @brief Load reverse flag from NVS (defaults to OFF if not present)
 */
static void reverse_state_load(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("app", NVS_READONLY, &nvs);
    if (err == ESP_OK) {
        uint8_t v = 0;
        err = nvs_get_u8(nvs, "reverse", &v);
        nvs_close(nvs);
        if (err == ESP_OK) {
            s_reverse_mode = (v != 0);
            ESP_LOGI(GATTC_TAG, "Reverse state loaded: %s", s_reverse_mode ? "ON" : "OFF");
        } else if (err == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGI(GATTC_TAG, "Reverse state not found in NVS, defaulting to OFF");
            s_reverse_mode = false;
        } else {
            ESP_LOGW(GATTC_TAG, "NVS get failed: %s", esp_err_to_name(err));
        }
    } else {
        ESP_LOGW(GATTC_TAG, "NVS open failed for load: %s", esp_err_to_name(err));
    }
}

/**
 * @brief GPIO interrupt handler for reverse button
 */
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    static TickType_t last_tick = 0;
    TickType_t now = xTaskGetTickCountFromISR();

    // Simple debounce: ignore presses within 200ms of last press
    if ((now - last_tick) > pdMS_TO_TICKS(1000)) {
        s_reverse_mode = !s_reverse_mode;
        s_reverse_toggle_count++;  // Increment counter so we can detect changes
        last_tick = now;
    }
}

/**
 * @brief Initialize GPIO button for reverse control
 */
static void init_reverse_button(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,        // Interrupt on any edge (debug visibility)
        .mode = GPIO_MODE_INPUT,                // Set as input
        .pin_bit_mask = GPIO_INPUT_PIN_SEL,    // Bit mask of pin
        .pull_up_en = GPIO_PULLUP_ENABLE,      // Enable pull-up (button to GND)
        .pull_down_en = GPIO_PULLDOWN_DISABLE  // Disable pull-down
    };
    
    gpio_config(&io_conf);
    
    // Install GPIO ISR service
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    
    // Hook ISR handler for specific GPIO pin
    gpio_isr_handler_add(GPIO_REVERSE_BUTTON, gpio_isr_handler, NULL);
    
    ESP_LOGI(GATTC_TAG, "Reverse button initialized on GPIO%d", GPIO_REVERSE_BUTTON);
}

/**
 * @brief Task to monitor reverse button state changes
 */
static void reverse_monitor_task(void *arg) {
    uint32_t last_count = 0;
    int last_level = -1;
    
    while (1) {
        uint32_t current_count = s_reverse_toggle_count;
        int level = gpio_get_level(GPIO_REVERSE_BUTTON);
        
        // Print raw GPIO level on change (helps wiring/pull debug)
        if (level != last_level) {
            ESP_LOGI(GATTC_TAG, "GPIO%d level changed: %d", GPIO_REVERSE_BUTTON, level);
            last_level = level;
        }

        // Check if button was pressed (count changed)
        if (current_count != last_count) {
            ESP_LOGI(GATTC_TAG, "REVERSE BUTTON PRESSED! Mode: %s (Press #%lu)",
                     s_reverse_mode ? "ON" : "OFF", (unsigned long)current_count);
            // Persist new state
            reverse_state_save(s_reverse_mode);
            last_count = current_count;
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));  // Check every 50ms
    }
}

/**
 * @brief Map angle to servo PWM (1000 to 2000 microseconds) - MPU-6050
 */
static inline uint16_t angle_to_servo_pwm_full(float angle) {
    // Clamp angle to -90 to +90 range
    if (angle < -90.0f) angle = -90.0f;
    if (angle > 90.0f) angle = 90.0f;
    
    // Map: -90° -> 1000µs, 0° -> 1500µs, +90° -> 2000µs
    uint16_t pwm = (uint16_t)(1500.0f + (angle * 1000.0f / 180.0f));
    
    return pwm;
}

/**
 * @brief Map roll angle (-90° to +90°) to DRV2605L strength (0–255)
 *        Larger roll angle = stronger haptic intensity
 */
static inline uint8_t roll_to_drv2605_intensity(float roll_deg)
{
    // Clamp input
    if (roll_deg < -90.0f) roll_deg = -90.0f;
    if (roll_deg > 90.0f)  roll_deg = 90.0f;

    // Convert roll from [-90, +90] → [0, 1]
    float normalized = (roll_deg + 90.0f) / 180.0f;

    // Map to DRV2605 amplitude range [0, 255]
    uint8_t intensity = (uint8_t)(normalized * 255.0f);

    return intensity;
}


/**
 * @brief Map angle to servo PWM (ICM-20948, half span, mode-dependent)
 * - Normal: 0° -> 1610µs, 90° -> 1560µs (decreasing)
 * - Reverse: 0° -> 1200µs, 90° -> 1350µs (increasing)
 */
static inline uint16_t angle_to_servo_pwm_half(float angle) {
    // Clamp to 0-90° range (ignore negative angles)
    if (angle < 0.0f) {
        angle = 0.0f;
    }
    if (angle > 90.0f) {
        angle = 90.0f;
    }
    
    if (s_reverse_mode) {
        // Reverse mode: 0° -> 1350µs, 90° -> 1460µs
        float y = 1450.0f - 19.952f * logf(91.0f - angle);
        if (y > 1450.f) y = 1450.f;

        return (uint16_t) y;
    } else {
        // Normal mode (log curve): y = 1520 + 13.3013 * ln(91 - x), clamp to max 1580
        // x = angle in degrees [0,90]
        float y = 1550.0f + 11.0844 * logf(91.0f - angle);
        if (y > 1600.0f) y = 1600.0f;
        // Guard against minor float underflow
        return (uint16_t)y;
    }
}

/* ICM-20948 reader task (SPI) */
static void icm20948_reader_task(void *arg) {
    imu_data_t sensor_data;
    imu_orientation_t orientation;
    uint64_t last_time = esp_timer_get_time();

    ESP_LOGI(GATTC_TAG, "Initializing ICM-20948 (SPI)...");
    
    if (icm20948_init() != ESP_OK) {
        ESP_LOGE(GATTC_TAG, "ICM-20948 initialization failed!");
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(GATTC_TAG, "ICM-20948 initialized, calibrating gyroscope...");
    icm20948_calibrate_gyro(1000);
    
    s_icm_ready = true;
    ESP_LOGI(GATTC_TAG, "ICM-20948 ready");
    
    while (1) {
        uint64_t current_time = esp_timer_get_time();
        float dt = (current_time - last_time) / 1000000.0f;
        last_time = current_time;
        
        if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;
        
        if (icm20948_read_sensors(&sensor_data) == ESP_OK) {
            icm20948_update_fusion(&sensor_data, dt);
            icm20948_get_orientation(&orientation);
            
            s_latest_pwm.icm_roll = angle_to_servo_pwm_half(orientation.roll);
            s_latest_pwm.icm_pitch = angle_to_servo_pwm_half(orientation.pitch);
            
            static int log_counter = 0;
            if (++log_counter >= 100) {
                ESP_LOGI(GATTC_TAG, "ICM: Roll=%.1f° (PWM=%d) Pitch=%.1f° (PWM=%d)", 
                         orientation.roll, s_latest_pwm.icm_roll,
                         orientation.pitch, s_latest_pwm.icm_pitch);
                log_counter = 0;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));  // 100 Hz
    }
}

static esp_err_t i2c_init() {
    // Configure I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2C driver initialization success: %s", esp_err_to_name(ret));    
    return ESP_OK;
}

/* MPU-6050 reader task (I2C) */
static void mpu6050_reader_task(void *arg) {
    imu_data_t sensor_data;
    imu_orientation_t orientation;
    uint64_t last_time = esp_timer_get_time();

    ESP_LOGI(GATTC_TAG, "Initializing MPU-6050 (I2C)...");
    
    if (mpu6050_imu_init() != ESP_OK) {
        ESP_LOGE(GATTC_TAG, "MPU-6050 initialization failed!");
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(GATTC_TAG, "MPU-6050 initialized, calibrating gyroscope...");
    mpu6050_calibrate_gyro(1000);
    
    s_mpu_ready = true;
    ESP_LOGI(GATTC_TAG, "MPU-6050 ready");
    
    while (1) {
        uint64_t current_time = esp_timer_get_time();
        float dt = (current_time - last_time) / 1000000.0f;
        last_time = current_time;
        
        if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;
        
        if (mpu6050_read_sensors(&sensor_data) == ESP_OK) {
            mpu6050_update_fusion(&sensor_data, dt);
            mpu6050_get_orientation(&orientation);
            
            s_latest_pwm.mpu_roll = angle_to_servo_pwm_full(orientation.roll);
            s_latest_pwm.mpu_pitch = angle_to_servo_pwm_full(orientation.pitch);
            haptic_intensity = roll_to_drv2605_intensity(orientation.roll);
            
            static int log_counter = 0;
            if (++log_counter >= 100) {
                ESP_LOGI(GATTC_TAG, "MPU: Roll=%.1f° (PWM=%d) Pitch=%.1f° (PWM=%d)", 
                         orientation.roll, s_latest_pwm.mpu_roll,
                         orientation.pitch, s_latest_pwm.mpu_pitch);
                log_counter = 0;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));  // 100 Hz
    }
}

/* Haptic task (I2C) */
static void haptic_task(void *arg) {

    ESP_LOGI(GATTC_TAG, "Initializing DRV2605L Haptic (I2C)...");

    if (!drv2605l_init(I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO)) {
        ESP_LOGE(TAG, "Failed to initialize DRV2605L");
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(GATTC_TAG, "DRV2605L initialized");
    // drv2605l_use_library(DRV2605_LIBRARY_TS2200A);
    drv2605l_use_rtp();
    drv2605l_set_realtime_value(50);
    while (1) {
        // Wait for both IMUs to be ready
        if (!s_mpu_ready) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        // Copy current PWM values (atomic read)
        uint8_t haptic_data = haptic_intensity;
        // haptic_data = 230;

        // c += 1;
        drv2605l_set_realtime_value(haptic_data);
        
        static int log_counter = 0;
        if (++log_counter >= 100) {
            ESP_LOGI(GATTC_TAG, "HAPTIC: Intensity=%d", haptic_data);
            log_counter = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* BLE TX task: send 4 PWM values (8 bytes total) */
static void ble_tx_task(void *arg) {
    ESP_LOGI(GATTC_TAG, "BLE TX task started (interval %d ms, dual IMU mode)", BLE_TX_INTERVAL_MS);
    
    while (1) {
        // Wait for both IMUs to be ready
        if (!s_icm_ready || !s_mpu_ready) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        // Copy current PWM values (atomic read)
        dual_imu_pwm_t pwm_data = s_latest_pwm;

        if (is_connected &&
            gl_profile_tab[PROFILE_APP_ID].led_handle != INVALID_HANDLE &&
            gl_profile_tab[PROFILE_APP_ID].gattc_if != ESP_GATT_IF_NONE) {
            
            // Pack 4 PWM values as 8 bytes (little-endian)
            // Format: [ICM_ROLL_LSB, ICM_ROLL_MSB, ICM_PITCH_LSB, ICM_PITCH_MSB, 
            //          MPU_ROLL_LSB, MPU_ROLL_MSB, MPU_PITCH_LSB, MPU_PITCH_MSB]
            uint8_t pwm_bytes[8];
            pwm_bytes[0] = (uint8_t)(pwm_data.icm_roll & 0xFF);
            pwm_bytes[1] = (uint8_t)((pwm_data.icm_roll >> 8) & 0xFF);
            pwm_bytes[2] = (uint8_t)(pwm_data.icm_pitch & 0xFF);
            pwm_bytes[3] = (uint8_t)((pwm_data.icm_pitch >> 8) & 0xFF);
            pwm_bytes[4] = (uint8_t)(pwm_data.mpu_roll & 0xFF);
            pwm_bytes[5] = (uint8_t)((pwm_data.mpu_roll >> 8) & 0xFF);
            pwm_bytes[6] = (uint8_t)(pwm_data.mpu_pitch & 0xFF);
            pwm_bytes[7] = (uint8_t)((pwm_data.mpu_pitch >> 8) & 0xFF);
            
            esp_err_t err = esp_ble_gattc_write_char(
                gl_profile_tab[PROFILE_APP_ID].gattc_if,
                gl_profile_tab[PROFILE_APP_ID].conn_id,
                gl_profile_tab[PROFILE_APP_ID].led_handle,
                8,  // 8 bytes
                pwm_bytes,
                ESP_GATT_WRITE_TYPE_NO_RSP,
                ESP_GATT_AUTH_REQ_NONE);
            
            if (err != ESP_OK) {
                ESP_LOGW(GATTC_TAG, "BLE TX write failed: %s", esp_err_to_name(err));
            } else {
                // Log every 20 transmissions (~1 second)
                static int log_counter = 0;
                if (++log_counter >= 20) {
                    ESP_LOGI(GATTC_TAG, "BLE TX: ICM[R=%d P=%d] MPU[R=%d P=%d] µs | REVERSE: %s", 
                             pwm_data.icm_roll, pwm_data.icm_pitch,
                             pwm_data.mpu_roll, pwm_data.mpu_pitch,
                             s_reverse_mode ? "ON" : "OFF");
                    log_counter = 0;
                }
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

    // Load persisted reverse state
    reverse_state_load();

    // Initialize reverse button on GPIO1
    init_reverse_button();

    i2c_init();

    // Start dual IMU reader tasks, BLE TX task, and reverse monitor
    xTaskCreate(icm20948_reader_task, "icm20948", 4096, NULL, 5, NULL);
    xTaskCreate(mpu6050_reader_task, "mpu6050", 4096, NULL, 5, NULL);
    xTaskCreate(haptic_task, "haptic", 4096, NULL, 6, NULL);
    xTaskCreate(ble_tx_task, "ble_tx", 3072, NULL, 6, NULL);
    xTaskCreate(reverse_monitor_task, "reverse_mon", 4096, NULL, 4, NULL);

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
