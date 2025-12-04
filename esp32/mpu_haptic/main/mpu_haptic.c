#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "drv2605l.h"
#include "mpu.h"
#include "driver/gpio.h"

// XIAO ESP32S3 I2C pins
#define I2C_SDA_PIN 5
#define I2C_SCL_PIN 6

#define BUTTON_GPIO GPIO_NUM_1
#define DEBOUNCE_TIME_MS 50

static const char *TAG = "MAIN";

void app_main(void)
{

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),  // Bit mask of the pin
        .mode = GPIO_MODE_INPUT,                 // Set as input mode
        .pull_up_en = GPIO_PULLUP_ENABLE,        // Enable pull-up resistor
        .pull_down_en = GPIO_PULLDOWN_DISABLE,   // Disable pull-down resistor
        .intr_type = GPIO_INTR_DISABLE           // Disable interrupt (we'll use polling)
    };
    gpio_config(&io_conf);
    ESP_LOGI(TAG, "Button GPIO configured on pin %d", BUTTON_GPIO);
    ESP_LOGI(TAG, "Press the button to test...");

    int last_state = 1;

    while (1) {
        // Read the GPIO level
        int current_state = gpio_get_level(BUTTON_GPIO);
        
        // Detect button press (transition from HIGH to LOW)
        if (last_state == 1 && current_state == 0) {
            ESP_LOGI(TAG, "Button PRESSED!");
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_TIME_MS));  // Simple debounce
        }
        // Detect button release (transition from LOW to HIGH)
        else if (last_state == 0 && current_state == 1) {
            ESP_LOGI(TAG, "Button RELEASED!");
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_TIME_MS));  // Simple debounce
        }
        
        last_state = current_state;
        
        // Small delay to avoid busy-waiting
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // ESP_LOGI(TAG, "Starting DRV2605L Haptic Demo");
    
    // Configure I2C
    // i2c_config_t conf = {
    //     .mode = I2C_MODE_MASTER,
    //     .sda_io_num = sda_pin,
    //     .scl_io_num = scl_pin,
    //     .sda_pullup_en = GPIO_PULLUP_ENABLE,
    //     .scl_pullup_en = GPIO_PULLUP_ENABLE,
    //     .master.clk_speed = 400000, // 400kHz
    // };
    
    // esp_err_t ret = i2c_param_config(i2c_port, &conf);
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "I2C config failed");
    //     return false;
    // }
    
    // ret = i2c_driver_install(i2c_port, conf.mode, 0, 0, 0);
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "I2C driver install failed");
    //     return false;
    // }
    
    // // Initialize the DRV2605L
    // if (!drv2605l_init(I2C_SDA_PIN, I2C_SCL_PIN)) {
    //     ESP_LOGE(TAG, "Failed to initialize DRV2605L");
    //     return;
    // }

    // if (mpu_init() == ESP_OK) {
    //     ESP_LOGI(TAG, "IMU initialized successfully");
        
    //     // Create IMU task
    //     xTaskCreate(mpu_task, "mpu_task", 4096, NULL, 5, NULL);
    // } else {
    //     ESP_LOGE(TAG, "IMU initialization failed!");
    // }
    
    // Use the ERM library (for standard vibration motors)
    // drv2605l_use_library(DRV2605_LIBRARY_TS2200A);

    // while (1) {
    //     drv2605l_play_effect(47);
    //     vTaskDelay(pdMS_TO_TICKS(2000));
    //     drv2605l_play_effect(70);
    //     vTaskDelay(pdMS_TO_TICKS(2000));
    //     drv2605l_play_effect(83);
    //     vTaskDelay(pdMS_TO_TICKS(2000));
    // }

}
