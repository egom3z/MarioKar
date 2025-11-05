#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "drv2605l.h"
#include "mpu.h"

// XIAO ESP32S3 I2C pins
#define I2C_SDA_PIN 5
#define I2C_SCL_PIN 6

static const char *TAG = "MAIN";

void app_main(void)
{
    // ESP_LOGI(TAG, "Starting DRV2605L Haptic Demo");
    
    // Initialize the DRV2605L
    // if (!drv2605l_init(I2C_SDA_PIN, I2C_SCL_PIN)) {
    //     ESP_LOGE(TAG, "Failed to initialize DRV2605L");
    //     return;
    // }

    // Use the ERM library (for standard vibration motors)
    // drv2605l_use_library(DRV2605_LIBRARY_TS2200A);

    if (mpu_init() == ESP_OK) {
        ESP_LOGI(TAG, "IMU initialized successfully");
        
        // Create IMU task
        xTaskCreate(mpu_task, "mpu_task", 4096, NULL, 5, NULL);
    } else {
        ESP_LOGE(TAG, "IMU initialization failed!");
    }

    // while (1) {
    //     drv2605l_play_effect(47);
    //     vTaskDelay(pdMS_TO_TICKS(2000));
    //     drv2605l_play_effect(70);
    //     vTaskDelay(pdMS_TO_TICKS(2000));
    //     drv2605l_play_effect(83);
    //     vTaskDelay(pdMS_TO_TICKS(2000));
    // }

}
