#include <stdio.h>
#include "led.h"
#include "imu.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MAIN";

void app_main(void) {
    // led_init();
    
    // Initialize IMU
    if (imu_init() == ESP_OK) {
        ESP_LOGI(TAG, "IMU initialized successfully");
        
        // Create IMU task
        xTaskCreate(imu_task, "imu_task", 4096, NULL, 5, NULL);
    } else {
        ESP_LOGE(TAG, "IMU initialization failed!");
    }
    
    // xTaskCreate(led_task, "led_task", 2048, NULL, 5, NULL);
}