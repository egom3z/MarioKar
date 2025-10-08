#include <stdio.h>
#include "led.h"
#include "imu.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void) {
    led_init();
    
    // Initialize IMU
    if (imu_init() == ESP_OK) {
        // Create IMU task
        xTaskCreate(imu_task, "imu_task", 4096, NULL, 5, NULL);
    }
    
    xTaskCreate(led_task, "led_task", 2048, NULL, 5, NULL);
}