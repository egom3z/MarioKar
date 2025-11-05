#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "drv2605l.h"

// XIAO ESP32S3 I2C pins
#define I2C_SDA_PIN 5
#define I2C_SCL_PIN 6

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "Starting DRV2605L Haptic Demo");
    
    // Initialize the DRV2605L
    if (!drv2605l_init(I2C_SDA_PIN, I2C_SCL_PIN)) {
        ESP_LOGE(TAG, "Failed to initialize DRV2605L");
        return;
    }
    
    // Use the ERM library (for standard vibration motors)
    drv2605l_use_library(DRV2605_LIBRARY_TS2200A);
    
    while (1) {
        for (int i = 1; i < 100; i ++) {
            drv2605l_play_effect(i);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        // // Example 1: Play a strong click
        // ESP_LOGI(TAG, "Strong Click");
        // drv2605l_play_effect(1);
        // vTaskDelay(pdMS_TO_TICKS(500));
        
        // // Example 2: Play a sharp click
        // ESP_LOGI(TAG, "Sharp Click");
        // drv2605l_play_effect(10);
        // vTaskDelay(pdMS_TO_TICKS(500));
        
        // // Example 3: Play a soft buzz
        // ESP_LOGI(TAG, "Soft Buzz");
        // drv2605l_play_effect(47);
        // vTaskDelay(pdMS_TO_TICKS(500));
        
        // // Example 4: Play a sequence of effects
        // ESP_LOGI(TAG, "Playing sequence");
        // uint8_t sequence[] = {1, 10, 1}; // Strong, Sharp, Strong
        // drv2605l_play_sequence(sequence, 3);
        // vTaskDelay(pdMS_TO_TICKS(1000));
        
        // Example 5: Real-time playback (RTP mode)
        // ESP_LOGI(TAG, "RTP Pulse");
        // drv2605l_use_rtp();
        
        // // Ramp up
        // ESP_LOGI(TAG, "ramp up");
        // for (int i = 0; i < 255; i += 5) {
        //     drv2605l_set_realtime_value(i);
        //     vTaskDelay(pdMS_TO_TICKS(10));
        // }

        // vTaskDelay(pdMS_TO_TICKS(1000));
        
        // ESP_LOGI(TAG, "ramp down");
        // // Ramp down
        // for (int i = 255; i >= 0; i -= 5) {
        //     drv2605l_set_realtime_value(i);
        //     vTaskDelay(pdMS_TO_TICKS(10));
        // }
        
        // Switch back to library mode
        // drv2605l_use_library(DRV2605_LIBRARY_TS2200A);
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}