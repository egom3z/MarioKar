#include "led.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "LED";

esp_err_t led_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << USER_LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "LED initialized on GPIO%d", USER_LED_GPIO);
    }
    return ret;
}

void led_task(void *pvParameters) {
    ESP_LOGI(TAG, "LED task started");
    
    while (1) {
        gpio_set_level(USER_LED_GPIO, 1);  // LED ON
        vTaskDelay(pdMS_TO_TICKS(100));
        
        gpio_set_level(USER_LED_GPIO, 0);  // LED OFF
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}