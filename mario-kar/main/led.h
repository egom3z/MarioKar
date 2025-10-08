#ifndef LED_H
#define LED_H

#include "esp_err.h"

#define USER_LED_GPIO 21

esp_err_t led_init(void);
void led_task(void *pvParameters);

#endif