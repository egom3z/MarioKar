#ifndef MPU_H
#define MPU_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief Initialize the IMU (MPU6050)
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t mpu_init(void);

/**
 * @brief IMU task that continuously reads sensor data
 * 
 * @param pvParameters Task parameters (not used)
 */
void mpu_task(void *pvParameters);

#endif