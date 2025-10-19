#ifndef IMU_H
#define IMU_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief Initialize the IMU (MPU6050)
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_init(void);

/**
 * @brief IMU task that continuously reads sensor data
 * 
 * @param pvParameters Task parameters (not used)
 */
void imu_task(void *pvParameters);

/**
//  * @brief Get the latest accelerometer readings
//  * 
//  * @param acce_x Pointer to store X-axis acceleration
//  * @param acce_y Pointer to store Y-axis acceleration
//  * @param acce_z Pointer to store Z-axis acceleration
//  * @return ESP_OK on success, error code otherwise
//  */
// esp_err_t imu_get_accel(float *acce_x, float *acce_y, float *acce_z);

// /**
//  * @brief Get the latest gyroscope readings
//  * 
//  * @param gyro_x Pointer to store X-axis angular velocity
//  * @param gyro_y Pointer to store Y-axis angular velocity
//  * @param gyro_z Pointer to store Z-axis angular velocity
//  * @return ESP_OK on success, error code otherwise
//  */
// esp_err_t imu_get_gyro(float *gyro_x, float *gyro_y, float *gyro_z);

#endif // IMU_H