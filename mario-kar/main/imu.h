#ifndef IMU_H
#define IMU_H

#include "esp_err.h"

// Initialize the IMU
esp_err_t imu_init(void);

// IMU task that reads gyroscope data
void imu_task(void *pvParameters);

// Structure to hold gyroscope data
typedef struct {
    float gyro_x;  // degrees/second
    float gyro_y;
    float gyro_z;
} imu_gyro_data_t;

#endif // IMU_H