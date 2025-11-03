#ifndef SENSOR_H
#define SENSOR_H

#include "esp_err.h"
#include <stdbool.h>

/**
 * @brief IMU sensor data structure
 */
typedef struct {
    float accel_x;  // Acceleration in g
    float accel_y;
    float accel_z;
    float gyro_x;   // Angular velocity in deg/s
    float gyro_y;
    float gyro_z;
    float mag_x;    // Magnetic field in uT
    float mag_y;
    float mag_z;
    bool mag_valid; // Magnetometer data ready flag
} imu_data_t;

/**
 * @brief IMU orientation (Euler angles)
 */
typedef struct {
    float roll;     // degrees
    float pitch;    // degrees
    float yaw;      // degrees (heading)
} imu_orientation_t;

/**
 * @brief Initialize the ICM-20948 IMU
 * 
 * Sets up SPI communication, configures accelerometer, gyroscope,
 * and magnetometer for continuous operation.
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_init(void);

/**
 * @brief Read raw sensor data from IMU
 * 
 * @param data Pointer to structure to store sensor readings
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_read_sensors(imu_data_t *data);

/**
 * @brief Get current orientation from sensor fusion
 * 
 * @param orientation Pointer to structure to store orientation
 */
void imu_get_orientation(imu_orientation_t *orientation);

/**
 * @brief Update sensor fusion filter with new IMU data
 * 
 * @param data IMU sensor data
 * @param dt Time delta in seconds since last update
 */
void imu_update_fusion(const imu_data_t *data, float dt);

/**
 * @brief Calibrate gyroscope bias (device must be stationary)
 * 
 * @param samples Number of samples to average (recommend 1000+)
 * @return ESP_OK on success
 */
esp_err_t imu_calibrate_gyro(int samples);

/**
 * @brief Main IMU task - handles continuous sensor reading and fusion
 * 
 * @param pvParameters FreeRTOS task parameters (unused)
 */
void imu_task(void *pvParameters);

#endif // SENSOR_H
