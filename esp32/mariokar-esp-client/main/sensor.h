#ifndef SENSOR_H
#define SENSOR_H

#include "esp_err.h"
#include <stdbool.h>

/**
 * @brief Dual IMU operation - both sensors active simultaneously
 * ICM-20948: SPI on GPIO 7,8,9,44
 * MPU-6050:  I2C on GPIO 5,6
 */
#define DUAL_IMU_MODE       1  // Enable both sensors at once

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
    float mag_x;    // Magnetic field in uT (only valid for ICM20948)
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

// =============================================================================
// ICM-20948 Functions (SPI)
// =============================================================================

esp_err_t icm20948_init(void);
esp_err_t second_init(void);
esp_err_t icm20948_read_sensors(imu_data_t *data);
void icm20948_get_orientation(imu_orientation_t *orientation);
void icm20948_update_fusion(const imu_data_t *data, float dt);
esp_err_t icm20948_calibrate_gyro(int samples);

// =============================================================================
// MPU-6050 Functions (I2C)
// =============================================================================

esp_err_t mpu6050_imu_init(void);
esp_err_t mpu6050_read_sensors(imu_data_t *data);
void mpu6050_get_orientation(imu_orientation_t *orientation);
void mpu6050_update_fusion(const imu_data_t *data, float dt);
esp_err_t mpu6050_calibrate_gyro(int samples);

#endif // SENSOR_H
