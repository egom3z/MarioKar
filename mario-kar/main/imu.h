#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include "esp_err.h"

// MPU6050 I2C Address
#define MPU6050_ADDR 0x68

// Default I2C pins for ESP32-S3 (you can modify these)
#define I2C_MASTER_SCL_IO 6
#define I2C_MASTER_SDA_IO 5
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000

// MPU6050 Register Addresses
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_GYRO_XOUT_H 0x43
#define MPU6050_REG_TEMP_OUT_H 0x41
#define MPU6050_REG_WHO_AM_I 0x75
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_GYRO_CONFIG 0x1B

// Accelerometer range settings
typedef enum {
    ACCEL_RANGE_2G = 0,
    ACCEL_RANGE_4G = 1,
    ACCEL_RANGE_8G = 2,
    ACCEL_RANGE_16G = 3
} mpu6050_accel_range_t;

// Gyroscope range settings
typedef enum {
    GYRO_RANGE_250DPS = 0,
    GYRO_RANGE_500DPS = 1,
    GYRO_RANGE_1000DPS = 2,
    GYRO_RANGE_2000DPS = 3
} mpu6050_gyro_range_t;

// IMU data structure
typedef struct {
    float accel_x;  // in g
    float accel_y;
    float accel_z;
    float gyro_x;   // in degrees/sec
    float gyro_y;
    float gyro_z;
    float temp;     // in Celsius
} imu_data_t;

/**
 * @brief Initialize the IMU (MPU6050)
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t imu_init(void);

/**
 * @brief Read all IMU data (accelerometer, gyroscope, temperature)
 * 
 * @param data Pointer to imu_data_t structure to store the data
 * @return esp_err_t ESP_OK on success
 */
esp_err_t imu_read_data(imu_data_t *data);

/**
 * @brief Read raw accelerometer data
 * 
 * @param accel_x Pointer to store X-axis data
 * @param accel_y Pointer to store Y-axis data
 * @param accel_z Pointer to store Z-axis data
 * @return esp_err_t ESP_OK on success
 */
esp_err_t imu_read_accel(int16_t *accel_x, int16_t *accel_y, int16_t *accel_z);

/**
 * @brief Read raw gyroscope data
 * 
 * @param gyro_x Pointer to store X-axis data
 * @param gyro_y Pointer to store Y-axis data
 * @param gyro_z Pointer to store Z-axis data
 * @return esp_err_t ESP_OK on success
 */
esp_err_t imu_read_gyro(int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z);

/**
 * @brief Read temperature from MPU6050
 * 
 * @param temp Pointer to store temperature in Celsius
 * @return esp_err_t ESP_OK on success
 */
esp_err_t imu_read_temp(float *temp);

/**
 * @brief Set accelerometer range
 * 
 * @param range Accelerometer range setting
 * @return esp_err_t ESP_OK on success
 */
esp_err_t imu_set_accel_range(mpu6050_accel_range_t range);

/**
 * @brief Set gyroscope range
 * 
 * @param range Gyroscope range setting
 * @return esp_err_t ESP_OK on success
 */
esp_err_t imu_set_gyro_range(mpu6050_gyro_range_t range);

/**
 * @brief FreeRTOS task to continuously read IMU data
 * 
 * @param pvParameters Task parameters (unused)
 */
void imu_task(void *pvParameters);

#endif // IMU_H