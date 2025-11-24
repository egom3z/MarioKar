/**
 * @file sensor_mpu6050.c
 * @brief MPU6050 6-axis IMU driver with I2C interface
 * 
 * Features:
 * - I2C communication with MPU6050
 * - Madgwick AHRS sensor fusion (6-DOF, no magnetometer)
 * - Gyroscope calibration
 * 
 * Hardware Configuration for XIAO ESP32S3:
 * - SCL: GPIO 6
 * - SDA: GPIO 5
 */

#include "sensor.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu6050.h"
#include <string.h>
#include <math.h>

static const char *TAG = "MPU6050";

// =============================================================================
// Hardware Configuration
// =============================================================================

#define I2C_MASTER_SCL_IO    GPIO_NUM_6    // Xiao ESP32S3 default SCL
#define I2C_MASTER_SDA_IO    GPIO_NUM_5    // Xiao ESP32S3 default SDA
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   400000
#define MPU6050_ADDR         0x68

// =============================================================================
// Madgwick Filter Implementation
// =============================================================================

typedef struct {
    float q0, q1, q2, q3;  // Quaternion elements
    float beta;             // Filter gain
    float sample_freq;      // Sample frequency in Hz
} madgwick_t;

static madgwick_t g_filter;
static imu_orientation_t g_orientation = {0};
static mpu6050_handle_t g_mpu6050 = NULL;

// Gyro calibration offsets
static float g_gyro_offset_x = 0.0f;
static float g_gyro_offset_y = 0.0f;
static float g_gyro_offset_z = 0.0f;

// Fast inverse square root
static float inv_sqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

// Initialize Madgwick filter
static void madgwick_init(madgwick_t *filter, float sample_freq, float beta) {
    filter->q0 = 1.0f;
    filter->q1 = 0.0f;
    filter->q2 = 0.0f;
    filter->q3 = 0.0f;
    filter->beta = beta;
    filter->sample_freq = sample_freq;
}

// Madgwick filter update (6DOF - accel + gyro only)
static void madgwick_update(madgwick_t *filter, float gx, float gy, float gz, 
                           float ax, float ay, float az) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Convert gyroscope degrees/sec to radians/sec
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-filter->q1 * gx - filter->q2 * gy - filter->q3 * gz);
    qDot2 = 0.5f * (filter->q0 * gx + filter->q2 * gz - filter->q3 * gy);
    qDot3 = 0.5f * (filter->q0 * gy - filter->q1 * gz + filter->q3 * gx);
    qDot4 = 0.5f * (filter->q0 * gz + filter->q1 * gy - filter->q2 * gx);

    // Compute feedback only if accelerometer measurement valid
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalise accelerometer measurement
        recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * filter->q0;
        _2q1 = 2.0f * filter->q1;
        _2q2 = 2.0f * filter->q2;
        _2q3 = 2.0f * filter->q3;
        _4q0 = 4.0f * filter->q0;
        _4q1 = 4.0f * filter->q1;
        _4q2 = 4.0f * filter->q2;
        _8q1 = 8.0f * filter->q1;
        _8q2 = 8.0f * filter->q2;
        q0q0 = filter->q0 * filter->q0;
        q1q1 = filter->q1 * filter->q1;
        q2q2 = filter->q2 * filter->q2;
        q3q3 = filter->q3 * filter->q3;

        // Gradient descent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * filter->q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * filter->q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * filter->q3 - _2q1 * ax + 4.0f * q2q2 * filter->q3 - _2q2 * ay;
        recipNorm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= filter->beta * s0;
        qDot2 -= filter->beta * s1;
        qDot3 -= filter->beta * s2;
        qDot4 -= filter->beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    filter->q0 += qDot1 * (1.0f / filter->sample_freq);
    filter->q1 += qDot2 * (1.0f / filter->sample_freq);
    filter->q2 += qDot3 * (1.0f / filter->sample_freq);
    filter->q3 += qDot4 * (1.0f / filter->sample_freq);

    // Normalise quaternion
    recipNorm = inv_sqrt(filter->q0 * filter->q0 + filter->q1 * filter->q1 + 
                        filter->q2 * filter->q2 + filter->q3 * filter->q3);
    filter->q0 *= recipNorm;
    filter->q1 *= recipNorm;
    filter->q2 *= recipNorm;
    filter->q3 *= recipNorm;
}

// Convert quaternion to Euler angles (roll, pitch, yaw)
static void madgwick_get_euler(madgwick_t *filter, float *roll, float *pitch, float *yaw) {
    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (filter->q0 * filter->q1 + filter->q2 * filter->q3);
    float cosr_cosp = 1.0f - 2.0f * (filter->q1 * filter->q1 + filter->q2 * filter->q2);
    *roll = atan2f(sinr_cosp, cosr_cosp) * 57.2958f; // Convert to degrees

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (filter->q0 * filter->q2 - filter->q3 * filter->q1);
    if (fabsf(sinp) >= 1.0f)
        *pitch = copysignf(90.0f, sinp); // Use 90 degrees if out of range
    else
        *pitch = asinf(sinp) * 57.2958f;

    // Yaw (z-axis rotation) - Note: without magnetometer, yaw will drift
    float siny_cosp = 2.0f * (filter->q0 * filter->q3 + filter->q1 * filter->q2);
    float cosy_cosp = 1.0f - 2.0f * (filter->q2 * filter->q2 + filter->q3 * filter->q3);
    *yaw = atan2f(siny_cosp, cosy_cosp) * 57.2958f;
}

// =============================================================================
// Public API Implementation
// =============================================================================

esp_err_t mpu6050_imu_init(void) {
    ESP_LOGI(TAG, "Initializing MPU6050 IMU");
    
    // Initialize MPU6050
    g_mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_ADDR);
    if (g_mpu6050 == NULL) {
        ESP_LOGE(TAG, "MPU6050 create failed");
        i2c_driver_delete(I2C_MASTER_NUM);
        return ESP_FAIL;
    }
    
    // Configure: ±4g accel, ±500°/s gyro
    esp_err_t ret = mpu6050_config(g_mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 config failed: %s", esp_err_to_name(ret));
        mpu6050_delete(g_mpu6050);
        i2c_driver_delete(I2C_MASTER_NUM);
        return ret;
    }
    
    ret = mpu6050_wake_up(g_mpu6050);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 wake up failed: %s", esp_err_to_name(ret));
        mpu6050_delete(g_mpu6050);
        i2c_driver_delete(I2C_MASTER_NUM);
        return ret;
    }
    
    // Initialize Madgwick filter
    // 100Hz sample rate, beta = 0.1 (lower = smoother but slower response)
    madgwick_init(&g_filter, 100.0f, 0.1f);
    
    ESP_LOGI(TAG, "MPU6050 initialized successfully (I2C mode)");
    return ESP_OK;
}

esp_err_t mpu6050_read_sensors(imu_data_t *data) {
    if (g_mpu6050 == NULL) {
        return ESP_FAIL;
    }
    
    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;
    
    esp_err_t ret = mpu6050_get_acce(g_mpu6050, &acce);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = mpu6050_get_gyro(g_mpu6050, &gyro);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Convert to standard units and apply calibration
    data->accel_x = acce.acce_x;
    data->accel_y = acce.acce_y;
    data->accel_z = acce.acce_z;
    
    data->gyro_x = gyro.gyro_x - g_gyro_offset_x;
    data->gyro_y = gyro.gyro_y - g_gyro_offset_y;
    data->gyro_z = gyro.gyro_z - g_gyro_offset_z;
    
    // MPU6050 has no magnetometer
    data->mag_x = 0.0f;
    data->mag_y = 0.0f;
    data->mag_z = 0.0f;
    data->mag_valid = false;
    
    return ESP_OK;
}

void mpu6050_update_fusion(const imu_data_t *data, float dt) {
    // Update filter sample frequency based on actual dt
    if (dt > 0.0f && dt < 1.0f) {
        g_filter.sample_freq = 1.0f / dt;
    }
    
    // Update Madgwick filter (6-DOF mode, no magnetometer)
    madgwick_update(&g_filter,
                   data->gyro_x, data->gyro_y, data->gyro_z,
                   data->accel_x, data->accel_y, data->accel_z);
    
    // Get Euler angles
    madgwick_get_euler(&g_filter, 
                      &g_orientation.roll, 
                      &g_orientation.pitch, 
                      &g_orientation.yaw);
}

void mpu6050_get_orientation(imu_orientation_t *orientation) {
    memcpy(orientation, &g_orientation, sizeof(imu_orientation_t));
}

esp_err_t mpu6050_calibrate_gyro(int samples) {
    if (g_mpu6050 == NULL || samples <= 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Calibrating gyroscope... keep device stationary!");
    ESP_LOGI(TAG, "Collecting %d samples", samples);
    
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;
    int valid_samples = 0;
    
    for (int i = 0; i < samples; i++) {
        mpu6050_gyro_value_t gyro;
        esp_err_t ret = mpu6050_get_gyro(g_mpu6050, &gyro);
        
        if (ret == ESP_OK) {
            sum_x += gyro.gyro_x;
            sum_y += gyro.gyro_y;
            sum_z += gyro.gyro_z;
            valid_samples++;
        }
        
        vTaskDelay(pdMS_TO_TICKS(2));  // ~500Hz sampling
        
        // Progress indicator
        if (i % (samples / 10) == 0) {
            ESP_LOGI(TAG, "Progress: %d%%", (i * 100) / samples);
        }
    }
    
    if (valid_samples > 0) {
        g_gyro_offset_x = sum_x / valid_samples;
        g_gyro_offset_y = sum_y / valid_samples;
        g_gyro_offset_z = sum_z / valid_samples;
        
        ESP_LOGI(TAG, "Calibration complete (%d samples)", valid_samples);
        ESP_LOGI(TAG, "Gyro offsets: X=%.3f Y=%.3f Z=%.3f deg/s",
                 g_gyro_offset_x, g_gyro_offset_y, g_gyro_offset_z);
        return ESP_OK;
    }
    
    ESP_LOGE(TAG, "Calibration failed - no valid samples");
    return ESP_FAIL;
}

// Task removed - handled in main.c for dual IMU operation

