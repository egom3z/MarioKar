#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "mpu6050.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO    6
#define I2C_MASTER_SDA_IO    5
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   400000

static const char *TAG = "MPU6050_MADGWICK";

// Madgwick filter structure
typedef struct {
    float q0, q1, q2, q3;  // Quaternion elements
    float beta;             // Filter gain
    float sample_freq;      // Sample frequency in Hz
} madgwick_t;

// Initialize Madgwick filter
void madgwick_init(madgwick_t *filter, float sample_freq, float beta) {
    filter->q0 = 1.0f;
    filter->q1 = 0.0f;
    filter->q2 = 0.0f;
    filter->q3 = 0.0f;
    filter->beta = beta;
    filter->sample_freq = sample_freq;
}

// Fast inverse square root
float inv_sqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

// Madgwick filter update (6DOF - accel + gyro only)
void madgwick_update(madgwick_t *filter, float gx, float gy, float gz, 
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

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
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

        // Gradient decent algorithm corrective step
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
    recipNorm = inv_sqrt(filter->q0 * filter->q0 + filter->q1 * filter->q1 + filter->q2 * filter->q2 + filter->q3 * filter->q3);
    filter->q0 *= recipNorm;
    filter->q1 *= recipNorm;
    filter->q2 *= recipNorm;
    filter->q3 *= recipNorm;
}

// Convert quaternion to Euler angles (roll, pitch, yaw)
void madgwick_get_euler(madgwick_t *filter, float *roll, float *pitch, float *yaw) {
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

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (filter->q0 * filter->q3 + filter->q1 * filter->q2);
    float cosy_cosp = 1.0f - 2.0f * (filter->q2 * filter->q2 + filter->q3 * filter->q3);
    *yaw = atan2f(siny_cosp, cosy_cosp) * 57.2958f;
}

void app_main(void)
{
    esp_err_t ret;
    mpu6050_handle_t mpu6050 = NULL;
    
    // Configure I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C config failed");
        return;
    }
    
    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed");
        return;
    }
    
    // Initialize MPU6050
    mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
    if (mpu6050 == NULL) {
        ESP_LOGE(TAG, "MPU6050 create failed");
        return;
    }
    
    ret = mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 config failed");
        mpu6050_delete(mpu6050);
        return;
    }
    
    ret = mpu6050_wake_up(mpu6050);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 wake up failed");
        mpu6050_delete(mpu6050);
        return;
    }
    
    ESP_LOGI(TAG, "MPU6050 initialized successfully!");
    
    // Initialize Madgwick filter
    // sample_freq = 10Hz (100ms delay), beta = 0.1 (lower = smoother but slower response)
    madgwick_t filter;
    madgwick_init(&filter, 10.0f, 0.1f);
    
    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;
    float roll, pitch, yaw;
    
    ESP_LOGI(TAG, "Starting orientation tracking...");
    
    while (1) {
        // Read sensor values
        ret = mpu6050_get_acce(mpu6050, &acce);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read accelerometer");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        ret = mpu6050_get_gyro(mpu6050, &gyro);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read gyroscope");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        // Update Madgwick filter
        madgwick_update(&filter, 
                       gyro.gyro_x, gyro.gyro_y, gyro.gyro_z,
                       acce.acce_x, acce.acce_y, acce.acce_z);
        
        // Get Euler angles
        madgwick_get_euler(&filter, &roll, &pitch, &yaw);
        
        // Display orientation
        ESP_LOGI(TAG, "Roll: %6.2f°  Pitch: %6.2f°  Yaw: %6.2f°", roll, pitch, yaw);
        
        vTaskDelay(pdMS_TO_TICKS(100));  // 10Hz update rate
    }
    
    // Cleanup
    mpu6050_delete(mpu6050);
    i2c_driver_delete(I2C_MASTER_NUM);
}

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "mpu6050.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO    6      // Xiao ESP32S3 default SCL
#define I2C_MASTER_SDA_IO    5      // Xiao ESP32S3 default SDA
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   400000

static const char *TAG = "MPU6050_TEST";

// Simple low-pass filter for smoothing
typedef struct {
    float alpha;  // Filter coefficient (0-1, lower = more filtering)
    float prev_x, prev_y, prev_z;
} lpf_t;

void lpf_init(lpf_t *filter, float alpha) {
    filter->alpha = alpha;
    filter->prev_x = 0;
    filter->prev_y = 0;
    filter->prev_z = 0;
}

void lpf_update(lpf_t *filter, float *x, float *y, float *z) {
    filter->prev_x = filter->alpha * (*x) + (1 - filter->alpha) * filter->prev_x;
    filter->prev_y = filter->alpha * (*y) + (1 - filter->alpha) * filter->prev_y;
    filter->prev_z = filter->alpha * (*z) + (1 - filter->alpha) * filter->prev_z;
    
    *x = filter->prev_x;
    *y = filter->prev_y;
    *z = filter->prev_z;
}

void app_main(void)
{
    esp_err_t ret;
    mpu6050_handle_t mpu6050 = NULL;
    
    // Configure I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C config failed");
        return;
    }
    
    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed");
        return;
    }
    
    // Initialize MPU6050
    mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
    if (mpu6050 == NULL) {
        ESP_LOGE(TAG, "MPU6050 create failed");
        return;
    }
    
    ret = mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 config failed");
        mpu6050_delete(mpu6050);
        return;
    }
    
    ret = mpu6050_wake_up(mpu6050);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 wake up failed");
        mpu6050_delete(mpu6050);
        return;
    }
    
    ESP_LOGI(TAG, "MPU6050 initialized successfully!");
    
    // Initialize filters (alpha = 0.3 gives moderate smoothing)
    lpf_t accel_filter, gyro_filter;
    lpf_init(&accel_filter, 0.3);
    lpf_init(&gyro_filter, 0.3);
    
    // Main loop - read and display filtered data
    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;
    
    while (1) {
        // Read raw values
        ret = mpu6050_get_acce(mpu6050, &acce);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read accelerometer");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        ret = mpu6050_get_gyro(mpu6050, &gyro);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read gyroscope");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        // Apply low-pass filter
        float ax = acce.acce_x, ay = acce.acce_y, az = acce.acce_z;
        float gx = gyro.gyro_x, gy = gyro.gyro_y, gz = gyro.gyro_z;
        
        lpf_update(&accel_filter, &ax, &ay, &az);
        lpf_update(&gyro_filter, &gx, &gy, &gz);
        
        // Display filtered data
        ESP_LOGI(TAG, "Accel: X=%.2f Y=%.2f Z=%.2f g", ax, ay, az);
        // ESP_LOGI(TAG, "Gyro:  X=%.2f Y=%.2f Z=%.2f deg/s", gx, gy, gz);
        // ESP_LOGI(TAG, "---");
        
        vTaskDelay(pdMS_TO_TICKS(100));  // 10Hz update rate
    }
    
    // Cleanup (never reached in this example)
    mpu6050_delete(mpu6050);
    i2c_driver_delete(I2C_MASTER_NUM);
}