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
#define MPU6050_ADDR         0x68          /* MPU6050 I2C address */


static const char *TAG = "MPU6050";
mpu6050_handle_t mpu6050 = NULL;

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

// Initialize filters (alpha = 0.3 gives moderate smoothing)
lpf_t accel_filter, gyro_filter;

mpu6050_acce_value_t acce;
mpu6050_gyro_value_t gyro;

esp_err_t mpu_init(void) {
    ESP_LOGI(TAG, "MPU6050 initialized successfully");

    esp_err_t ret;

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
        return ret;
    }
    
    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed");
        return ret;
    }
    
    // Initialize MPU6050
    mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_ADDR);
    if (mpu6050 == NULL) {
        ESP_LOGE(TAG, "MPU6050 create failed");
        return ret;
    }
    
    ret = mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 config failed");
        mpu6050_delete(mpu6050);
        return ret;
    }
    
    ret = mpu6050_wake_up(mpu6050);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 wake up failed");
        mpu6050_delete(mpu6050);
        return ret;
    }

    lpf_init(&accel_filter, 0.3);
    lpf_init(&gyro_filter, 0.3);
    
    return ESP_OK;
}

void mpu_task(void)
{   
    ESP_LOGI(TAG, "MPU task started");
    esp_err_t ret;
    
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