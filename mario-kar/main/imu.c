#include <stdio.h>
#include "esp_log.h"
#include "mpu6050.h"
#include "driver/i2c.h"
#include "imu.h"
#include "imu_filters.h"

#define I2C_MASTER_SCL_IO 6        /* SCL pin on XIAO ESP32S3 */
#define I2C_MASTER_SDA_IO 5        /* SDA pin on XIAO ESP32S3 */
#define I2C_MASTER_NUM I2C_NUM_0   /* I2C port number */
#define I2C_MASTER_FREQ_HZ 100000  /* I2C master clock frequency */
#define MPU6050_ADDR 0x68          /* MPU6050 I2C address */

static const char *TAG = "IMU";
static mpu6050_handle_t mpu6050 = NULL;

// Latest sensor readings
static mpu6050_gyro_value_t raw_gyro = {0};

// Three separate FIR filters for gyro axes
static fir_filter_t gyro_x_filter;
static fir_filter_t gyro_y_filter;
static fir_filter_t gyro_z_filter;

// Latest filtered gyro data
static mpu6050_gyro_value_t filtered_gyro = {0};

esp_err_t imu_init(void) {
    // Configure I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create MPU6050 device handle
    mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_ADDR);
    if (mpu6050 == NULL) {
        ESP_LOGE(TAG, "MPU6050 create failed");
        return ESP_FAIL;
    }

    // Configure MPU6050
    ret = mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 config failed: %s", esp_err_to_name(ret));
        mpu6050_delete(mpu6050);
        return ret;
    }

    // Wake up MPU6050
    ret = mpu6050_wake_up(mpu6050);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 wake up failed: %s", esp_err_to_name(ret));
        mpu6050_delete(mpu6050);
        return ret;
    }

    ESP_LOGI(TAG, "MPU6050 initialized successfully");

    float default_coeffs[5] = {0.2, 0.2, 0.2, 0.2, 0.2};       // Values balanced equally
    // float responsive_coeffs[5] = {0.05, 0.1, 0.15, 0.3, 0.4};  // Heavily weighs toward recent values

    fir_filter_init(&gyro_x_filter, default_coeffs);
    fir_filter_init(&gyro_y_filter, default_coeffs);
    fir_filter_init(&gyro_z_filter, default_coeffs);

    ESP_LOGI(TAG, "MPU6050 initialized with gyro FIR filters");


    return ESP_OK;
}

void imu_task(void *pvParameters) {
    
    ESP_LOGI(TAG, "IMU task started");
    
    while (1) {
        
        // Get gyroscope data
        esp_err_t ret = mpu6050_get_gyro(mpu6050, &raw_gyro);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get gyro data: %s", esp_err_to_name(ret));
        }

        // Apply FIR filter to each axis
        filtered_gyro.gyro_x = fir_filter_update(&gyro_x_filter, raw_gyro.gyro_x);
        filtered_gyro.gyro_y = fir_filter_update(&gyro_y_filter, raw_gyro.gyro_y);
        filtered_gyro.gyro_z = fir_filter_update(&gyro_z_filter, raw_gyro.gyro_z);


        // Log sensor data
        ESP_LOGI(TAG, "Raw gyro: X=%.2f, Y=%.2f, Z=%.2f", 
                 raw_gyro.gyro_x, raw_gyro.gyro_y, raw_gyro.gyro_z);
        ESP_LOGI(TAG, "FIR gyro: X=%.2f, Y=%.2f, Z=%.2f", 
                 filtered_gyro.gyro_x, filtered_gyro.gyro_y, filtered_gyro.gyro_z);

        vTaskDelay(pdMS_TO_TICKS(100));  // 10hz update rate
    }
}