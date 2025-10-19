#include <stdio.h>
#include "esp_log.h"
#include "mpu6050.h"
#include "driver/i2c.h"
#include "imu.h"

#define I2C_MASTER_SCL_IO 6        /* SCL pin on XIAO ESP32S3 */
#define I2C_MASTER_SDA_IO 5        /* SDA pin on XIAO ESP32S3 */
#define I2C_MASTER_NUM I2C_NUM_0   /* I2C port number */
#define I2C_MASTER_FREQ_HZ 100000  /* I2C master clock frequency */
#define MPU6050_ADDR 0x68          /* MPU6050 I2C address */

static const char *TAG = "IMU";
static mpu6050_handle_t mpu6050 = NULL;

// Latest sensor readings
static mpu6050_acce_value_t latest_acce = {0};
static mpu6050_gyro_value_t latest_gyro = {0};

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
    return ESP_OK;
}

void imu_task(void *pvParameters) {
    // mpu6050_temp_value_t temp;
    
    ESP_LOGI(TAG, "IMU task started");
    
    while (1) {
        // Get accelerometer data
        // esp_err_t ret = mpu6050_get_acce(mpu6050, &latest_acce);
        // if (ret != ESP_OK) {
        //     ESP_LOGE(TAG, "Failed to get accel data: %s", esp_err_to_name(ret));
        // }
        
        // Get gyroscope data
        esp_err_t ret = mpu6050_get_gyro(mpu6050, &latest_gyro);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get gyro data: %s", esp_err_to_name(ret));
        }
        
        // Get temperature data (optional)
        // ret = mpu6050_get_temp(mpu6050, &temp);
        // if (ret != ESP_OK) {
        //     ESP_LOGE(TAG, "Failed to get temp data: %s", esp_err_to_name(ret));
        // }

        // Log sensor data
        // ESP_LOGI(TAG, "Accel: X=%.2f, Y=%.2f, Z=%.2f", 
        //          latest_acce.acce_x, latest_acce.acce_y, latest_acce.acce_z);
        ESP_LOGI(TAG, " ");
        ESP_LOGI(TAG, "Raw gyro: X=%.2f, Y=%.2f, Z=%.2f", 
                 latest_gyro.gyro_x, latest_gyro.gyro_y, latest_gyro.gyro_z);
        // ESP_LOGI(TAG, "Temperature: %.2f°C", temp.temp);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// esp_err_t imu_get_accel(float *acce_x, float *acce_y, float *acce_z) {
//     if (mpu6050 == NULL) {
//         return ESP_ERR_INVALID_STATE;
//     }
    
//     if (acce_x) *acce_x = latest_acce.acce_x;
//     if (acce_y) *acce_y = latest_acce.acce_y;
//     if (acce_z) *acce_z = latest_acce.acce_z;
    
//     return ESP_OK;
// }

// esp_err_t imu_get_gyro(float *gyro_x, float *gyro_y, float *gyro_z) {
//     if (mpu6050 == NULL) {
//         return ESP_ERR_INVALID_STATE;
//     }
    
//     if (gyro_x) *gyro_x = latest_gyro.gyro_x;
//     if (gyro_y) *gyro_y = latest_gyro.gyro_y;
//     if (gyro_z) *gyro_z = latest_gyro.gyro_z;
    
//     return ESP_OK;
// }