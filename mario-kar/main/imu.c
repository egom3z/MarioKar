#include "imu.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "IMU";

// Current sensor ranges
static mpu6050_accel_range_t current_accel_range = ACCEL_RANGE_2G;
static mpu6050_gyro_range_t current_gyro_range = GYRO_RANGE_250DPS;

// Scale factors for converting raw data
static float accel_scale = 16384.0f;  // for ±2g
static float gyro_scale = 131.0f;     // for ±250°/s

/**
 * @brief Write a byte to MPU6050 register
 */
static esp_err_t mpu6050_write_byte(uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, 
                                      write_buf, sizeof(write_buf), 
                                      pdMS_TO_TICKS(1000));
}

/**
 * @brief Read bytes from MPU6050 register
 */
static esp_err_t mpu6050_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDR,
                                        &reg_addr, 1, data, len,
                                        pdMS_TO_TICKS(1000));
}

/**
 * @brief Initialize I2C bus
 */
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        return err;
    }
    
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

/**
 * @brief Update scale factors based on current ranges
 */
static void update_scale_factors(void) {
    switch (current_accel_range) {
        case ACCEL_RANGE_2G:  accel_scale = 16384.0f; break;
        case ACCEL_RANGE_4G:  accel_scale = 8192.0f;  break;
        case ACCEL_RANGE_8G:  accel_scale = 4096.0f;  break;
        case ACCEL_RANGE_16G: accel_scale = 2048.0f;  break;
    }
    
    switch (current_gyro_range) {
        case GYRO_RANGE_250DPS:  gyro_scale = 131.0f;   break;
        case GYRO_RANGE_500DPS:  gyro_scale = 65.5f;    break;
        case GYRO_RANGE_1000DPS: gyro_scale = 32.8f;    break;
        case GYRO_RANGE_2000DPS: gyro_scale = 16.4f;    break;
    }
}

esp_err_t imu_init(void) {
    esp_err_t ret;
    
    // Initialize I2C
    ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "I2C initialized successfully");
    
    // Small delay to let the sensor stabilize
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Check WHO_AM_I register
    uint8_t who_am_i;
    ret = mpu6050_read_bytes(MPU6050_REG_WHO_AM_I, &who_am_i, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register: %s (0x%X)", esp_err_to_name(ret), ret);
        ESP_LOGE(TAG, "Check wiring: SCL=GPIO%d, SDA=GPIO%d", I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO);
        return ret;
    }
    
    if (who_am_i != 0x68) {
        ESP_LOGE(TAG, "MPU6050 not found! WHO_AM_I: 0x%02X (expected 0x68)", who_am_i);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "MPU6050 detected successfully");
    
    // Wake up MPU6050 (clear sleep bit)
    ret = mpu6050_write_byte(MPU6050_REG_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up MPU6050");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Set accelerometer range to ±2g
    ret = imu_set_accel_range(ACCEL_RANGE_2G);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set accelerometer range");
        return ret;
    }
    
    // Set gyroscope range to ±250°/s
    ret = imu_set_gyro_range(GYRO_RANGE_250DPS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set gyroscope range");
        return ret;
    }
    
    ESP_LOGI(TAG, "MPU6050 initialized successfully");
    return ESP_OK;
}

esp_err_t imu_set_accel_range(mpu6050_accel_range_t range) {
    esp_err_t ret = mpu6050_write_byte(MPU6050_REG_ACCEL_CONFIG, range << 3);
    if (ret == ESP_OK) {
        current_accel_range = range;
        update_scale_factors();
    }
    return ret;
}

esp_err_t imu_set_gyro_range(mpu6050_gyro_range_t range) {
    esp_err_t ret = mpu6050_write_byte(MPU6050_REG_GYRO_CONFIG, range << 3);
    if (ret == ESP_OK) {
        current_gyro_range = range;
        update_scale_factors();
    }
    return ret;
}

esp_err_t imu_read_accel(int16_t *accel_x, int16_t *accel_y, int16_t *accel_z) {
    uint8_t data[6];
    esp_err_t ret = mpu6050_read_bytes(MPU6050_REG_ACCEL_XOUT_H, data, 6);
    
    if (ret == ESP_OK) {
        *accel_x = (int16_t)((data[0] << 8) | data[1]);
        *accel_y = (int16_t)((data[2] << 8) | data[3]);
        *accel_z = (int16_t)((data[4] << 8) | data[5]);
    }
    
    return ret;
}

esp_err_t imu_read_gyro(int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) {
    uint8_t data[6];
    esp_err_t ret = mpu6050_read_bytes(MPU6050_REG_GYRO_XOUT_H, data, 6);
    
    if (ret == ESP_OK) {
        *gyro_x = (int16_t)((data[0] << 8) | data[1]);
        *gyro_y = (int16_t)((data[2] << 8) | data[3]);
        *gyro_z = (int16_t)((data[4] << 8) | data[5]);
    }
    
    return ret;
}

esp_err_t imu_read_temp(float *temp) {
    uint8_t data[2];
    esp_err_t ret = mpu6050_read_bytes(MPU6050_REG_TEMP_OUT_H, data, 2);
    
    if (ret == ESP_OK) {
        int16_t raw_temp = (int16_t)((data[0] << 8) | data[1]);
        *temp = (raw_temp / 340.0f) + 36.53f;
    }
    
    return ret;
}

esp_err_t imu_read_data(imu_data_t *data) {
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    float temp;
    
    esp_err_t ret = imu_read_accel(&accel_x, &accel_y, &accel_z);
    if (ret != ESP_OK) return ret;
    
    ret = imu_read_gyro(&gyro_x, &gyro_y, &gyro_z);
    if (ret != ESP_OK) return ret;
    
    ret = imu_read_temp(&temp);
    if (ret != ESP_OK) return ret;
    
    // Convert raw values to physical units
    data->accel_x = accel_x / accel_scale;
    data->accel_y = accel_y / accel_scale;
    data->accel_z = accel_z / accel_scale;
    
    data->gyro_x = gyro_x / gyro_scale;
    data->gyro_y = gyro_y / gyro_scale;
    data->gyro_z = gyro_z / gyro_scale;
    
    data->temp = temp;
    
    return ESP_OK;
}

void imu_task(void *pvParameters) {
    imu_data_t imu_data;
    
    ESP_LOGI(TAG, "IMU task started");
    
    while (1) {
        if (imu_read_data(&imu_data) == ESP_OK) {
            ESP_LOGI(TAG, "Accel: X=%.2f Y=%.2f Z=%.2f g", 
                     imu_data.accel_x, imu_data.accel_y, imu_data.accel_z);
            ESP_LOGI(TAG, "Gyro: X=%.2f Y=%.2f Z=%.2f °/s", 
                     imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z);
            ESP_LOGI(TAG, "Temp: %.2f °C", imu_data.temp);
        } else {
            ESP_LOGE(TAG, "Failed to read IMU data");
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));  // Read at 10Hz
    }
}