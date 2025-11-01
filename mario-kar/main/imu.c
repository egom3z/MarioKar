#include <esp_timer.h>
#include "imu.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "IMU";

// Pin definitions for XIAO ESP32S3
#define PIN_MISO    GPIO_NUM_8
#define PIN_MOSI    GPIO_NUM_9
#define PIN_SCLK    GPIO_NUM_7
#define PIN_CS      GPIO_NUM_44

// ICM-20948 Register addresses (Bank 0)
#define ICM20948_WHO_AM_I       0x00
#define ICM20948_USER_CTRL      0x03
#define ICM20948_PWR_MGMT_1     0x06
#define ICM20948_PWR_MGMT_2     0x07
#define ICM20948_GYRO_XOUT_H    0x33
#define ICM20948_REG_BANK_SEL   0x7F

// ICM-20948 Register addresses (Bank 2)
#define ICM20948_GYRO_SMPLRT_DIV   0x00
#define ICM20948_GYRO_CONFIG_1     0x01

// Register banks
#define ICM20948_BANK_0         0x00
#define ICM20948_BANK_2         0x20

// USER_CTRL bits
#define ICM20948_I2C_IF_DIS     0x10  // Disable I2C interface

// Expected WHO_AM_I value
#define ICM20948_WHO_AM_I_VAL   0xEA

static spi_device_handle_t spi_handle;

// Select register bank
static esp_err_t select_bank(uint8_t bank) {
    uint8_t tx_data[2] = {ICM20948_REG_BANK_SEL & 0x7F, bank};
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx_data,
    };
    return spi_device_polling_transmit(spi_handle, &t);
}

// Read register
static esp_err_t read_register(uint8_t reg, uint8_t *data) {
    uint8_t tx_data[2] = {reg | 0x80, 0x00};  // Set read bit
    uint8_t rx_data[2] = {0};
    
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };
    
    esp_err_t ret = spi_device_polling_transmit(spi_handle, &t);
    *data = rx_data[1];
    return ret;
}

// Write register
static esp_err_t write_register(uint8_t reg, uint8_t data) {
    uint8_t tx_data[2] = {reg & 0x7F, data};  // Clear read bit
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx_data,
    };
    return spi_device_polling_transmit(spi_handle, &t);
}

// Read multiple bytes
static esp_err_t read_bytes(uint8_t reg, uint8_t *data, size_t len) {
    uint8_t *tx_data = malloc(len + 1);
    uint8_t *rx_data = malloc(len + 1);
    
    if (!tx_data || !rx_data) {
        free(tx_data);
        free(rx_data);
        return ESP_ERR_NO_MEM;
    }
    
    tx_data[0] = reg | 0x80;  // Set read bit
    memset(&tx_data[1], 0x00, len);
    
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = (len + 1) * 8;
    t.tx_buffer = tx_data;
    t.rx_buffer = rx_data;
    
    esp_err_t ret = spi_device_polling_transmit(spi_handle, &t);
    
    if (ret == ESP_OK) {
        memcpy(data, &rx_data[1], len);
    }
    
    free(tx_data);
    free(rx_data);
    return ret;
}

float gyro_z_offset; 


static float gyro_z_angle = 0.0f;
static uint64_t last_time_us = 0;

void imu_update_heading(float gyro_z_dps) {
    uint64_t now = esp_timer_get_time(); // microseconds
    if (last_time_us == 0) {
        last_time_us = now;
        return;
    }

    float dt = (now - last_time_us) / 1e6f;  // convert to seconds
    last_time_us = now;

    gyro_z_angle += gyro_z_dps * dt;  // integrate ωz * Δt
}

float imu_calibrate_gyro_z_offset(void) {
    const int samples = 500;
    float sum = 0.0f;
    uint8_t data[6];

    for (int i = 0; i < samples; i++) {
        read_bytes(ICM20948_GYRO_XOUT_H, data, 6);
        int16_t raw_z = (int16_t)((data[4] << 8) | data[5]);
        sum += raw_z;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    float avg = sum / samples;
    return avg * (500.0f / 32768.0f);  // same scale as gyro_data.gyro_z
}

esp_err_t imu_init(void) {
    esp_err_t ret;
    
    ESP_LOGI(TAG, "Starting IMU initialization...");
    
    // Configure SPI bus
    spi_bus_config_t bus_cfg = {
        .miso_io_num = PIN_MISO,
        .mosi_io_num = PIN_MOSI,
        .sclk_io_num = PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    
    ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed");
        return ret;
    }
    
    // Configure SPI device
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 4000000,  // 1MHz
        .mode = 0,                   // SPI mode 0
        .spics_io_num = PIN_CS,
        .queue_size = 1,
        .cs_ena_pretrans = 2,
        .cs_ena_posttrans = 2,
    };
    
    ret = spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed");
        return ret;
    }
    
    ESP_LOGI(TAG, "SPI initialized, waiting for IMU power-up...");
    vTaskDelay(pdMS_TO_TICKS(100));  // Wait for IMU to power up
    
    // CRITICAL: Disable I2C interface to enable SPI mode
    ESP_LOGI(TAG, "Disabling I2C interface...");
    select_bank(ICM20948_BANK_0);
    write_register(ICM20948_USER_CTRL, ICM20948_I2C_IF_DIS);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Check WHO_AM_I
    uint8_t who_am_i;
    ret = read_register(ICM20948_WHO_AM_I, &who_am_i);
    if (ret != ESP_OK || who_am_i != ICM20948_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "WHO_AM_I failed. Expected 0x%02X, got 0x%02X", 
                 ICM20948_WHO_AM_I_VAL, who_am_i);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "WHO_AM_I OK: 0x%02X", who_am_i);
    
    // Reset device
    ESP_LOGI(TAG, "Resetting device...");
    write_register(ICM20948_PWR_MGMT_1, 0x80);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // After reset, disable I2C again (reset clears this bit)
    select_bank(ICM20948_BANK_0);
    write_register(ICM20948_USER_CTRL, ICM20948_I2C_IF_DIS);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Wake up and set clock source to auto-select best available
    ESP_LOGI(TAG, "Configuring power management...");
    write_register(ICM20948_PWR_MGMT_1, 0x01);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Enable all sensors (gyro, accel, temp)
    write_register(ICM20948_PWR_MGMT_2, 0x00);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Select bank 2 for gyro config
    ESP_LOGI(TAG, "Configuring gyroscope...");
    select_bank(ICM20948_BANK_2);

    // Set gyro sample rate divider (1125Hz / (1 + divider))
    // Divider = 0 gives ~1125 Hz
    write_register(ICM20948_GYRO_SMPLRT_DIV, 0x00);
    write_register(ICM20948_GYRO_CONFIG_1, 0x03); // DLPF=0 (196.6Hz), FS=±500, FCHOICE=1
    // write_register(ICM20948_GYRO_CONFIG_2, some_avg_cfg); // if you want averaging in low-power mode
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // Configure gyroscope:
    // Bits 5:3 = 001 (±500 dps for better sensitivity)
    // Bits 2:0 = 001 (~1.1kHz bandwidth)
    write_register(ICM20948_GYRO_CONFIG_1, 0x09);  // Changed from 0x01
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Back to bank 0 for reading
    select_bank(ICM20948_BANK_0);
    
    // Give the gyro time to stabilize
    ESP_LOGI(TAG, "Waiting for gyro to stabilize...");
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ESP_LOGI(TAG, "ICM-20948 initialized successfully");

    gyro_z_offset = imu_calibrate_gyro_z_offset();
    ESP_LOGI(TAG, "Calibrated gyro Z offset = %.2f dps", gyro_z_offset);

    return ESP_OK;
}

void imu_task(void *pvParameters) {
    uint8_t data[6];
    int16_t raw_gyro_x, raw_gyro_y, raw_gyro_z;
    imu_gyro_data_t gyro_data;
    const float gyro_scale = 500.0f / 32768.0f;  // Changed to ±500 dps range
    
    ESP_LOGI(TAG, "IMU task started");
    
    // Let sensor warm up
    vTaskDelay(pdMS_TO_TICKS(500));
    
    while (1) {
        select_bank(ICM20948_BANK_0);
        if (read_bytes(ICM20948_GYRO_XOUT_H, data, 6) == ESP_OK) {
            raw_gyro_x = (int16_t)((data[0] << 8) | data[1]);
            raw_gyro_y = (int16_t)((data[2] << 8) | data[3]);
            raw_gyro_z = (int16_t)((data[4] << 8) | data[5]);

            gyro_data.gyro_x = raw_gyro_x * gyro_scale;
            gyro_data.gyro_y = raw_gyro_y * gyro_scale;
            gyro_data.gyro_z = raw_gyro_z * gyro_scale;

            if (!(gyro_data.gyro_z < 2.0 && gyro_data.gyro_z > -2.0)) {
                imu_update_heading(gyro_data.gyro_z - gyro_z_offset);
            }

            ESP_LOGI(TAG,
                "Gyro Z: %.2f dps | Heading: %.2f deg",
                gyro_data.gyro_z, gyro_z_angle);
        } else {
            ESP_LOGE(TAG, "Failed to read gyro data");
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // 100 Hz loop
    }
}
