/**
 * @file sensor.c
 * @brief ICM-20948 9-axis IMU driver with SPI interface
 * 
 * Features:
 * - SPI communication with ICM-20948
 * - I2C master mode for AK09916 magnetometer
 * - Madgwick AHRS sensor fusion
 * - Gyroscope calibration
 */

#include "sensor.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>

static const char *TAG = "ICM20948";

// =============================================================================
// Hardware Configuration
// =============================================================================

// GPIO pin assignments for XIAO ESP32S3
#define PIN_MISO        GPIO_NUM_8
#define PIN_MOSI        GPIO_NUM_9
#define PIN_SCLK        GPIO_NUM_7
#define PIN_CS          GPIO_NUM_44

#define SPI_CLOCK_HZ    4000000  // 4 MHz SPI clock

// =============================================================================
// ICM-20948 Register Definitions
// =============================================================================

// Bank 0 Registers
#define REG_WHO_AM_I            0x00
#define REG_USER_CTRL           0x03
#define REG_PWR_MGMT_1          0x06
#define REG_PWR_MGMT_2          0x07
#define REG_INT_PIN_CFG         0x0F
#define REG_I2C_MST_STATUS      0x17
#define REG_ACCEL_XOUT_H        0x2D
#define REG_GYRO_XOUT_H         0x33
#define REG_EXT_SLV_SENS_DATA_00 0x3B
#define REG_BANK_SEL            0x7F

// Bank 2 Registers
#define REG_GYRO_SMPLRT_DIV     0x00
#define REG_GYRO_CONFIG_1       0x01
#define REG_GYRO_CONFIG_2       0x02
#define REG_ACCEL_SMPLRT_DIV_1  0x10
#define REG_ACCEL_SMPLRT_DIV_2  0x11
#define REG_ACCEL_CONFIG        0x14
#define REG_ACCEL_CONFIG_2      0x15

// Bank 3 Registers
#define REG_I2C_MST_CTRL        0x01
#define REG_I2C_SLV0_ADDR       0x03
#define REG_I2C_SLV0_REG        0x04
#define REG_I2C_SLV0_CTRL       0x05
#define REG_I2C_SLV0_DO         0x06

// Register Banks
#define BANK_0                  0x00
#define BANK_1                  0x10
#define BANK_2                  0x20
#define BANK_3                  0x30

// USER_CTRL bits
#define BIT_I2C_MST_EN          0x20
#define BIT_I2C_IF_DIS          0x10

// WHO_AM_I expected value
#define ICM20948_WHO_AM_I_VAL   0xEA

// =============================================================================
// AK09916 Magnetometer Definitions
// =============================================================================

#define AK09916_I2C_ADDR        0x0C
#define AK09916_WIA2            0x01
#define AK09916_ST1             0x10
#define AK09916_HXL             0x11
#define AK09916_ST2             0x18
#define AK09916_CNTL2           0x31
#define AK09916_CNTL3           0x32

#define AK09916_MODE_CONT_100HZ 0x08
#define AK09916_WIA2_VAL        0x09

// =============================================================================
// Sensor Configuration
// =============================================================================

#define GYRO_RANGE_DPS          500.0f
#define ACCEL_RANGE_G           4.0f
#define MAG_RANGE_UT            4912.0f

#define GYRO_SCALE              (GYRO_RANGE_DPS / 32768.0f)
#define ACCEL_SCALE             (ACCEL_RANGE_G / 32768.0f)
#define MAG_SCALE               (MAG_RANGE_UT / 32768.0f)

// =============================================================================
// Madgwick Filter Configuration
// =============================================================================

#define MADGWICK_BETA           0.1f
#define DEG_TO_RAD              (M_PI / 180.0f)
#define RAD_TO_DEG              (180.0f / M_PI)

// =============================================================================
// Static Variables
// =============================================================================

static spi_device_handle_t spi_handle = NULL;
static uint8_t current_bank = 0xFF;  // Track current register bank

// Gyroscope bias from calibration
static float gyro_bias[3] = {0.0f, 0.0f, 0.0f};

// Madgwick filter state (quaternion)
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// =============================================================================
// SPI Communication Functions
// =============================================================================

/**
 * @brief Select ICM-20948 register bank
 */
static esp_err_t select_bank(uint8_t bank)
{
    if (bank == current_bank) {
        return ESP_OK;  // Already in correct bank
    }

    uint8_t tx_data[2] = {REG_BANK_SEL & 0x7F, bank};
    spi_transaction_t trans = {
        .length = 16,
        .tx_buffer = tx_data,
    };

    esp_err_t ret = spi_device_polling_transmit(spi_handle, &trans);
    if (ret == ESP_OK) {
        current_bank = bank;
    }
    return ret;
}

/**
 * @brief Read single register from ICM-20948
 */
static esp_err_t read_register(uint8_t reg, uint8_t *data)
{
    uint8_t tx_data[2] = {reg | 0x80, 0x00};  // MSB=1 for read
    uint8_t rx_data[2] = {0};

    spi_transaction_t trans = {
        .length = 16,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };

    esp_err_t ret = spi_device_polling_transmit(spi_handle, &trans);
    if (ret == ESP_OK) {
        *data = rx_data[1];
    }
    return ret;
}

/**
 * @brief Write single register to ICM-20948
 */
static esp_err_t write_register(uint8_t reg, uint8_t data)
{
    uint8_t tx_data[2] = {reg & 0x7F, data};  // MSB=0 for write

    spi_transaction_t trans = {
        .length = 16,
        .tx_buffer = tx_data,
    };

    return spi_device_polling_transmit(spi_handle, &trans);
}

/**
 * @brief Read multiple bytes from ICM-20948
 */
static esp_err_t read_bytes(uint8_t reg, uint8_t *data, size_t len)
{
    uint8_t *tx_buf = calloc(len + 1, 1);
    uint8_t *rx_buf = calloc(len + 1, 1);

    if (!tx_buf || !rx_buf) {
        free(tx_buf);
        free(rx_buf);
        return ESP_ERR_NO_MEM;
    }

    tx_buf[0] = reg | 0x80;  // MSB=1 for read

    spi_transaction_t trans = {
        .length = (len + 1) * 8,
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
    };

    esp_err_t ret = spi_device_polling_transmit(spi_handle, &trans);
    if (ret == ESP_OK) {
        memcpy(data, rx_buf + 1, len);
    }

    free(tx_buf);
    free(rx_buf);
    return ret;
}

// =============================================================================
// Magnetometer I2C Master Functions
// =============================================================================

/**
 * @brief Write to magnetometer via I2C master
 */
static esp_err_t mag_write(uint8_t reg, uint8_t data)
{
    select_bank(BANK_3);

    write_register(REG_I2C_SLV0_ADDR, AK09916_I2C_ADDR);
    write_register(REG_I2C_SLV0_REG, reg);
    write_register(REG_I2C_SLV0_DO, data);
    write_register(REG_I2C_SLV0_CTRL, 0x81);  // Enable, 1 byte

    vTaskDelay(pdMS_TO_TICKS(10));

    select_bank(BANK_0);
    return ESP_OK;
}

/**
 * @brief Read from magnetometer via I2C master
 */
static esp_err_t mag_read(uint8_t reg, uint8_t *data)
{
    select_bank(BANK_3);

    write_register(REG_I2C_SLV0_ADDR, AK09916_I2C_ADDR | 0x80);  // Read bit
    write_register(REG_I2C_SLV0_REG, reg);
    write_register(REG_I2C_SLV0_CTRL, 0x81);  // Enable, 1 byte

    vTaskDelay(pdMS_TO_TICKS(10));

    select_bank(BANK_0);
    return read_register(REG_EXT_SLV_SENS_DATA_00, data);
}

/**
 * @brief Setup continuous magnetometer reading
 */
static esp_err_t mag_setup_continuous_read(void)
{
    select_bank(BANK_3);

    // Configure I2C Slave 0 to automatically read magnetometer
    // Read 9 bytes starting from ST1 (ST1 + 6 mag bytes + ST2 + 1 dummy)
    write_register(REG_I2C_SLV0_ADDR, AK09916_I2C_ADDR | 0x80);  // Read mode
    write_register(REG_I2C_SLV0_REG, AK09916_ST1);
    write_register(REG_I2C_SLV0_CTRL, 0x89);  // Enable + 9 bytes
    
    // Also set the data output rate divider to read every sample
    write_register(REG_I2C_SLV0_DO, 0x00);

    select_bank(BANK_0);
    return ESP_OK;
}

// =============================================================================
// Madgwick Filter Implementation
// =============================================================================

/**
 * @brief Fast inverse square root (Quake III algorithm)
 */
static inline float inv_sqrt(float x)
{
    if (x <= 0.0f) return 0.0f;

    union {
        float f;
        uint32_t i;
    } conv = {.f = x};

    conv.i = 0x5f3759df - (conv.i >> 1);
    float y = conv.f;

    // Newton-Raphson iterations
    y = y * (1.5f - 0.5f * x * y * y);
    y = y * (1.5f - 0.5f * x * y * y);

    return y;
}

/**
 * @brief Madgwick IMU update (6-axis: accel + gyro only, no magnetometer)
 */
static void madgwick_update_imu(float gx, float gy, float gz,
                                float ax, float ay, float az,
                                float dt)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Convert gyroscope to rad/s
    gx *= DEG_TO_RAD;
    gy *= DEG_TO_RAD;
    gz *= DEG_TO_RAD;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalize accelerometer
        recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // Gradient descent algorithm corrective step (6-axis)
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

        recipNorm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= MADGWICK_BETA * s0;
        qDot2 -= MADGWICK_BETA * s1;
        qDot3 -= MADGWICK_BETA * s2;
        qDot4 -= MADGWICK_BETA * s3;
    }

    // Integrate rate of change of quaternion
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Normalize quaternion
    recipNorm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

// =============================================================================
// Public API Implementation
// =============================================================================

esp_err_t imu_init(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "Initializing ICM-20948 IMU...");

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
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure SPI device
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = SPI_CLOCK_HZ,
        .mode = 0,  // SPI mode 0
        .spics_io_num = PIN_CS,
        .queue_size = 1,
        .cs_ena_pretrans = 2,
        .cs_ena_posttrans = 2,
    };

    ret = spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(100));  // Wait for power-up

    // Disable I2C slave interface to enable SPI
    select_bank(BANK_0);
    write_register(REG_USER_CTRL, BIT_I2C_IF_DIS);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Check WHO_AM_I
    uint8_t who_am_i;
    ret = read_register(REG_WHO_AM_I, &who_am_i);
    if (ret != ESP_OK || who_am_i != ICM20948_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "WHO_AM_I check failed: expected 0x%02X, got 0x%02X",
                 ICM20948_WHO_AM_I_VAL, who_am_i);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "WHO_AM_I verified: 0x%02X", who_am_i);

    // Reset device
    write_register(REG_PWR_MGMT_1, 0x80);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Disable I2C slave again after reset
    write_register(REG_USER_CTRL, BIT_I2C_IF_DIS);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Wake up and select auto clock source
    write_register(REG_PWR_MGMT_1, 0x01);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Enable accelerometer and gyroscope
    write_register(REG_PWR_MGMT_2, 0x00);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Configure gyroscope (Bank 2)
    select_bank(BANK_2);
    write_register(REG_GYRO_SMPLRT_DIV, 0x00);
    write_register(REG_GYRO_CONFIG_1, 0x0B);  // ±500dps, DLPF enabled
    write_register(REG_GYRO_CONFIG_2, 0x00);

    // Configure accelerometer (Bank 2)
    write_register(REG_ACCEL_SMPLRT_DIV_1, 0x00);
    write_register(REG_ACCEL_SMPLRT_DIV_2, 0x00);
    write_register(REG_ACCEL_CONFIG, 0x13);    // ±4g, DLPF enabled
    write_register(REG_ACCEL_CONFIG_2, 0x00);

    // Enable I2C master mode (Bank 0)
    select_bank(BANK_0);
    write_register(REG_USER_CTRL, BIT_I2C_MST_EN | BIT_I2C_IF_DIS);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Configure I2C master clock (Bank 3)
    // Bits 3:0 = I2C master clock divider
    // 0x07 = 345.6 kHz (recommended for AK09916)
    select_bank(BANK_3);
    write_register(REG_I2C_MST_CTRL, 0x07);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Check magnetometer WHO_AM_I
    uint8_t mag_id;
    mag_read(AK09916_WIA2, &mag_id);
    if (mag_id == AK09916_WIA2_VAL) {
        ESP_LOGI(TAG, "Magnetometer detected: 0x%02X", mag_id);
    } else {
        ESP_LOGW(TAG, "Magnetometer not detected (got 0x%02X)", mag_id);
    }

    // Reset magnetometer
    mag_write(AK09916_CNTL3, 0x01);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Set continuous measurement mode
    mag_write(AK09916_CNTL2, AK09916_MODE_CONT_100HZ);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Setup continuous magnetometer reading
    mag_setup_continuous_read();

    select_bank(BANK_0);
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "ICM-20948 initialization complete");
    return ESP_OK;
}

esp_err_t imu_read_sensors(imu_data_t *data)
{
    if (!data) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buffer[6];
    int16_t raw_data[3];

    select_bank(BANK_0);

    // Read accelerometer
    if (read_bytes(REG_ACCEL_XOUT_H, buffer, 6) == ESP_OK) {
        raw_data[0] = (int16_t)((buffer[0] << 8) | buffer[1]);
        raw_data[1] = (int16_t)((buffer[2] << 8) | buffer[3]);
        raw_data[2] = (int16_t)((buffer[4] << 8) | buffer[5]);

        data->accel_x = raw_data[0] * ACCEL_SCALE;
        data->accel_y = raw_data[1] * ACCEL_SCALE;
        data->accel_z = raw_data[2] * ACCEL_SCALE;
    }

    // Read gyroscope
    if (read_bytes(REG_GYRO_XOUT_H, buffer, 6) == ESP_OK) {
        raw_data[0] = (int16_t)((buffer[0] << 8) | buffer[1]);
        raw_data[1] = (int16_t)((buffer[2] << 8) | buffer[3]);
        raw_data[2] = (int16_t)((buffer[4] << 8) | buffer[5]);

        data->gyro_x = raw_data[0] * GYRO_SCALE - gyro_bias[0];
        data->gyro_y = raw_data[1] * GYRO_SCALE - gyro_bias[1];
        data->gyro_z = raw_data[2] * GYRO_SCALE - gyro_bias[2];
    }

    // Read magnetometer from I2C slave buffer
    uint8_t mag_buffer[9];
    data->mag_valid = false;

    if (read_bytes(REG_EXT_SLV_SENS_DATA_00, mag_buffer, 9) == ESP_OK) {
        uint8_t st1 = mag_buffer[0];
        uint8_t st2 = mag_buffer[8];
        
        // Check ST1 DRDY bit (bit 0) and ST2 HOFL bit (bit 3)
        if ((st1 & 0x01) && !(st2 & 0x08)) {  // Data ready and no overflow
            // AK09916 outputs data in little-endian format
            raw_data[0] = (int16_t)(mag_buffer[1] | (mag_buffer[2] << 8));
            raw_data[1] = (int16_t)(mag_buffer[3] | (mag_buffer[4] << 8));
            raw_data[2] = (int16_t)(mag_buffer[5] | (mag_buffer[6] << 8));

            data->mag_x = raw_data[0] * MAG_SCALE;
            data->mag_y = raw_data[1] * MAG_SCALE;
            data->mag_z = raw_data[2] * MAG_SCALE;
            data->mag_valid = true;
        }
    }

    return ESP_OK;
}

void imu_update_fusion(const imu_data_t *data, float dt)
{
    if (!data) return;

    // Use 6-axis IMU fusion (no magnetometer)
    madgwick_update_imu(
        data->gyro_x, data->gyro_y, data->gyro_z,
        data->accel_x, data->accel_y, data->accel_z,
        dt
    );
}

void imu_get_orientation(imu_orientation_t *orientation)
{
    if (!orientation) return;

    // Convert quaternion to Euler angles
    float q0_sq = q0 * q0;
    float q1_sq = q1 * q1;
    float q2_sq = q2 * q2;
    float q3_sq = q3 * q3;

    // Roll (x-axis rotation)
    float sinr = 2.0f * (q0 * q1 + q2 * q3);
    float cosr = q0_sq - q1_sq - q2_sq + q3_sq;
    orientation->roll = atan2f(sinr, cosr) * RAD_TO_DEG;

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1.0f) {
        orientation->pitch = copysignf(90.0f, sinp);  // Gimbal lock
    } else {
        orientation->pitch = asinf(sinp) * RAD_TO_DEG;
    }

    // Yaw (z-axis rotation)
    float siny = 2.0f * (q0 * q3 + q1 * q2);
    float cosy = q0_sq + q1_sq - q2_sq - q3_sq;
    orientation->yaw = atan2f(siny, cosy) * RAD_TO_DEG;
}

/**
 * @brief Map angle (-90 to +90 degrees) to servo PWM (1000 to 2000 microseconds)
 * 
 * @param angle Input angle in degrees (-90 to +90)
 * @return Servo PWM value in microseconds (1000 to 2000)
 */
static inline int angle_to_servo_pwm(float angle)
{
    // Clamp angle to -90 to +90 range
    if (angle < -90.0f) angle = -90.0f;
    if (angle > 90.0f) angle = 90.0f;
    
    // Map: -90° -> 1000µs, 0° -> 1500µs, +90° -> 2000µs
    // Formula: pwm = 1500 + (angle * 1000 / 180)
    //        = 1500 + (angle * 5.556)
    int pwm = (int)(1500.0f + (angle * 1000.0f / 180.0f));
    
    return pwm;
}

esp_err_t imu_calibrate_gyro(int samples)
{
    if (samples < 100) {
        ESP_LOGW(TAG, "Calibration samples too low, using 100");
        samples = 100;
    }

    ESP_LOGI(TAG, "Calibrating gyroscope with %d samples...", samples);
    ESP_LOGI(TAG, "Keep device stationary!");

    int32_t sum[3] = {0, 0, 0};
    uint8_t buffer[6];
    int16_t raw_data[3];

    select_bank(BANK_0);

    for (int i = 0; i < samples; i++) {
        if (read_bytes(REG_GYRO_XOUT_H, buffer, 6) == ESP_OK) {
            raw_data[0] = (int16_t)((buffer[0] << 8) | buffer[1]);
            raw_data[1] = (int16_t)((buffer[2] << 8) | buffer[3]);
            raw_data[2] = (int16_t)((buffer[4] << 8) | buffer[5]);

            sum[0] += raw_data[0];
            sum[1] += raw_data[1];
            sum[2] += raw_data[2];
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }

    gyro_bias[0] = (sum[0] / (float)samples) * GYRO_SCALE;
    gyro_bias[1] = (sum[1] / (float)samples) * GYRO_SCALE;
    gyro_bias[2] = (sum[2] / (float)samples) * GYRO_SCALE;

    ESP_LOGI(TAG, "Gyro bias: X=%.3f Y=%.3f Z=%.3f deg/s",
             gyro_bias[0], gyro_bias[1], gyro_bias[2]);

    return ESP_OK;
}

void imu_task(void *pvParameters)
{
    imu_data_t sensor_data;
    imu_orientation_t orientation;
    uint64_t last_time = esp_timer_get_time();
    int loop_count = 0;
    int mag_valid_count = 0;
    int mag_invalid_count = 0;

    ESP_LOGI(TAG, "IMU task started");

    // Calibrate gyroscope
    imu_calibrate_gyro(1000);

    ESP_LOGI(TAG, "Starting sensor fusion loop at 100 Hz");

    while (1) {
        // Calculate time delta
        uint64_t current_time = esp_timer_get_time();
        float dt = (current_time - last_time) / 1000000.0f;
        last_time = current_time;

        // Clamp dt to reasonable range
        if (dt <= 0.0f || dt > 0.1f) {
            dt = 0.01f;  // Default 100 Hz
        }

        // Read sensors
        if (imu_read_sensors(&sensor_data) == ESP_OK) {
            // Update sensor fusion (6-axis: accel + gyro only)
            imu_update_fusion(&sensor_data, dt);

            // Get orientation
            imu_get_orientation(&orientation);

            // Log periodically (every 0.5 seconds)
            if (++loop_count % 50 == 0) {
                float acc_mag = sqrtf(sensor_data.accel_x * sensor_data.accel_x +
                                     sensor_data.accel_y * sensor_data.accel_y +
                                     sensor_data.accel_z * sensor_data.accel_z);

                ESP_LOGI(TAG, "Accel[%.2f, %.2f, %.2f]g |g|=%.2f",
                         sensor_data.accel_x, sensor_data.accel_y, 
                         sensor_data.accel_z, acc_mag);
                ESP_LOGI(TAG, "Gyro[%.1f, %.1f, %.1f]dps",
                         sensor_data.gyro_x, sensor_data.gyro_y, sensor_data.gyro_z);

                ESP_LOGI(TAG, "Orientation: Roll=%.1f Pitch=%.1f Yaw=%.1f (6-axis)",
                         orientation.roll, orientation.pitch, orientation.yaw);

                // Map angles to servo PWM values (1000-2000 microseconds)
                int roll_pwm = angle_to_servo_pwm(orientation.roll);
                int pitch_pwm = angle_to_servo_pwm(orientation.pitch);
                int yaw_pwm = angle_to_servo_pwm(orientation.yaw);
                
                ESP_LOGI(TAG, "Servo PWM: Roll=%d Pitch=%d Yaw=%d µs",
                         roll_pwm, pitch_pwm, yaw_pwm);

                // Sanity check
                if (acc_mag < 0.8f || acc_mag > 1.2f) {
                    ESP_LOGW(TAG, "Accel magnitude %.2fg outside expected range!", acc_mag);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // 100 Hz
    }
}
