#include "drv2605l.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DRV2605L";
static i2c_port_t i2c_port = I2C_NUM_0;

// Write to a register
static esp_err_t drv2605l_write_register(uint8_t reg, uint8_t value)
{
    uint8_t write_buf[2] = {reg, value};
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DRV2605L_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, write_buf, 2, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

// Read from a register
static esp_err_t drv2605l_read_register(uint8_t reg, uint8_t *value)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DRV2605L_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DRV2605L_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, value, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

// Initialize the DRV2605L
bool drv2605l_init(uint8_t sda_pin, uint8_t scl_pin)
{   
    // Check if device is present
    uint8_t status;
    esp_err_t ret = drv2605l_read_register(DRV2605_REG_STATUS, &status);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DRV2605L not found");
        return false;
    }
    
    ESP_LOGI(TAG, "DRV2605L found, status: 0x%02X", status);
    
    // Exit standby mode
    drv2605l_write_register(DRV2605_REG_MODE, 0x00);
    
    // Set to internal trigger mode
    drv2605l_set_mode(DRV2605_MODE_INTTRIG);
    
    // Select library
    drv2605l_select_library(DRV2605_LIBRARY_EMPTY);
    
    // Set rated voltage and overdrive (adjust for your motor)
    drv2605l_write_register(DRV2605_REG_CONTROL3, 0xA0); // ERM open loop
    
    ESP_LOGI(TAG, "DRV2605L initialized");
    return true;
}

// Set operating mode
void drv2605l_set_mode(uint8_t mode)
{
    drv2605l_write_register(DRV2605_REG_MODE, mode);
}

// Select effect library
void drv2605l_select_library(uint8_t library)
{
    drv2605l_write_register(DRV2605_REG_LIBRARY, library);
}

// Set waveform in sequence slot
void drv2605l_set_waveform(uint8_t slot, uint8_t wave)
{
    if (slot < 8) {
        drv2605l_write_register(DRV2605_REG_WAVESEQ1 + slot, wave);
    }
}

// Start playback
void drv2605l_go(void)
{
    drv2605l_write_register(DRV2605_REG_GO, 1);
}

// Stop playback
void drv2605l_stop(void)
{
    drv2605l_write_register(DRV2605_REG_GO, 0);
}

// Play a single effect
void drv2605l_play_effect(uint8_t effect)
{
    drv2605l_set_waveform(0, effect);
    drv2605l_set_waveform(1, 0); // End sequence
    drv2605l_go();
}

// Play a sequence of effects
void drv2605l_play_sequence(uint8_t *effects, uint8_t count)
{
    for (uint8_t i = 0; i < count && i < 8; i++) {
        drv2605l_set_waveform(i, effects[i]);
    }
    if (count < 8) {
        drv2605l_set_waveform(count, 0); // End sequence
    }
    drv2605l_go();
}

// Set real-time playback value (0-255)
void drv2605l_set_realtime_value(uint8_t value)
{
    drv2605l_write_register(DRV2605_REG_RTPIN, value);
}

// Switch to library mode
void drv2605l_use_library(uint8_t library)
{
    drv2605l_set_mode(DRV2605_MODE_INTTRIG);
    drv2605l_select_library(library);
}

// Switch to real-time playback (RTP) mode
void drv2605l_use_rtp(void)
{
    drv2605l_set_mode(DRV2605_MODE_REALTIME);
}

// Check if effect is still playing
bool drv2605l_is_playing(void)
{
    uint8_t go_reg;
    drv2605l_read_register(DRV2605_REG_GO, &go_reg);
    return (go_reg & 0x01) != 0;
}

// Enter standby mode (low power)
void drv2605l_standby(void)
{
    drv2605l_write_register(DRV2605_REG_MODE, 0x40);
}