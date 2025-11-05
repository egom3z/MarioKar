#ifndef DRV2605L_H
#define DRV2605L_H

#include <stdint.h>
#include <stdbool.h>

// DRV2605L I2C Address
#define DRV2605L_ADDR 0x5A

// DRV2605L Registers
#define DRV2605_REG_STATUS      0x00
#define DRV2605_REG_MODE        0x01
#define DRV2605_REG_RTPIN       0x02
#define DRV2605_REG_LIBRARY     0x03
#define DRV2605_REG_WAVESEQ1    0x04
#define DRV2605_REG_WAVESEQ2    0x05
#define DRV2605_REG_WAVESEQ3    0x06
#define DRV2605_REG_WAVESEQ4    0x07
#define DRV2605_REG_WAVESEQ5    0x08
#define DRV2605_REG_WAVESEQ6    0x09
#define DRV2605_REG_WAVESEQ7    0x0A
#define DRV2605_REG_WAVESEQ8    0x0B
#define DRV2605_REG_GO          0x0C
#define DRV2605_REG_OVERDRIVE   0x0D
#define DRV2605_REG_SUSTAINPOS  0x0E
#define DRV2605_REG_SUSTAINNEG  0x0F
#define DRV2605_REG_BREAK       0x10
#define DRV2605_REG_AUDIOCTRL   0x11
#define DRV2605_REG_CONTROL1    0x1B
#define DRV2605_REG_CONTROL2    0x1C
#define DRV2605_REG_CONTROL3    0x1D
#define DRV2605_REG_CONTROL4    0x1E
#define DRV2605_REG_VBAT        0x21
#define DRV2605_REG_LRARESON    0x22

// Mode values
#define DRV2605_MODE_INTTRIG     0x00
#define DRV2605_MODE_EXTTRIGEDGE 0x01
#define DRV2605_MODE_EXTTRIGLVL  0x02
#define DRV2605_MODE_PWMANALOG   0x03
#define DRV2605_MODE_AUDIOVIBE   0x04
#define DRV2605_MODE_REALTIME    0x05
#define DRV2605_MODE_DIAGNOS     0x06
#define DRV2605_MODE_AUTOCAL     0x07

// Library values
#define DRV2605_LIBRARY_EMPTY    0x00
#define DRV2605_LIBRARY_TS2200A  0x01
#define DRV2605_LIBRARY_TS2200B  0x02
#define DRV2605_LIBRARY_TS2200C  0x03
#define DRV2605_LIBRARY_TS2200D  0x04
#define DRV2605_LIBRARY_TS2200E  0x05
#define DRV2605_LIBRARY_LRA      0x06

// Function prototypes
bool drv2605l_init(uint8_t sda_pin, uint8_t scl_pin);
void drv2605l_set_mode(uint8_t mode);
void drv2605l_select_library(uint8_t library);
void drv2605l_set_waveform(uint8_t slot, uint8_t wave);
void drv2605l_go(void);
void drv2605l_stop(void);
void drv2605l_play_effect(uint8_t effect);
void drv2605l_play_sequence(uint8_t *effects, uint8_t count);
void drv2605l_set_realtime_value(uint8_t value);
void drv2605l_use_library(uint8_t library);
void drv2605l_use_rtp(void);
bool drv2605l_is_playing(void);
void drv2605l_standby(void);

#endif // DRV2605L_H