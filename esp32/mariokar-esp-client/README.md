# MarioKar ESP32 BLE Client with IMU Control

ESP32-S3 BLE GATT client that reads IMU orientation and transmits servo PWM values (1000-2000 µs) to STM32WB over Bluetooth.

## Features

- ✅ **IMU Integration**: ICM-20948 9-axis sensor with 6-axis fusion (accel + gyro)
- ✅ **Servo Mapping**: Converts roll angle (-90° to +90°) to PWM (1000-2000 µs)
- ✅ **BLE Transmission**: Sends 2-byte PWM values at 50ms intervals
- ✅ **Auto-Calibration**: Gyroscope bias calibration on startup
- ✅ **Connection Management**: Auto-scan, connect, and retry

## Hardware

**ESP32-S3 XIAO** connected to **ICM-20948 IMU** via SPI:
- MISO: GPIO 8
- MOSI: GPIO 9
- SCLK: GPIO 7
- CS: GPIO 44

**Target**: STM32WB "MyCST" BLE peripheral

## PWM Mapping

| IMU Roll | Servo PWM | Servo Position |
|----------|-----------|----------------|
| -90°     | 1000 µs   | Full left      |
| -45°     | 1250 µs   | Half left      |
| 0°       | 1500 µs   | Center         |
| +45°     | 1750 µs   | Half right     |
| +90°     | 2000 µs   | Full right     |

**Formula**: `PWM = 1500 + (angle × 1000 / 180)`

## BLE Protocol

### Data Format
- **Size**: 2 bytes
- **Encoding**: Little-endian (LSB first, MSB second)
- **Range**: 1000-2000 (0x03E8 - 0x07D0)

### Example Transmissions
```
Roll = 0°     → PWM = 1500 → [0xDC 0x05]
Roll = -45°   → PWM = 1250 → [0xE2 0x04]
Roll = +45°   → PWM = 1750 → [0xD6 0x06]
Roll = -90°   → PWM = 1000 → [0xE8 0x03]
Roll = +90°   → PWM = 2000 → [0xD0 0x07]
```

## Build and Flash

```bash
cd /Users/enriquegomez/CMU/Fall-2025/18-500/MarioKar/esp32/mariokar-esp-client
idf.py build flash monitor
```

## Expected Output

```
I (XXX) GATTC_CLEAN: Initializing IMU...
I (XXX) ICM20948: WHO_AM_I verified: 0xEA
I (XXX) GATTC_CLEAN: IMU initialized, calibrating gyroscope...
I (XXX) ICM20948: Gyro bias: X=-0.891 Y=0.147 Z=-0.230 deg/s
I (XXX) GATTC_CLEAN: IMU ready, starting orientation tracking
I (XXX) GATTC_CLEAN: Found target device MyCST
I (XXX) GATTC_CLEAN: P2P Service found: start=0x000C end=0x0010
I (XXX) GATTC_CLEAN: LED value handle 0x000E
I (XXX) GATTC_CLEAN: Notifications enabled.
I (XXX) GATTC_CLEAN: IMU: Roll=-1.0° -> PWM=1494 µs
I (XXX) GATTC_CLEAN: BLE TX: PWM=1494 µs [0xD6 0x05]
```

## Usage

### Tilt Control
1. **Power on** - IMU calibrates (keep stationary for 2 seconds)
2. **Center position** - Device level = 1500 µs (center)
3. **Tilt left** - Roll negative = PWM < 1500 (left turn)
4. **Tilt right** - Roll positive = PWM > 1500 (right turn)

### Changing Control Axis

Edit `main.c` line 412 to use different axis:

```c
// Use ROLL for left/right steering (default)
s_latest_pwm = angle_to_servo_pwm(orientation.roll);

// Use PITCH for forward/backward control
s_latest_pwm = angle_to_servo_pwm(orientation.pitch);

// Use YAW for rotation control
s_latest_pwm = angle_to_servo_pwm(orientation.yaw);
```

## STM32WB Receiver Code

Your STM32WB should receive and decode the PWM value:

```c
uint16_t pwm_value = (rx_buffer[1] << 8) | rx_buffer[0];  // Little-endian
// Use pwm_value (1000-2000) to control servo
```

## Troubleshooting

### IMU Not Responding
- Check SPI connections (especially CS line)
- Verify power supply (3.3V)
- Check WHO_AM_I register (should be 0xEA)

### BLE Connection Fails
- Ensure STM32WB is advertising as "MyCST"
- Check UUID matches (service: 0xFE00, char: 0xFE41)
- Try power cycling both devices

### PWM Values Stuck at 1500
- IMU might not be initialized
- Check gyro calibration completed
- Verify roll angle is changing (check logs)

### Drift Over Time
- Normal for 6-axis fusion (~3-4°/minute)
- Re-calibrate by restarting device
- Keep device level during power-on

## Technical Details

### Sensor Fusion
- **Algorithm**: Madgwick AHRS (6-axis)
- **Update Rate**: 100 Hz
- **Gyro Range**: ±500 dps
- **Accel Range**: ±4g
- **Beta**: 0.1 (filter gain)

### BLE Settings
- **TX Interval**: 50ms (20 Hz)
- **Write Type**: No response (faster)
- **MTU**: 500 bytes
- **Scan Interval**: 80ms

### Task Stack Sizes
- IMU Reader: 4096 bytes
- BLE TX: 3072 bytes

## Performance

- **CPU Usage**: ~5% @ 160MHz
- **IMU Latency**: <10ms sensor-to-orientation
- **BLE Latency**: 50ms update interval
- **Total Latency**: <60ms tilt-to-transmission

## License

Part of MarioKar project - CMU 18-500 Fall 2025

