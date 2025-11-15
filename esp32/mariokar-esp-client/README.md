# MarioKar ESP32 BLE Client with IMU Control

ESP32-S3 BLE GATT client that reads IMU orientation and transmits servo PWM values (1000-2000 µs) to STM32WB over Bluetooth.

## Features

- ✅ **Multi-Sensor Support**: Choose between MPU6050 (I2C) or ICM-20948 (SPI)
- ✅ **Servo Mapping**: Converts roll angle (-90° to +90°) to PWM (1000-2000 µs)
- ✅ **BLE Transmission**: Sends 2-byte PWM values at 50ms intervals
- ✅ **Auto-Calibration**: Gyroscope bias calibration on startup
- ✅ **Connection Management**: Auto-scan, connect, and retry
- ✅ **Madgwick AHRS**: Sensor fusion for accurate orientation

## Supported IMU Sensors

### MPU6050 (Default - I2C)
- **Axes**: 6-axis (accelerometer + gyroscope)
- **Interface**: I2C
- **Pins (XIAO ESP32-S3)**:
  - SCL: GPIO 6
  - SDA: GPIO 5
- **Features**: 
  - ±4g accelerometer range
  - ±500°/s gyroscope range
  - No magnetometer (yaw will drift without external reference)
- **When to use**: Budget-friendly, easier wiring, sufficient for roll/pitch control

### ICM-20948 (SPI)
- **Axes**: 9-axis (accelerometer + gyroscope + magnetometer)
- **Interface**: SPI
- **Pins (XIAO ESP32-S3)**:
  - MISO: GPIO 8
  - MOSI: GPIO 9
  - SCLK: GPIO 7
  - CS: GPIO 44
- **Features**:
  - ±4g accelerometer range
  - ±500°/s gyroscope range
  - AK09916 magnetometer for absolute heading
- **When to use**: Need accurate yaw/heading, willing to pay premium

## Selecting IMU Sensor

Edit `main/sensor.h` and change the sensor type:

```c
// For MPU6050 (I2C)
#define IMU_SENSOR_TYPE     IMU_TYPE_MPU6050

// OR for ICM-20948 (SPI)
#define IMU_SENSOR_TYPE     IMU_TYPE_ICM20948
```

The build system will automatically compile only the relevant sensor driver.

## Hardware Connections

### Option 1: MPU6050 (I2C) - Default
```
MPU6050          XIAO ESP32-S3
  VCC    <-->    3.3V
  GND    <-->    GND
  SCL    <-->    GPIO 6
  SDA    <-->    GPIO 5
```

### Option 2: ICM-20948 (SPI)
```
ICM-20948        XIAO ESP32-S3
  VCC    <-->    3.3V
  GND    <-->    GND
  MISO   <-->    GPIO 8
  MOSI   <-->    GPIO 9
  SCLK   <-->    GPIO 7
  CS     <-->    GPIO 44
```

## Software Architecture

```
┌─────────────────────────────────────────────────────┐
│              app_main()                             │
│  ┌───────────────────┐  ┌──────────────────────┐  │
│  │  imu_reader_task  │  │    ble_tx_task       │  │
│  │  (100 Hz)         │  │    (20 Hz)           │  │
│  └─────────┬─────────┘  └──────────┬───────────┘  │
│            │                        │              │
│            ▼                        ▼              │
│  ┌─────────────────┐      ┌────────────────┐     │
│  │  Sensor Driver  │      │  BLE TX Buffer  │     │
│  │  (MPU/ICM)      │─────▶│  (s_latest_pwm) │     │
│  └─────────────────┘      └────────────────┘     │
│            │                        │              │
│            ▼                        ▼              │
│  ┌─────────────────┐      ┌────────────────┐     │
│  │ Madgwick Filter │      │  GATT Client   │     │
│  │ (6/9-DOF AHRS)  │      │                │     │
│  └─────────────────┘      └────────┬───────┘     │
│            │                        │              │
│            ▼                        ▼              │
│  ┌─────────────────┐      ┌────────────────┐     │
│  │ Euler Angles    │      │   STM32WB      │     │
│  │ (Roll/Pitch/Yaw)│      │   (BLE Server) │     │
│  └─────────────────┘      └────────────────┘     │
└─────────────────────────────────────────────────────┘
```

## Building and Flashing

### Prerequisites
```bash
# Install ESP-IDF v5.x
cd /opt/esp/idf
./install.sh
source export.sh
```

### Build
```bash
cd esp32/mariokar-esp-client
idf.py set-target esp32s3
idf.py build
```

### Flash
```bash
idf.py flash monitor
```

### Expected Output
```
I (1234) MPU6050: Initializing MPU6050 IMU
I (1245) MPU6050: MPU6050 initialized successfully (I2C mode)
I (1250) MPU6050: Calibrating gyroscope... keep device stationary!
I (1255) MPU6050: Progress: 0%
...
I (3456) MPU6050: Calibration complete (1000 samples)
I (3460) MPU6050: Gyro offsets: X=-0.234 Y=0.123 Z=-0.056 deg/s
I (3465) GATTC_CLEAN: IMU ready, starting orientation tracking
I (3470) GATTC_CLEAN: BLE TX task started (interval 50 ms)
I (4567) GATTC_CLEAN: Found target device MyCST
I (5678) GATTC_CLEAN: P2P Service found: start=0x000c end=0x0010
I (5890) GATTC_CLEAN: LED value handle 0x000d
I (6123) GATTC_CLEAN: SWITCH decl handle 0x000f
I (6234) GATTC_CLEAN: Notifications enabled.
I (6345) GATTC_CLEAN: BLE TX: PWM=1500 µs [0xDC 0x05]
I (7456) GATTC_CLEAN: IMU: Roll=-15.3° -> PWM=1415 µs
```

## Calibration

**Important**: Keep the device **completely stationary** during the ~2 second gyroscope calibration on startup. This measures and removes gyroscope bias for accurate orientation tracking.

## Steering Control Mapping

The system uses **ROLL** angle for steering:

| Roll Angle | Servo PWM | Steering Direction |
|------------|-----------|-------------------|
| -90°       | 1000 µs   | Full Left         |
| -45°       | 1250 µs   | Half Left         |
| 0°         | 1500 µs   | Center/Straight   |
| +45°       | 1750 µs   | Half Right        |
| +90°       | 2000 µs   | Full Right        |

## Code Structure

```
main/
├── main.c              # BLE client, IMU reader, TX tasks
├── sensor.h            # Common IMU API & sensor selection
├── sensor.c            # ICM-20948 driver (SPI, 9-axis)
├── sensor_mpu6050.c    # MPU6050 driver (I2C, 6-axis)
├── idf_component.yml   # MPU6050 component dependency
└── CMakeLists.txt      # Build configuration
```

## Sensor Fusion Details

Both sensors use the **Madgwick AHRS algorithm** for sensor fusion:

### MPU6050 (6-DOF Mode)
- Fuses accelerometer + gyroscope data
- No magnetometer → **yaw will drift** over time
- Excellent for roll and pitch (used for steering)
- Sample rate: 100 Hz
- Filter beta: 0.1 (tunable for response vs smoothness)

### ICM-20948 (9-DOF Mode)
- Fuses accelerometer + gyroscope + magnetometer
- Magnetometer provides absolute heading reference
- Yaw is stable and doesn't drift
- Sample rate: 100 Hz
- Filter beta: 0.1

## Troubleshooting

### IMU not detected
**MPU6050:**
- Check I2C connections (SCL on GPIO 6, SDA on GPIO 5)
- Verify 3.3V power supply
- Try adding external 4.7kΩ pull-up resistors on SCL/SDA

**ICM-20948:**
- Check SPI connections (see pinout above)
- Verify CS line is connected to GPIO 44
- Ensure 3.3V power (not 5V)

### Orientation drifts
- Ensure device was **stationary** during calibration
- For MPU6050: Yaw drift is normal (no magnetometer)
- Recalibrate by restarting the device
- Check for magnetic interference if using ICM-20948

### BLE connection fails
- Verify STM32WB is advertising as "MyCST"
- Check that P2P service UUID matches
- Monitor with `idf.py monitor` for connection logs
- Try power cycling both devices

### Roll angle reversed
- Swap sign in `angle_to_servo_pwm()` function in `main.c`
- Or physically mount IMU rotated 180°

## Customization

### Change steering axis
In `main.c`, line ~412:
```c
// Use PITCH instead of ROLL
s_latest_pwm = angle_to_servo_pwm(orientation.pitch);

// Or use YAW (only stable with ICM-20948)
s_latest_pwm = angle_to_servo_pwm(orientation.yaw);
```

### Adjust transmission rate
In `main.c`, line ~34:
```c
#define BLE_TX_INTERVAL_MS      50  // Change to 20 for 50Hz, 100 for 10Hz
```

### Tune sensor fusion
In `sensor_mpu6050.c` or `sensor.c`, adjust filter parameters:
```c
madgwick_init(&g_filter, 100.0f, 0.1f);
//                        ^       ^
//                        |       beta (filter gain)
//                        sample rate (Hz)
```

## Performance Metrics

| Metric                 | MPU6050  | ICM-20948 |
|-----------------------|----------|-----------|
| Update Rate           | 100 Hz   | 100 Hz    |
| BLE TX Rate           | 20 Hz    | 20 Hz     |
| Roll/Pitch Accuracy   | ±2°      | ±2°       |
| Yaw Accuracy          | Drifts   | ±5°       |
| Calibration Time      | ~2 sec   | ~2 sec    |
| Latency (IMU→BLE)     | ~60 ms   | ~60 ms    |

## License

See project root LICENSE file.
