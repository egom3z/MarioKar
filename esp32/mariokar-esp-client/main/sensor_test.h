// Quick test to verify sensor selection
#include "sensor.h"

#if IMU_SENSOR_TYPE == IMU_TYPE_MPU6050
#pragma message "Sensor selection: MPU6050 (I2C) - CORRECT"
#elif IMU_SENSOR_TYPE == IMU_TYPE_ICM20948  
#pragma message "Sensor selection: ICM20948 (SPI) - WRONG, change sensor.h"
#else
#error "IMU_SENSOR_TYPE not defined correctly!"
#endif

