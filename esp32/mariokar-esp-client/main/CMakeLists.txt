idf_component_register(
    SRCS "main.c" "sensor.c" "sensor_mpu6050.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES nvs_flash bt esp_driver_usb_serial_jtag driver
)
