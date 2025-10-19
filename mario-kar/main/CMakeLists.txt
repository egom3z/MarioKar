idf_component_register(SRCS "main.c" "led.c" "imu.c" "imu_filters.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver)
