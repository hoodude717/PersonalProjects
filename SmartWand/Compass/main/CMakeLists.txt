idf_component_register(
    SRCS "main.c" "MPU9250.c" "madgwick/MadgwickAHRS.c"
    INCLUDE_DIRS "."  "madgwick"
    REQUIRES esp_driver_i2c esp_timer
)
