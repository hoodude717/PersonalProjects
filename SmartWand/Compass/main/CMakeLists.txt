idf_component_register(
    SRCS "main.c" "MPU9250.c" "madgwick/MadgwickAHRS.c" "compass.c" "led.c"
    INCLUDE_DIRS "."  "madgwick"
    REQUIRES esp_driver_i2c esp_timer esp_driver_ledc esp_driver_gpio nvs_flash
)
