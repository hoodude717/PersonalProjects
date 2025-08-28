idf_component_register(
    SRCS "main.c" "MPU9250.c" 
    INCLUDE_DIRS "." 
    REQUIRES esp_driver_i2c esp_timer
)
