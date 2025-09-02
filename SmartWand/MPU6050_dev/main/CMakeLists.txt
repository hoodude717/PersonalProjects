idf_component_register(
    SRCS "i2c_control.c" "main.c" "led.c" "button.c" "controls.c" "madgwick/MadgwickAHRS.c" "imu.c" "ble.c" "buffer.c"
    INCLUDE_DIRS "." "madgwick"
    REQUIRES esp_driver_i2c esp_driver_ledc esp_driver_gpio bt nvs_flash
)
