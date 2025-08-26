#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "MPU9250.h"




void app_main(void){

    printf("Hello World\n");

    MPU9250 l_imu;
    ESP_ERROR_CHECK(mpu9250_init(&l_imu));

    while(1) {
        mpu9250_read_imu(&l_imu);
        mpu9250_read_mag(&l_imu);
        mpu9250_print_data(&l_imu); 
        vTaskDelay(pdMS_TO_TICKS(100));  // 1 second delay

    }
}