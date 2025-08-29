#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "MPU9250.h"
#include "esp_timer.h"




void app_main(void) {
    printf("CALIBRATION_START\n"); // Marker for Python script
    
    MPU9250 l_imu;
    ESP_ERROR_CHECK(mpu9250_init(&l_imu));
    
    printf("CSV_HEADER: sample,mag_x,mag_y,mag_z,timestamp_ms\n");
    
    int sample_count = 0;
    
    while(1) {

        mpu9250_get_angles(&l_imu);
        // mpu9250_convert_data(&l_imu);
        
        // Print in CSV format for easy parsing
        // int64_t timestamp = esp_timer_get_time() / 1000;
        // mpu9250_print_data_smooth(&l_imu);
        printf("Pitch: %.2f, Roll: %.2f, Yaw: %.2f\n", 
            l_imu.pitch, l_imu.roll, l_imu.yaw);
            
        vTaskDelay(pdMS_TO_TICKS(50)); // 20Hz
    }
}