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
        mpu9250_read_imu(&l_imu);
        mpu9250_read_mag(&l_imu);
        
        // Print in CSV format for easy parsing
        int64_t timestamp = esp_timer_get_time() / 1000;
        printf("MAG_DATA: %d,%d,%d,%d,%lld\n", 
               sample_count++,
               l_imu.mag_raw[0]+150,  //Offsets calculated from previous calibration data
               l_imu.mag_raw[1]-75, 
               l_imu.mag_raw[2]-250,
               timestamp);
        
        if (sample_count % 100 == 0) {
            printf("INFO: Logged %d samples\n", sample_count);
        }
        
        vTaskDelay(pdMS_TO_TICKS(50)); // 20Hz
    }
}