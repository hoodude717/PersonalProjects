#include <stdio.h>
#include "esp_mac.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "controls.h"
#include "led.h"
#include "imu.h"
#include "ble.h"
#include "nvs_flash.h"
#include "buffer.h"

typedef enum {
    RED,
    GREEN,
    BLUE,
    WHITE
} color;

void change_color_fsm() {
    static uint8_t color_st = RED;

    switch (color_st) {
        case RED:
            led_set_color(255, 25, 0);
            color_st = GREEN;
            break;
        case GREEN:
            led_set_color(0, 255, 0);
            color_st = BLUE;
            break;
        case BLUE:
            led_set_color(0, 0, 255);
            color_st = WHITE;
            break;
        case WHITE:
            led_set_color(255,255,255);
            // led_set_brightness(10);
            color_st = RED;
            break;
        default: 
            controls_toggle_led(false);
    }
}



void print_imu_data(void)
{
    // printf("Accel (g): ax=%.3f, ay=%.3f, az=%.3f\t", 
    //        imu_get_ax(), imu_get_ay(), imu_get_az());

    // printf("Gyro (rad/s): gx=%0.3f, gy=%0.3f, gz=%0.3f\t", 
    //        imu_get_gx(), imu_get_gy(), imu_get_gz());

    printf("Angles (rad): roll=%.3f, pitch=%.3f, yaw=%.3f\n", 
           imu_get_roll(), imu_get_pitch(), imu_get_yaw());


}


void imu_task(void *pvParameters)
{
    while (1) {

        if (controls_get_button()) {

            imu_tick();
            float yaw = imu_get_yaw();
            float pitch = imu_get_roll();
            // printf("Ax: %d\t Ay: %d\t Az: %d\n", 
            //        imu_get_ax_raw(), imu_get_ay_raw(), imu_get_az_raw());
            // printf("Gx: %d\t Gy: %d\t Gz: %d\n", 
            //        imu_get_gx_raw(), imu_get_gy_raw(), imu_get_gz_raw());

            // printf("Yaw: %02f\t Pitch: %02f\n", yaw, pitch);
            buffer_draw_pos(yaw, pitch);
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // 100 Hz
    }
}




void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    controls_init();
    imu_init();
    ble_init();

    //Create a Task which loops through and gets Imu data
    xTaskCreate(imu_task, "imu_task", 8192, NULL, 5, NULL);



    while (1) {
        if(controls_button_pressed()) {
            buffer_set_origin(imu_get_yaw(), imu_get_roll());
        }
        if (controls_get_button()) {
            // print_imu_data();
        }
        if(controls_button_released()) {
            buffer_on_button_release();
            buffer_convert_to_bitmap();
            buffer_clear_buffer();
        }


        // ble_send_orientation(
        //     imu_get_roll(), 
        //     imu_get_pitch(), 
        //     imu_get_yaw()
        // );



        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
