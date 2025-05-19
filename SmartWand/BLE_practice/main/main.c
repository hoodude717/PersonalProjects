#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "controls.h"
#include "led.h"

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
            led_set_color(10, 1, 0);
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
            led_set_color(255,0,255);
            led_set_brightness(10);
            color_st = RED;
            break;
        default: 
            controls_toggle_led(false);
    }
}

void app_main(void)
{
    controls_init();

    bool waiting_for_release = false;

    while (1) {
        if (!waiting_for_release && controls_get_button()) {
            change_color_fsm();
            controls_toggle_led(true);
            waiting_for_release = true;
        } else if (waiting_for_release && !gpio_get_level(12)) {
            // still held, do nothing
        } else if (waiting_for_release && gpio_get_level(12)) {
            // button released
            waiting_for_release = false;
            controls_toggle_led(false);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
