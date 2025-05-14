#include "led.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_HIGH_SPEED_MODE
#define LEDC_OUTPUT_R           (18)  // Red channel GPIO
#define LEDC_OUTPUT_G           (19)  // Green channel GPIO
#define LEDC_OUTPUT_B           (21)  // Blue channel GPIO
#define LEDC_CHANNEL_R          LEDC_CHANNEL_0
#define LEDC_CHANNEL_G          LEDC_CHANNEL_1
#define LEDC_CHANNEL_B          LEDC_CHANNEL_2
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT
#define LEDC_FREQUENCY          (5000) // 5 kHz

static uint8_t red, green, blue;
void configure_ledc_channel(ledc_channel_t channel, int gpio_num) {
    ledc_channel_config_t ledc_channel = {
        .channel    = channel,
        .duty       = 0,
        .gpio_num   = gpio_num,
        .speed_mode = LEDC_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER
    };
    ledc_channel_config(&ledc_channel);
}

void led_init(void) {
    // Initialize the LED hardware (e.g., configure GPIO pins)
    // Set up the timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);
    // Configure the LED channels
    configure_ledc_channel(LEDC_CHANNEL_R, LEDC_OUTPUT_R);
    configure_ledc_channel(LEDC_CHANNEL_G, LEDC_OUTPUT_G);
    configure_ledc_channel(LEDC_CHANNEL_B, LEDC_OUTPUT_B);

    led_set_color(0, 0, 0); // Initialize LED to off
}

void led_on(void) {
    // Turn the LED on
    // Set the color to the saved values
    led_set_color(red, green, blue); // Set color to the last set color

}

void led_off(void) {
    // Turn the LED off
    led_set_color(0, 0, 0); // Set color to off (black)

}

void led_toggle(bool state) {
    // Toggle the LED state

    if(state) {
        led_on();
    } else {
        led_off();
    }
}

void led_set_color(uint8_t r, uint8_t g, uint8_t b) {
    // Set the color of the LED (RGB)
    // r, g, b values should be in the range of 0-255
    red = r;
    green = g;
    blue = b;
    // Set the duty cycle for each channel
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_R, r);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_R);
    
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_G, g);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_G);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_B, b);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_B);
}

void led_set_brightness(uint8_t brightness) {
    // Set the brightness of the LED (0-255)
    // This can be done by adjusting the duty cycle
    uint8_t r = (red * brightness) / 255;
    uint8_t g = (green * brightness) / 255;
    uint8_t b = (blue * brightness) / 255;
    led_set_color(r, g, b);
}