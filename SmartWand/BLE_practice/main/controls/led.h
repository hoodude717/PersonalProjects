/*
* File: led.h
*
* Description: Intereface for controlling an RGB LED using PWM. Using the IDF LEDC Configuration
*
* Author: Bradford B. 
*
* Version: 1.0.1 05-13-2025
*/

#ifndef LED_H
#define LED_H
#include "stdint.h"
#include "stdbool.h"


//Configurations for the LED
void configure_ledc_channel(ledc_channel_t channel, int gpio_num);

//Initializes the LED hardware
void led_init(void);

// Turns the LED on
void led_on(void);

// Turns the LED off
void led_off(void);

// Toggles the LED state
void led_toggle(bool state);

// Sets the color of the LED (RGB)
// r, g, b values should be in the range of 0-255
void led_set_color(uint8_t r, uint8_t g, uint8_t b);

// Set the brightness of the LED (0-255)
// This can be done by adjusting the duty cycle
void led_set_brightness(uint8_t brightness);


#endif // LED_H