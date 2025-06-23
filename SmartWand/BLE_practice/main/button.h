/*
 * File: button.h
 *
 * Description: Interface for controlling buttons and handling button events.
 * 
 * Author: Bradford B.
 * 
 * Version: 1.0.1 05-13-2025
 */
// button.h
#ifndef BUTTON_H
#define BUTTON_H

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h" // Add this include for QueueHandle_t

#define BUTTON_PIN 26 // Pin number for the button
#define BUTTON_DEBOUNCE_TIME 100 // Debounce time in milliseconds
#define BUTTON_LONG_PRESS_TIME 1000 // Long press time in milliseconds
#define BUTTON_SHORT_PRESS_TIME 200 // Short press time in milliseconds

// Define an event type for button presses/releases
typedef enum {
    BUTTON_EVENT_NONE = 0,
    BUTTON_EVENT_PRESSED,
    BUTTON_EVENT_RELEASED
} button_event_t;

// Declare a queue handle (make it extern if defined in button.c and used elsewhere)
extern QueueHandle_t xButtonEventQueue; // This queue will carry button events

void button_init();
bool button_get_level();
bool button_pressed();
bool button_released();

#endif // BUTTON_H