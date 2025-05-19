/*
 * File: button.h
 *
 * Description: Interface for controlling buttons and handling button events.
 * 
 * Author: Bradford B.
 * 
 * Version: 1.0.1 05-13-2025
 */

#ifndef BUTTON_H
#define BUTTON_H

#include "stdint.h"
#include "stdbool.h"

#define BUTTON_PIN 12 // Pin number for the button
#define BUTTON_DEBOUNCE_TIME 100 // Debounce time in milliseconds
#define BUTTON_LONG_PRESS_TIME 1000 // Long press time in milliseconds
#define BUTTON_SHORT_PRESS_TIME 200 // Short press time in milliseconds


// Initializes the button
void button_init();

// Debounces the button input
bool button_pressed();

//clear the button event
void button_clear_event();



#endif // BUTTON_H