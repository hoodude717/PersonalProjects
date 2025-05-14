#include "controls.h"



//Initialize the pins for the basic controls
// Button Pin and LED pin
// I2C controls and Bluertooth
void controls_init(){
    gpio_set_direction();
}

// Returns if the pin is on or off. 
bool controls_pin_on(int8_t pin){

}

// Writes a message in the form of buffer to the client.
void controls_write_ble(){

}

//Reads the message incoming from the BLE server. 
void controls_read_ble(){

}

// Reads incoming data from the i2c line based on the address and the number of bytes per message. 
void controls_read_i2c(uint32_t addr, int8_t nbytes){

}

//Turns the led on or off based off flag
void controls_toggle_led(int8_t pin, bool on){

}