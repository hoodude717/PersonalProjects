/*
* File: controls.h
*
* Description: Top Level controls functions for the basic functions like GPIO and I2C and BLE
*
* Author: Bradford B. 
*
* Version: 1.0.1 05-13-2025
*/


#ifndef CONTROLS_H
#define CONTROLS_H
#include "stdint.h"
#include "stdbool.h"



#define ACCEL_ADDR 0x3C // I2C address for the device
#define I2C_NUM_BYTES 16 // Number of bytes to read from I2C
#define BLE_BUFFER_SIZE 128 // Size of the buffer for BLE communication
#define BLE_DEVICE_NAME "SmartWand" // Name of the BLE device
#define BLE_SERVICE_UUID "12345678-1234-5678-1234-56789abcdef0" // UUID for the BLE service
#define BLE_CHARACTERISTIC_UUID "12345678-1234-5678-1234-56789abcdef1" // UUID for the BLE characteristic
#define BLE_DESCRIPTOR_UUID "12345678-1234-5678-1234-56789abcdef2" // UUID for the BLE descriptor
#define BLE_MTU_SIZE 512 // Maximum Transmission Unit size for BLE communication
#define BLE_MAX_CONNECTIONS 4 // Maximum number of BLE connections
#define BLE_TIMEOUT 1000 // Timeout for BLE operations in milliseconds
#define I2C_TIMEOUT 1000 // Timeout for I2C operations in milliseconds
#define I2C_SDA_PIN 21 // SDA pin for I2C
#define I2C_SCL_PIN 22 // SCL pin for I2C
#define I2C_FREQUENCY 100000 // Frequency for I2C communication

//Initialize the pins for the basic controls
// Button Pin and LED pin
// I2C controls and Bluertooth
void controls_init();

// Returns if the pin is on or off. 
bool controls_get_button();

// Writes a message in the form of buffer to the client.
void controls_write_ble();

//Reads the message incoming from the BLE server. 
void controls_read_ble();

// Reads incoming data from the i2c line based on the address and the number of bytes per message. 
void controls_read_i2c(uint32_t addr, int8_t nbytes);

//Turns the led on or off based off flag
void controls_toggle_led(bool on);


#endif