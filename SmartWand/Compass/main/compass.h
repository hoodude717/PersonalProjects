#ifndef COMPASS_H
#define COMPASS_H

#include <stdio.h>
#include <math.h>
#include "MPU9250.h"
#include <stdint.h>


typedef struct {
    float x;
    float y;
    float z;
} magnetometer_data_t;

typedef struct {
    float degrees;
    float radians;
} heading_t;

/**
 * @brief Calculate the heading angles from magnetometer data
 * 
 */
heading_t compass_calc_heading(magnetometer_data_t mag_data);

/**
 * @brief Print the heading angles
 * 
 */
void compass_print_heading(heading_t heading);


/**
 * @brief Default compass operation:
 *  - Reads data from the IMU
 *  - Calculates the heading
 *  - Changes the LED color based on heading
 */
void compass_default(MPU9250* imu);


#endif // COMPASS_H