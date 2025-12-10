#include "compass.h"
#include "led.h"

#define RAD_2_DEG (180.0f / M_PI)

/**
 * @brief Calculate the heading angles from magnetometer data
 * 
 */
heading_t compass_calc_heading(magnetometer_data_t mag_data) {
    heading_t heading;
    heading.radians = atan2f(mag_data.y, mag_data.x);
    if (heading.radians < 0) {
        heading.radians += 2 * M_PI;
    } else if (heading.radians > 2*M_PI) {
        heading.radians -= 2 * M_PI;
    }
    heading.degrees = heading.radians * RAD_2_DEG;
    return heading;


}

/**
 * @brief Print the heading angles
 * 
 */
void compass_print_heading(heading_t heading) {
    printf("Heading: %.2f degrees, %.2f radians\n", heading.degrees, heading.radians);
}


/**
 * @brief Default compass operation:
 *  - Reads data from the IMU
 *  - Calculates the heading
 *  - Changes the LED color based on heading
 */
void compass_default(MPU9250* imu){
    mpu9250_get_angles(imu);
    magnetometer_data_t mag_data;
    heading_t heading;
    mag_data.x = imu->mag_smooth[0];
    mag_data.y = imu->mag_smooth[1];
    mag_data.z = imu->mag_smooth[2];

    heading = compass_calc_heading(mag_data);
    compass_print_heading(heading);

    uint8_t r, g, b;
    //Map heading to RGB Color
    // Red at 0 degrees, Green at 120 degrees and Blue at 240 degrees
    // Blend between colors
    if (heading.degrees < 120) {
        r = (uint8_t)(255 - (heading.degrees / 120.0f) * 255);
        g = (uint8_t)((heading.degrees / 120.0f) * 255);
        b = 0;
    } else if (heading.degrees < 240) {
        r = 0;
        g = (uint8_t) (255 - (heading.degrees -120) / 120.0f * 255);
        b = (uint8_t) ((heading.degrees - 120) / 120.0f * 255);
    } else {  
        r = (uint8_t) ((heading.degrees - 240) / 120.0f * 255);
        g = 0;
        b = (uint8_t) (255 - (heading.degrees - 240) / 120.0f * 255);
    }
    printf("R: %d, G: %d, B: %d\t", r, g, b);
    led_set_color(255,255, 255);
    led_set_brightness(70);
    led_on();

}