#ifndef I2C_CONTROL_H
#define I2C_CONTROL_H

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

static const char *TAG = "i2c";

#define I2C_MASTER_SCL_IO           22       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21       /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          100000 /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define MPU9250_SENSOR_ADDR         0x68        /*!< Address of the MPU9250 sensor */
#define MPU9250_WHO_AM_I_REG_ADDR   0x75        /*!< Register addresses of the "who am I" register */
#define MPU9250_PWR_MGMT_1_REG_ADDR 0x6B        /*!< Register addresses of the power management register */
#define MPU9250_RESET_BIT           7

//IMU Constants
#define IMU_SENSOR_ADDR             0x68 // MPU6050 I2C Address
#define IMU_DATA_SIZE               14   // 7 numbers 2 bytes each (accel_x, accel_y, accel_z, temp, gyro_x, gyro_y, gyro_z)
#define IMU_START_ADDR              0x3B // Start reading data at accel_x

//Screen Constants
#define SCREEN_WIDTH                128 // OLED display width, in pixels
#define SCREEN_HEIGHT               64 // OLED display height, in pixels
#define OLED_RESET                  -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADR                  0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32



#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Read the data from the MPU6050 accelerometer/gyroscope
 * @param data buffer for the data to be saved
 */
void i2c_read_imu_raw(uint8_t *data);

// /**
//  * @brief Read a sequence of bytes from a MPU9250 sensor registers
//  */
// esp_err_t mpu9250_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len);


// /**
//  * @brief Write a byte to a MPU9250 sensor register
//  */
// esp_err_t mpu9250_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data);

/**
 * @brief i2c master initialization
 */
void i2c_master_init();

/**
 * @brief Initialize the IMU device and wake it up
 */
void i2c_imu_init();

#ifdef __cplusplus
}
#endif

#endif // I2C_MASTER_H
