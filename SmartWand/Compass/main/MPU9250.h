/*
*
* MPU9250 IMU Driver
*
* Author: Bradford Bawden
* Date: 08-1-2025
*/

#ifndef MPU9250_H
#define MPU9250_H

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

static const char *TAG = "mpu9250";

#define I2C_MASTER_SCL_IO           22       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21       /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          100000 /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define MPU9250_SENSOR_ADDR         0x68        /*!< Address of the MPU9250 sensor when AD0 is 0, Otherwise it is 0x69 */
#define MPU9250_FREQ                400000      /*!< Frequency at which the I2C registers can be accessed */
#define MPU9250_WHO_AM_I_REG_ADDR   0x75        /*!< Register addresses of the "who am I" register if it is 0x70 it is mpu6050 without a magnetometer*/
#define MPU9250_PWR_MGMT_1_REG_ADDR 0x6B        /*!< Register addresses of the power management register */
#define MPU9250_USER_CTRL          0x6A        /*!< Register address of the user control register */
#define MPU9250_RESET_BIT           7
#define MPU9250_NUM_IMU_VALUES      7         /*!< Number of IMU values to read from the MPU9250 */
#define MPU9250_NUM_MAG_VALUES      3         /*!< Number of magnetometer values to read */
#define MPU9250_NUM_IMU_BYTES       MPU9250_NUM_IMU_VALUES*2
#define MPU9250_NUM_MAG_BYTES       MPU9250_NUM_MAG_VALUES*2

#define MPU9250_ACC_XOUT_H          0X3B
#define MPU9250_ACC_XOUT_L          0X3C
#define MPU9250_ACC_YOUT_H          0X3D
#define MPU9250_ACC_YOUT_L          0X3E
#define MPU9250_ACC_ZOUT_H          0X3F
#define MPU9250_ACC_ZOUT_L          0X40
#define MPU9250_TEMP_OUT_H          0X41
#define MPU9250_TEMP_OUT_L          0X42
#define MPU9250_GY_XOUT_H           0X43
#define MPU9250_GY_XOUT_L           0X44
#define MPU9250_GY_YOUT_H           0X45
#define MPU9250_GY_YOUT_L           0X46
#define MPU9250_GY_ZOUT_H           0X47
#define MPU9250_GY_ZOUT_L           0X48

#define I2C_MASTER_CTRL             0x24        /*!< Register address of the I2C master control register */
#define I2C_SLV0_ADDR               0x25        /*!< Register address of the I2C slave 0 address */
#define I2C_SLV0_REG                0x26        /*!< Register address of the I2C slave 0 register */
#define I2C_SLV0_CTRL               0x27        /*!< Register address of the I2C slave 0 control register */
#define I2C_SENS_DATA_00            0x49        /*!< Register address of the external sensor data 0 register */

#define AK8963_I2C_ADDR             0x0C        /*!< Address of the AK8963 magnetometer */
#define MPU9250_MAG_WHO_AM_I        0x48        /*!< Register address of the AK8963*/

#define MPU9250_MAG_XOUT_H          0X04
#define MPU9250_MAG_XOUT_L          0X03
#define MPU9250_MAG_YOUT_H          0X06
#define MPU9250_MAG_YOUT_L          0X05
#define MPU9250_MAG_ZOUT_H          0X08
#define MPU9250_MAG_ZOUT_L          0X07
//IMU Constants
#define IMU_SENSOR_ADDR             0x68 // MPU6050 I2C Address
#define IMU_DATA_SIZE               14   // 7 numbers 2 bytes each (accel_x, accel_y, accel_z, temp, gyro_x, gyro_y, gyro_z)
#define IMU_START_ADDR              0x3B // Start reading data at accel_x


typedef struct {

    i2c_master_dev_handle_t dev_handle;

    int16_t accel_raw[3];
    int16_t temp_raw;
    int16_t gyro_raw[3];
    int16_t mag_raw[3];


} MPU9250;

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Read a sequence of bytes from a MPU9250 sensor registers
 */
esp_err_t mpu9250_register_read(MPU9250 *dev, uint8_t reg_addr, uint8_t *data);


/**
 * @brief Read a sequence of bytes from a MPU9250 sensor registers
 */
esp_err_t mpu9250_registers_read(MPU9250 *dev, uint8_t reg_addr, uint8_t *data, size_t len);


/**
 * @brief Write a byte to a MPU9250 sensor register
 */
esp_err_t mpu9250_register_write(MPU9250 *dev, uint8_t reg_addr, uint8_t data, uint8_t len);

/**
 * @brief i2c master initialization
 */
esp_err_t i2c_master_init();

/**
 * @brief Initialize the IMU device and wake it up
 */
esp_err_t mpu9250_init(MPU9250 *dev);

/**
 * @brief Get the registers that are inside the IMU including the Magnetometer
 */
esp_err_t mpu9250_read_imu(MPU9250 *dev);

/**
 * @brief Get the registers for the mganetometer data.
 */
esp_err_t mpu9250_read_mag(MPU9250 *dev);

/**
 * @brief Print the data from the IMU and Magnetometer neatly
 * @param dev Pointer to the MPU9250 device structure
 * Format:
 * Accel: x,   y,   z   Gyro: x,   y,   z   Temp: t   Mag: x,   y,   z 
 */
void mpu9250_print_data(MPU9250 *dev);



#ifdef __cplusplus
}
#endif

#endif // I2C_MASTER_H
