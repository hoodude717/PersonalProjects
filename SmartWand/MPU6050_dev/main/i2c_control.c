
#include "i2c_control.h"



i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t imu_handle;


/**
 * @brief Read the data from the MPU6050 accelerometer/gyroscope
 * @param data buffer for the data to be saved
 */
void i2c_read_imu_raw(uint8_t *data) {
    uint8_t reg_addr = IMU_START_ADDR;

    // Write register address then read 14 bytes
    ESP_ERROR_CHECK(i2c_master_transmit_receive(imu_handle, &reg_addr, 1, data, IMU_DATA_SIZE, -1)); 
}



// /**
//  * @brief Read a sequence of bytes from a MPU9250 sensor registers
//  */
// static esp_err_t mpu9250_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
// {
//     return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
// }

// /**
//  * @brief Write a byte to a MPU9250 sensor register
//  */
// static esp_err_t mpu9250_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
// {
//     uint8_t write_buf[2] = {reg_addr, data};
//     return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
// }

/**
 * @brief i2c master initialization
 */
void i2c_master_init()
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

}


void i2c_imu_init() {
    //add imu device
    i2c_device_config_t imu_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &imu_config, &imu_handle));


    uint8_t gyro_config_set_250dps[2] = {0x1B, 0x00}; // Register 0x1B, Value 0x00 (FS_SEL = 0)
    ESP_ERROR_CHECK(i2c_master_transmit(imu_handle, gyro_config_set_250dps, sizeof(gyro_config_set_250dps), -1));
    printf("Gyro FSR set to: ±250 dps\n"); // Print what you've actually set

    //Wake up the device
    uint8_t power_mgmt_write[2] = {0x6B, 0x00}; // Register, Value
    ESP_ERROR_CHECK(i2c_master_transmit(imu_handle, power_mgmt_write, sizeof(power_mgmt_write), -1));

    uint8_t gyro_config = 0;
    uint8_t reg_addr = 0x1B;
    
    ESP_ERROR_CHECK(i2c_master_transmit_receive(imu_handle, &reg_addr, 1, &gyro_config, 1, -1));
    
    uint8_t fs_sel = (gyro_config >> 3) & 0x03;
    float gyro_scale;
    switch (fs_sel) {
        case 0: gyro_scale = 131.0; printf("Gyro FSR: ±250 dps\n"); break;
        case 1: gyro_scale = 65.5;  printf("Gyro FSR: ±500 dps\n"); break;
        case 2: gyro_scale = 32.8;  printf("Gyro FSR: ±1000 dps\n"); break;
        case 3: gyro_scale = 16.4;  printf("Gyro FSR: ±2000 dps\n"); break;
        default: gyro_scale = 131.0; printf("Unknown FSR, assuming ±250 dps\n"); break;
    }
}



// void app_main(void)
// {
//     uint8_t data[2];
//     i2c_master_bus_handle_t bus_handle;
//     i2c_master_dev_handle_t dev_handle;
//     i2c_master_init(&bus_handle, &dev_handle);
//     ESP_LOGI(TAG, "I2C initialized successfully");

//     /* Read the MPU9250 WHO_AM_I register, on power up the register should have the value 0x71 */
//     ESP_ERROR_CHECK(mpu9250_register_read(dev_handle, MPU9250_WHO_AM_I_REG_ADDR, data, 1));
//     ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);

//     /* Demonstrate writing by resetting the MPU9250 */
//     ESP_ERROR_CHECK(mpu9250_register_write_byte(dev_handle, MPU9250_PWR_MGMT_1_REG_ADDR, 1 << MPU9250_RESET_BIT));

//     ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
//     ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
//     ESP_LOGI(TAG, "I2C de-initialized successfully");
// }