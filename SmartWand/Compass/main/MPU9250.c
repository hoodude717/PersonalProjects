
#include "MPU9250.h"



i2c_master_bus_handle_t bus_handle;
// i2c_master_dev_handle_t imu_handle;


// /**
//  * @brief Read the data from the MPU6050 accelerometer/gyroscope
//  * @param data buffer for the data to be saved
//  */
// void i2c_read_imu_raw(uint8_t *data) {
//     uint8_t reg_addr = IMU_START_ADDR;

//     // Write register address then read 14 bytes
//     ESP_ERROR_CHECK(i2c_master_transmit_receive(imu_handle, &reg_addr, 1, data, IMU_DATA_SIZE, -1)); 
// }



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

// /**
//  * @brief i2c master initialization
//  */
// void i2c_master_init()
// {
//     i2c_master_bus_config_t bus_config = {
//         .i2c_port = I2C_MASTER_NUM,
//         .sda_io_num = I2C_MASTER_SDA_IO,
//         .scl_io_num = I2C_MASTER_SCL_IO,
//         .clk_source = I2C_CLK_SRC_DEFAULT,
//         .glitch_ignore_cnt = 7,
//         .flags.enable_internal_pullup = true,
//     };
//     ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

// }


// void i2c_imu_init() {
//     //add imu device
//     i2c_device_config_t imu_config = {
//         .dev_addr_length = I2C_ADDR_BIT_LEN_7,
//         .device_address = IMU_SENSOR_ADDR,
//         .scl_speed_hz = I2C_MASTER_FREQ_HZ,
//     };
//     ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &imu_config, &imu_handle));


//     uint8_t gyro_config_set_250dps[2] = {0x1B, 0x00}; // Register 0x1B, Value 0x00 (FS_SEL = 0)
//     ESP_ERROR_CHECK(i2c_master_transmit(imu_handle, gyro_config_set_250dps, sizeof(gyro_config_set_250dps), -1));
//     printf("Gyro FSR set to: ±250 dps\n"); // Print what you've actually set

//     //Wake up the device
//     uint8_t power_mgmt_write[2] = {0x6B, 0x00}; // Register, Value
//     ESP_ERROR_CHECK(i2c_master_transmit(imu_handle, power_mgmt_write, sizeof(power_mgmt_write), -1));

//     uint8_t gyro_config = 0;
//     uint8_t reg_addr = 0x1B;
    
//     ESP_ERROR_CHECK(i2c_master_transmit_receive(imu_handle, &reg_addr, 1, &gyro_config, 1, -1));
    
//     uint8_t fs_sel = (gyro_config >> 3) & 0x03;
//     float gyro_scale;
//     switch (fs_sel) {
//         case 0: gyro_scale = 131.0; printf("Gyro FSR: ±250 dps\n"); break;
//         case 1: gyro_scale = 65.5;  printf("Gyro FSR: ±500 dps\n"); break;
//         case 2: gyro_scale = 32.8;  printf("Gyro FSR: ±1000 dps\n"); break;
//         case 3: gyro_scale = 16.4;  printf("Gyro FSR: ±2000 dps\n"); break;
//         default: gyro_scale = 131.0; printf("Unknown FSR, assuming ±250 dps\n"); break;
//     }
// }



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




/**
 * @brief Read a single byte from a MPU9250 sensor registers
 */
esp_err_t mpu9250_register_read(MPU9250 *dev, uint8_t reg_addr, uint8_t *data) {
    return i2c_master_transmit_receive(dev->dev_handle, &reg_addr, 1, data, 1, I2C_MASTER_TIMEOUT_MS);
}


/**
 * @brief Read a sequence of bytes from a MPU9250 sensor registers
 */
esp_err_t mpu9250_registers_read(MPU9250 *dev, uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_transmit_receive(dev->dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS);
}


/**
 * @brief Write a byte to a MPU9250 sensor register
 */
esp_err_t mpu9250_register_write(MPU9250 *dev, uint8_t reg_addr, uint8_t *data, uint8_t len) {
    return i2c_master_transmit(dev->dev_handle, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t check_device(MPU9250 *dev) {
    uint8_t device_id = 0;
    ESP_ERROR_CHECK(mpu9250_register_read(dev, MPU9250_WHO_AM_I_REG_ADDR, &device_id));

    printf("INFO: Device Who am I? 0x%02X\n", device_id);
    if (device_id == 0x71){
        return ESP_OK;
    } else {
        return ESP_FAIL;
    }
}



/**
 * @brief Initialize the IMU device and wake it up
 */
esp_err_t mpu9250_init(MPU9250 *dev) {
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    //add imu device
    i2c_device_config_t mcu9250_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU9250_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &mcu9250_config, &dev->dev_handle));

    // // Set up the magnetometer for reading. 
    // uint8_t user_ctrl_data = 0x20; // I2C_MST_EN = 1
    // ESP_ERROR_CHECK(mpu9250_register_read(dev, MPU9250_USER_CTRL, &user_ctrl_data));
    // user_ctrl_data |= 0x20;  // Set I2C_MST_EN
    // user_ctrl_data &= ~0x02; // Clear BYPASS_EN
    // ESP_ERROR_CHECK(mpu9250_register_write(dev, MPU9250_USER_CTRL, &user_ctrl_data, 1));

    // // Set I2C master clock to 400kHz
    // user_ctrl_data = 0x0D;  // 400kHz
    // ESP_ERROR_CHECK(mpu9250_register_write(dev, I2C_MASTER_CTRL, &user_ctrl_data, 1));
    
    // // Set the address in first slv0 for  magnetometer data
    // uint8_t address_data = AK8963_I2C_ADDR | 0x80; // 0x0C | 0x80 (read bit)
    // ESP_ERROR_CHECK(mpu9250_register_write(dev, I2C_SLV0_ADDR, &address_data, 1));
    // // Set starting register (HXL = 0x03)
    // uint8_t start_pnt = MPU9250_MAG_XOUT_L;
    // ESP_ERROR_CHECK(mpu9250_register_write(dev, I2C_SLV0_REG, &start_pnt, 1));
    // // Enable and set to read 7 bytes
    // uint8_t slv_ctrl = 0x87;  // Enable + 7 bytes
    // ESP_ERROR_CHECK(mpu9250_register_write(dev, I2C_SLV0_CTRL, &slv_ctrl, 1));

    // uint8_t mode = 0x16;  // 16-bit + 100Hz continuous
    // // Write to AK8963 CNTL1 register (0x0A)
    // ESP_ERROR_CHECK(mpu9250_register_write(dev, 0x0A, &mode, 1));

    return check_device(dev);

}

/**
 * @brief Get the registers that are inside the IMU including the Magnetometer
 */
esp_err_t mpu9250_read_imu(MPU9250 *dev) {
    uint8_t imu_data[MPU9250_NUM_IMU_BYTES];

    ESP_ERROR_CHECK(mpu9250_registers_read(dev, MPU9250_ACC_XOUT_H, imu_data, MPU9250_NUM_IMU_BYTES));
    
    dev->accel_raw[0] = (int16_t)((imu_data[0] << 8) | imu_data[1]);
    dev->accel_raw[1] = (int16_t)((imu_data[2] << 8) | imu_data[3]);
    dev->accel_raw[2] = (int16_t)((imu_data[4] << 8) | imu_data[5]);
    dev->temp_raw     = (int16_t)((imu_data[6] << 8) | imu_data[7]);
    dev->gyro_raw[0]  = (int16_t)((imu_data[8] << 8) | imu_data[9]);
    dev->gyro_raw[1]  = (int16_t)((imu_data[10] << 8) | imu_data[11]);
    dev->gyro_raw[2]  = (int16_t)((imu_data[12] << 8) | imu_data[13]);


    return ESP_OK;

}

/**
 * @brief Get the registers for the mganetometer data.
 */
esp_err_t mpu9250_read_mag(MPU9250 *dev){
    uint8_t mag_data[MPU9250_NUM_MAG_BYTES];

    ESP_ERROR_CHECK(mpu9250_registers_read(dev, I2C_SENS_DATA_00, mag_data, MPU9250_NUM_MAG_BYTES));

    dev->mag_raw[0] = (mag_data[0] << 8) | mag_data[1];
    dev->mag_raw[1] = (mag_data[2] << 8) | mag_data[3];
    dev->mag_raw[2] = (mag_data[4] << 8) | mag_data[5];
    
    return ESP_OK;
}

/**
 * @brief Print the data from the IMU and Magnetometer n eatly
 * @param dev Pointer to the MPU9250 device structure
 * Format:
 * Accel: x,   y,   z   Gyro: x,   y,   z   Temp: t   Mag: x,   y,   z 
 */
void mpu9250_print_data(MPU9250 *dev) {
    int16_t a_x = dev->accel_raw[0];
    int16_t a_y = dev->accel_raw[1];
    int16_t a_z = dev->accel_raw[2];
    int16_t g_x = dev->gyro_raw[0];
    int16_t g_y = dev->gyro_raw[1];
    int16_t g_z = dev->gyro_raw[2];
    int16_t t = dev->temp_raw;
    int16_t m_x = dev->mag_raw[0];
    int16_t m_y = dev->mag_raw[1];
    int16_t m_z = dev->mag_raw[2];
    printf("Accel: %d,\t%d,\t%d\tGyro: %d,\t%d,\t%d\tTemp: %d\tMag: %d,\t%d,\t%d\n", 
        a_x, a_y, a_z,
        g_x, g_y, g_z,
        t,
        m_x, m_y, m_z
    );
}