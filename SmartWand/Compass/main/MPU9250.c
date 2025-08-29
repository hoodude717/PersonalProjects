
#include "MPU9250.h"
#include "MadgwickAHRS.h"
#include <math.h>

#define ACCEL_SENSITIVITY 16384.0f // for +/- 2g
#define GYRO_SENSITIVITY 131.0f // for +/- 250 degrees/s
#define TEMP_SENSITIVITY 333.87f // LSB per degree C
#define TEMP_OFFSET 21.0f // in degree C
#define MAG_SENSITIVITY 0.15f // in microteslas per LSB
#define MAG_X_OFFSET 150
#define MAG_Y_OFFSET -75
#define MAG_Z_OFFSET -275

#define DEG_2_RAD (M_PI / 180.0f)


i2c_master_bus_handle_t bus_handle;
// i2c_master_dev_handle_t imu_handle;

// static const uint16_t mag_x_offset = 150;
// static const uint16_t mag_y_offset = -75;
// static const uint16_t mag_z_offset = -275;
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
esp_err_t mpu9250_register_write(MPU9250 *dev, uint8_t reg_addr, uint8_t data, uint8_t len) {
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev->dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS);
}

/**
 * @brief Write to AK8963 magnetometer register through MPU9250's auxiliary I2C
 */
esp_err_t ak8963_register_write(MPU9250 *dev, uint8_t reg_addr, uint8_t data) {
    // Set slave 4 address for write (clear read bit)
    ESP_ERROR_CHECK(mpu9250_register_write(dev, 0x31, AK8963_I2C_ADDR, 1)); // I2C_SLV4_ADDR
    // Set register address
    ESP_ERROR_CHECK(mpu9250_register_write(dev, 0x32, reg_addr, 1)); // I2C_SLV4_REG
    // Set data to write
    ESP_ERROR_CHECK(mpu9250_register_write(dev, 0x33, data, 1)); // I2C_SLV4_DO
    // Enable write
    ESP_ERROR_CHECK(mpu9250_register_write(dev, 0x34, 0x80, 1)); // I2C_SLV4_CTRL (enable)
    
    // Wait for transaction to complete
    vTaskDelay(pdMS_TO_TICKS(10));
    
    return ESP_OK;
}

/**
 * @brief Read from AK8963 magnetometer register through MPU9250's auxiliary I2C
 */
esp_err_t ak8963_register_read(MPU9250 *dev, uint8_t reg_addr, uint8_t *data) {
    // Set slave 4 address for read (set read bit)
    ESP_ERROR_CHECK(mpu9250_register_write(dev, 0x31, AK8963_I2C_ADDR | 0x80, 1)); // I2C_SLV4_ADDR
    // Set register address
    ESP_ERROR_CHECK(mpu9250_register_write(dev, 0x32, reg_addr, 1)); // I2C_SLV4_REG
    // Enable read
    ESP_ERROR_CHECK(mpu9250_register_write(dev, 0x34, 0x80, 1)); // I2C_SLV4_CTRL (enable)
    
    // Wait for transaction to complete
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Read the data
    ESP_ERROR_CHECK(mpu9250_register_read(dev, 0x35, data)); // I2C_SLV4_DI
    
    return ESP_OK;
}


static esp_err_t check_device(MPU9250 *dev) {
    uint8_t device_id = 0;
    ESP_ERROR_CHECK(mpu9250_register_read(dev, MPU9250_WHO_AM_I_REG_ADDR, &device_id));

    printf("INFO: MPU9250 Who am I? 0x%02X\n", device_id);
    if (device_id == 0x71) {
        printf("INFO: MPU9250 detected successfully\n");
        return ESP_OK;
    } else {
        printf("ERROR: Expected 0x71, got 0x%02X\n", device_id);
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
    uint8_t user_ctrl = 0;
    ESP_ERROR_CHECK(mpu9250_register_read(dev, MPU9250_USER_CTRL, &user_ctrl));
    user_ctrl |= 0x20;  // Set I2C_MST_EN
    user_ctrl &= ~0x02; // Clear I2C_IF_DIS (bypass)
    ESP_ERROR_CHECK(mpu9250_register_write(dev, MPU9250_USER_CTRL, user_ctrl, 1));
    printf("INFO: I2C master mode enabled\n");

    ESP_ERROR_CHECK(mpu9250_register_write(dev, I2C_MASTER_CTRL, 0x0D, 1));
    printf("INFO: I2C master clock set to 400kHz\n");

    vTaskDelay(pdMS_TO_TICKS(50)); 

    uint8_t ak8963_id = 0;
    ESP_ERROR_CHECK(ak8963_register_read(dev, 0x00, &ak8963_id)); // AK8963_WIA
    printf("INFO: AK8963 Who am I? 0x%02X\n", ak8963_id);
    if (ak8963_id != MPU9250_MAG_WHO_AM_I) {
        printf("ERROR: AK8963 not detected. Expected 0x48, got 0x%02X\n", ak8963_id);
        return ESP_FAIL;
    }

    // Reset the magnetometer
    ESP_ERROR_CHECK(ak8963_register_write(dev, 0x0B, 0x01)); // AK8963_CNTL2 - reset
    vTaskDelay(pdMS_TO_TICKS(100));
    // Set Magnetometer to continuous measurement mode 2 
    ESP_ERROR_CHECK(ak8963_register_write(dev, 0x0A, 0x16)); // AK8963_CNTL1
    vTaskDelay(pdMS_TO_TICKS(100));

    // Configure the MPU9250 to read from the magnetometer
    // Set the address in first slv0 for  magnetometer data
    ESP_ERROR_CHECK(mpu9250_register_write(dev, I2C_SLV0_ADDR, (AK8963_I2C_ADDR | 0x80), 1)); // Set the 7th bit for read mode
    ESP_ERROR_CHECK(mpu9250_register_write(dev, I2C_SLV0_REG, MPU9250_MAG_XOUT_L, 1)); // Start reading at HXL
    ESP_ERROR_CHECK(mpu9250_register_write(dev, I2C_SLV0_CTRL, 0x87, 1)); // Enable and set read to 7 bytes

    printf("INFO: Magnetometer initialxized successfully\n");
    return ESP_OK;

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
    uint8_t mag_data[MPU9250_NUM_MAG_BYTES+1]; // Extra bit for the status

    ESP_ERROR_CHECK(mpu9250_registers_read(dev, I2C_SENS_DATA_00, mag_data, MPU9250_NUM_MAG_BYTES+1));

    // Check if data is ready (ST2 register bit 3 should be 0 for valid data)
    if (mag_data[6] & 0x08) {
        printf("WARNING: Magnetometer overflow detected\n");
        return ESP_FAIL;
    }

    // AK8963 stores data in little-endian format (LSB first)
    dev->mag_raw[0] = (int16_t)(mag_data[1] << 8 | mag_data[0]) + MAG_X_OFFSET; // X
    dev->mag_raw[1] = (int16_t)(mag_data[3] << 8 | mag_data[2]) + MAG_Y_OFFSET; // Y  
    dev->mag_raw[2] = (int16_t)(mag_data[5] << 8 | mag_data[4]) + MAG_Z_OFFSET; // Z

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

esp_err_t mpu9250_convert_data(MPU9250* dev) {
    mpu9250_read_imu(dev);
    mpu9250_read_mag(dev);
    dev->accel_smooth[0] = dev->accel_raw[0] / ACCEL_SENSITIVITY; 
    dev->accel_smooth[1] = dev->accel_raw[1] / ACCEL_SENSITIVITY; 
    dev->accel_smooth[2] = dev->accel_raw[2] / ACCEL_SENSITIVITY; 

    dev->gyro_smooth[0] = dev->gyro_raw[0] / GYRO_SENSITIVITY; 
    dev->gyro_smooth[1] = dev->gyro_raw[1] / GYRO_SENSITIVITY; 
    dev->gyro_smooth[2] = dev->gyro_raw[2] / GYRO_SENSITIVITY; 

    dev->temp_c = (dev->temp_raw / TEMP_SENSITIVITY) + TEMP_OFFSET;

    dev->mag_smooth[0] = dev->mag_raw[0] / MAG_SENSITIVITY; 
    dev->mag_smooth[1] = dev->mag_raw[1] / MAG_SENSITIVITY; 
    dev->mag_smooth[2] = dev->mag_raw[2] / MAG_SENSITIVITY; 
    return ESP_OK;
}

void mpu9250_print_data_smooth(MPU9250 *dev) {
    float a_x = dev->accel_smooth[0];
    float a_y = dev->accel_smooth[1];
    float a_z = dev->accel_smooth[2];
    float g_x = dev->gyro_smooth[0];
    float g_y = dev->gyro_smooth[1];
    float g_z = dev->gyro_smooth[2];
    float t = dev->temp_c;
    float m_x = dev->mag_smooth[0];
    float m_y = dev->mag_smooth[1];
    float m_z = dev->mag_smooth[2];
    printf("Accel: %.2f,\t%.2f,\t%.2f\tGyro: %.2f,\t%.2f,\t%.2f\tTemp: %.2f\tMag: %.2f,\t%.2f,\t%.2f\n", 
        a_x, a_y, a_z,
        g_x, g_y, g_z,
        t,
        m_x, m_y, m_z
    );
}


void mpu9250_get_angles(MPU9250* dev) {
    mpu9250_convert_data(dev);

    // Madgwick filter update
    float q[4];
    float q0,q1,q2,q3;
    MadgwickAHRSupdate(
        dev->gyro_smooth[0] * DEG_2_RAD,
        dev->gyro_smooth[1] * DEG_2_RAD,
        dev->gyro_smooth[2] * DEG_2_RAD,
        dev->accel_smooth[0],
        dev->accel_smooth[1],
        dev->accel_smooth[2],
        dev->mag_smooth[0],
        dev->mag_smooth[1],
        dev->mag_smooth[2]
    );
    MadgwickGetQuaternion(q);
    q0=q[0]; q1=q[1]; q2=q[2]; q3=q[3];
    //Calculate Euclidean Angles
    float roll, pitch, yaw;
    roll  = atan2f(2.0f * (q0*q1 + q2*q3), 1.0f - 2.0f * (q1*q1 + q2*q2));
    pitch = asinf(2.0f * (q0*q2 - q3*q1));
    yaw   = atan2f(2.0f * (q0*q3 + q1*q2), 1.0f - 2.0f * (q2*q2 + q3*q3));

    dev->roll = roll * (180.0f / M_PI);
    dev->pitch = pitch * (180.0f / M_PI);
    dev->yaw = yaw * (180.0f / M_PI);

}