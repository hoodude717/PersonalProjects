#include "imu.h"
#include "i2c_control.h"
#include "MadgwickAHRS.h"
#include <stdio.h>

#include <math.h>    // For angle calculations
#include <string.h>  // For memset (if needed)
#include <stdio.h>   // For debug printing (optional)

#define NUM_QUATERN 4
#define NUM_IMU_VALUES 7
#define BUF_SIZE (NUM_IMU_VALUES * 2)

#define RAW_2_G  16384.0f //Dividing each accelerometer value by this will convert to +-2g.
#define RAW_2_DEG_PER_SEC 131.0f // Dividing each gyro value by this will convert to degrees per second
#define DEG_2_RAD (M_PI / 180.0f)
#define RAD_2_DEG (180.0f / 3.14)
#define RAW_2_RAD_PER_SEC (DEG_2_RAD / RAW_2_DEG_PER_SEC)

// Static variables to hold the latest sensor data
static imu_raw_data_t imu_latest_raw_data = {0};
static imu_data_t imu_latest_data = {0};
static int16_t imu_raw_temp = 0;
static imu_angles_t imu_latest_angles = {0};
float gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;

static int16_t gx_bias = -296;
static int16_t gy_bias = -8;
static int16_t gz_bias = -87;

// ===================== Public Function Implementations =====================

// Initializes the IMU device
int imu_init(void)
{
    //Initialize and wake up Master and Devices
    i2c_master_init();   
    i2c_imu_init();
    //Get the first read of the data
    uint8_t data[BUF_SIZE];
    imu_get_sensor_data(data);
    imu_get_gyro_bias();
    return 0;
}


/**
 * @brief tick function to continuously call and update the data
 */
void imu_tick(void){
    uint8_t data[BUF_SIZE];
    imu_get_sensor_data(data);
    imu_calculate_angles();

}

// Reads accelerometer and gyroscope data from IMU
int imu_get_sensor_data(uint8_t *data)
{
    i2c_read_imu_raw(data);
    imu_latest_raw_data.ax = (data[0] << 8) | data[1];
    imu_latest_raw_data.ay = (data[2] << 8) | data[3];
    imu_latest_raw_data.az = (data[4] << 8) | data[5];
    imu_raw_temp           = (data[6] << 8) | data[7];
    imu_latest_raw_data.gx = ((data[8] << 8) | data[9]);
    imu_latest_raw_data.gy = ((data[10] << 8) | data[11]);
    imu_latest_raw_data.gz = ((data[12] << 8) | data[13]);

    imu_latest_data.ax = (float)imu_latest_raw_data.ax / RAW_2_G;
    imu_latest_data.ay = (float)imu_latest_raw_data.ay / RAW_2_G;
    imu_latest_data.az = (float)imu_latest_raw_data.az / RAW_2_G;
    imu_latest_data.gx = ((float)imu_latest_raw_data.gx - gx_bias) / RAW_2_DEG_PER_SEC * DEG_2_RAD;
    imu_latest_data.gy = ((float)imu_latest_raw_data.gy - gy_bias) / RAW_2_DEG_PER_SEC * DEG_2_RAD;
    imu_latest_data.gz = ((float)imu_latest_raw_data.gz - gz_bias) / RAW_2_DEG_PER_SEC * DEG_2_RAD;

    return 0;
}

// Calculates roll, pitch, and yaw based on accelerometer/gyro data
void imu_calculate_angles()
{
    //Gather quaternions for calculations
    float q[NUM_QUATERN];
    float q0,q1,q2,q3;
    MadgwickAHRSupdateIMU(
        imu_latest_data.gx, 
        imu_latest_data.gy, 
        imu_latest_data.gz, 
        imu_latest_data.ax, 
        imu_latest_data.ay, 
        imu_latest_data.az
    );
    MadgwickGetQuaternion(q);
    q0=q[0]; q1=q[1]; q2=q[2]; q3=q[3];
    //Calculate Euclidean Angles
    float roll, pitch, yaw;
    roll  = atan2f(2.0f * (q0*q1 + q2*q3), 1.0f - 2.0f * (q1*q1 + q2*q2));
    pitch = asinf(2.0f * (q0*q2 - q3*q1));
    yaw   = atan2f(2.0f * (q0*q3 + q1*q2), 1.0f - 2.0f * (q2*q2 + q3*q3));
    //Save the angles in the imu_angles_t as degrees
    roll    *= RAD_2_DEG;
    pitch   *= RAD_2_DEG;
    yaw     *= RAD_2_DEG;
    imu_latest_angles.roll  = roll;
    imu_latest_angles.pitch = pitch;
    imu_latest_angles.yaw   = yaw;
}

void imu_get_gyro_bias(void){
    float gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;
    uint8_t data[BUF_SIZE];
    for (int i = 0; i < 500; i++) {
        imu_get_sensor_data(data);
        gyro_bias_x += imu_latest_raw_data.gx;
        gyro_bias_y += imu_latest_raw_data.gy;
        gyro_bias_z += imu_latest_raw_data.gz;
    }
    gyro_bias_x /= 500.0f;
    gyro_bias_y /= 500.0f;
    gyro_bias_z /= 500.0f;

    gx_bias = (int16_t)gyro_bias_x;
    gy_bias = (int16_t)gyro_bias_y;
    gz_bias = (int16_t)gyro_bias_z;

    // In your update loop:
    printf("Bias X: %f | Y: %f | X: %f \n", gyro_bias_x, gyro_bias_y, gyro_bias_z);
    // imu_latest_raw_data.gx -= gyro_bias_x;
    // imu_latest_raw_data.gy -= gyro_bias_y;
    // imu_latest_raw_data.gz -= gyro_bias_z;
}

void imu_reset_angles() {
    printf("Resetting the Angles\n");
    imu_latest_angles.pitch = 0;
    imu_latest_angles.roll = 0;
    imu_latest_angles.yaw = 0;
}
// ===================== Accessor Functions =====================

float imu_get_ax(void) { return imu_latest_data.ax; }
float imu_get_ay(void) { return imu_latest_data.ay; }
float imu_get_az(void) { return imu_latest_data.az; }

int16_t imu_get_ax_raw(void) { return imu_latest_raw_data.ax; }
int16_t imu_get_ay_raw(void) { return imu_latest_raw_data.ay; }
int16_t imu_get_az_raw(void) { return imu_latest_raw_data.az; }

float imu_get_gx(void) { return imu_latest_data.gx; }
float imu_get_gy(void) { return imu_latest_data.gy; }
float imu_get_gz(void) { return imu_latest_data.gz; }

int16_t imu_get_gx_raw(void) { return imu_latest_raw_data.gx; }
int16_t imu_get_gy_raw(void) { return imu_latest_raw_data.gy; }
int16_t imu_get_gz_raw(void) { return imu_latest_raw_data.gz; }

//Get the angles in Degrees
float imu_get_roll(void)  { return imu_latest_angles.roll; }
float imu_get_pitch(void) { return imu_latest_angles.pitch; }
float imu_get_yaw(void)   { return imu_latest_angles.yaw; }
