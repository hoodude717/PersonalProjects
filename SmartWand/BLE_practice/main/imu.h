#ifndef IMU_H
#define IMU_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Structure to hold raw IMU data
typedef struct {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
} imu_raw_data_t;
//Structure to hold the scaled IMU data
typedef struct {
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
} imu_data_t;

// Structure to hold computed angles (e.g., from a complementary filter)
typedef struct {
    float roll;
    float pitch;
    float yaw;
} imu_angles_t;

/**
 * @brief Initialize the IMU sensor
 *
 * @return 0 on success, non-zero on error
 */
int imu_init(void);

/**
 * @brief tick function to continuously call and update the data
 */
void imu_tick(void);

/**
 * @brief Retrieve raw accelerometer and gyroscope data from the IMU
 *
 * @param[out] data Pointer to an imu_raw_data_t structure to hold the result
 * @return 0 on success, non-zero on error
 */
int imu_get_sensor_data(uint8_t *data);

/**
 * @brief Calculate roll, pitch, and yaw angles from raw data
 *
 * @param[in] data Raw accelerometer/gyroscope data
 * @param[out] angles Output angles in degrees
 */
void imu_calculate_angles();

void imu_get_gyro_bias(void);

void imu_reset_angles();

// Accessors for individual axes (latest read)
float imu_get_ax(void);
float imu_get_ay(void);
float imu_get_az(void);
float imu_get_gx(void);
float imu_get_gy(void);
float imu_get_gz(void);
int16_t imu_get_ax_raw(void);
int16_t imu_get_ay_raw(void);
int16_t imu_get_az_raw(void);
int16_t imu_get_gx_raw(void);
int16_t imu_get_gy_raw(void);
int16_t imu_get_gz_raw(void);
// Optional: Access latest calculated angles
float imu_get_roll(void);
float imu_get_pitch(void);
float imu_get_yaw(void);

#ifdef __cplusplus
}
#endif

#endif // IMU_H
