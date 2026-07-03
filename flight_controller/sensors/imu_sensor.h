#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float accel_x;      /* m/s² */
    float accel_y;
    float accel_z;
    float gyro_x;       /* rad/s */
    float gyro_y;
    float gyro_z;
    float temp;         /* °C */
    uint32_t timestamp; /* ms */
    bool valid;
} IMUData;

void imu_sensor_init(uint32_t sample_rate_hz);
void imu_sensor_read(IMUData *data);
void imu_sensor_set_bias(float ax, float ay, float az, float gx, float gy, float gz);
void imu_sensor_calibrate(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_SENSOR_H */