#include "imu_sensor.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

static struct {
    uint32_t sample_rate;
    float bias_accel[3];
    float bias_gyro[3];
    float scale_accel[3];
    float scale_gyro[3];
    uint32_t last_sample_time;
    bool initialized;
} g_imu = {
    .bias_accel = {0, 0, 0},
    .bias_gyro = {0, 0, 0},
    .scale_accel = {1.0f, 1.0f, 1.0f},
    .scale_gyro = {1.0f, 1.0f, 1.0f},
    .initialized = false
};

void imu_sensor_init(uint32_t sample_rate_hz) {
    g_imu.sample_rate = sample_rate_hz;
    g_imu.initialized = true;
    g_imu.last_sample_time = 0;
    printf("IMU Sensor initialized at %u Hz\n", sample_rate_hz);
}

void imu_sensor_read(IMUData *data) {
    if (!data || !g_imu.initialized) {
        return;
    }
    
    /* Simulate IMU data - In real system, this would read from hardware */
    static float time = 0;
    time += 0.001f;
    
    /* Generate realistic IMU data with small variations */
    data->accel_x = (rand() % 100 - 50) * 0.001f; /* Small noise */
    data->accel_y = (rand() % 100 - 50) * 0.001f;
    data->accel_z = -9.81f + (rand() % 100 - 50) * 0.001f; /* Gravity + noise */
    
    data->gyro_x = sin(time * 2.0f) * 0.01f + (rand() % 100 - 50) * 0.0001f;
    data->gyro_y = cos(time * 1.5f) * 0.01f + (rand() % 100 - 50) * 0.0001f;
    data->gyro_z = sin(time * 0.8f) * 0.005f + (rand() % 100 - 50) * 0.0001f;
    
    data->temp = 25.0f + (rand() % 100 - 50) * 0.01f;
    data->timestamp = (uint32_t)(time * 1000);
    data->valid = true;
}

void imu_sensor_set_bias(float ax, float ay, float az, float gx, float gy, float gz) {
    g_imu.bias_accel[0] = ax;
    g_imu.bias_accel[1] = ay;
    g_imu.bias_accel[2] = az;
    g_imu.bias_gyro[0] = gx;
    g_imu.bias_gyro[1] = gy;
    g_imu.bias_gyro[2] = gz;
}

void imu_sensor_calibrate(void) {
    /* Calibration routine - would collect data and compute biases */
    printf("IMU Calibration started...\n");
    /* Simulate calibration */
    imu_sensor_set_bias(0.01f, -0.02f, 0.03f, 0.001f, -0.002f, 0.001f);
    printf("IMU Calibration complete\n");
}