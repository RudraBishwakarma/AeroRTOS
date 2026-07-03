#include "attitude_ekf.h"
#include <stdio.h>

void attitude_ekf_init(void) {
    printf("Attitude EKF initialized\n");
}

void attitude_ekf_update(const IMUData *imu, AttitudeState *state) {
    if (imu && state) {
        state->roll = 0.0f;
        state->pitch = 0.0f;
        state->yaw = 0.0f;
        state->roll_rate = imu->gyro_x;
        state->pitch_rate = imu->gyro_y;
        state->yaw_rate = imu->gyro_z;
    }
}
