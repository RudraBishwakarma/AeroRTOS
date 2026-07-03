#ifndef ATTITUDE_EKF_H
#define ATTITUDE_EKF_H

#include "sensors/imu_sensor.h"

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float roll_rate;
    float pitch_rate;
    float yaw_rate;
} AttitudeState;

void attitude_ekf_init(void);
void attitude_ekf_update(const IMUData *imu, AttitudeState *state);

#endif /* ATTITUDE_EKF_H */
