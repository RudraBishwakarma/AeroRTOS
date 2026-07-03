#ifndef ATTITUDE_CONTROLLER_H
#define ATTITUDE_CONTROLLER_H

#include "estimation/attitude_ekf.h"

typedef enum {
    CONTROL_MODE_ATTITUDE,
    CONTROL_MODE_RATE,
    CONTROL_MODE_ACRO
} ControlMode;

typedef struct {
    float throttle;
    float roll;
    float pitch;
    float yaw;
    ControlMode mode;
} ControlCommand;

typedef struct {
    float motor1;
    float motor2;
    float motor3;
    float motor4;
} MotorOutputs;

void attitude_controller_init(void);
void attitude_controller_update(const AttitudeState *attitude, const ControlCommand *cmd, MotorOutputs *outputs);

#endif /* ATTITUDE_CONTROLLER_H */
