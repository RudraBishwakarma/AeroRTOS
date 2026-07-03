#include "attitude_controller.h"
#include <stdio.h>

void attitude_controller_init(void) {
    printf("Attitude Controller initialized\n");
}

void attitude_controller_update(const AttitudeState *attitude, const ControlCommand *cmd, MotorOutputs *outputs) {
    if (attitude && cmd && outputs) {
        outputs->motor1 = cmd->throttle;
        outputs->motor2 = cmd->throttle;
        outputs->motor3 = cmd->throttle;
        outputs->motor4 = cmd->throttle;
    }
}
