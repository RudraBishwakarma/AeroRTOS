#ifndef DRONE_MODEL_H
#define DRONE_MODEL_H

#include "flight_controller/attitude_controller.h"

typedef struct {
    float x, y, z;
    float roll, pitch, yaw;
} DroneState;

void drone_model_init(DroneState *state);
void drone_model_update(DroneState *state, const MotorOutputs *outputs, double dt);

#endif /* DRONE_MODEL_H */
