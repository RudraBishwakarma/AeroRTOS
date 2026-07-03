#ifndef POSITION_ESTIMATOR_H
#define POSITION_ESTIMATOR_H

#include "sensors/gps_sensor.h"

typedef struct {
    float x;
    float y;
    float z;
    float vx;
    float vy;
    float vz;
} PositionState;

void position_estimator_init(void);
void position_estimator_update(const GPSData *gps, PositionState *state);

#endif /* POSITION_ESTIMATOR_H */
