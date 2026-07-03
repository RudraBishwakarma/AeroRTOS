#include "position_estimator.h"
#include <stdio.h>

void position_estimator_init(void) {
    printf("Position Estimator initialized\n");
}

void position_estimator_update(const GPSData *gps, PositionState *state) {
    if (gps && state) {
        state->x = 0.0f;
        state->y = 0.0f;
        state->z = gps->altitude;
        state->vx = gps->velocity_north;
        state->vy = gps->velocity_east;
        state->vz = gps->velocity_down;
    }
}
