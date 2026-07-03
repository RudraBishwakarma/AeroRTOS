#ifndef POSITION_CONTROLLER_H
#define POSITION_CONTROLLER_H

#include "estimation/position_estimator.h"
#include "attitude_controller.h"

void position_controller_init(void);
void position_controller_update(const PositionState *position, ControlCommand *cmd);

#endif /* POSITION_CONTROLLER_H */
