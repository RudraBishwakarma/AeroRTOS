#ifndef APP_STATE_H
#define APP_STATE_H

#include <stdbool.h>
#include "estimation/ekf.h"

extern volatile bool g_system_running;
extern EKFState g_ekf_state;

#endif /* APP_STATE_H */