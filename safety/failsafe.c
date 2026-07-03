#include "failsafe.h"
#include <stdio.h>

void failsafe_init(SafetyConfig *config) {
    if (config) {
        config->timeout_ms = 1000;
    }
    printf("Failsafe initialized\n");
}

FailsafeStatus failsafe_check_health(void) {
    return FAILSAFE_OK;
}

void failsafe_activate(void) {
    printf("Failsafe activated!\n");
}
