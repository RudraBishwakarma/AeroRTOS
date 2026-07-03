#include "logger.h"
#include <stdio.h>

void logger_init(void) {
    printf("Logger initialized\n");
}

void logger_info(const char *msg) {
    printf("[INFO] %s\n", msg ? msg : "");
}
