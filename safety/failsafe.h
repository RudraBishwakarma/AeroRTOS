#ifndef FAILSAFE_H
#define FAILSAFE_H

typedef enum {
    FAILSAFE_OK = 0,
    FAILSAFE_ERROR = 1
} FailsafeStatus;

typedef struct {
    int timeout_ms;
} SafetyConfig;

void failsafe_init(SafetyConfig *config);
FailsafeStatus failsafe_check_health(void);
void failsafe_activate(void);

#endif /* FAILSAFE_H */
