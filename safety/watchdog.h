#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Watchdog Timer Configuration
 * ============================================================================ */

#define WATCHDOG_MAX_TASKS 32
#define WATCHDOG_DEFAULT_TIMEOUT_MS 1000
#define WATCHDOG_CRITICAL_TIMEOUT_MS 500

/* Watchdog States */
typedef enum {
    WATCHDOG_STATE_OK = 0,
    WATCHDOG_STATE_WARNING,
    WATCHDOG_STATE_CRITICAL,
    WATCHDOG_STATE_TIMEOUT
} WatchdogState;

/* Watchdog Task Entry */
typedef struct {
    uint32_t task_id;
    char task_name[32];
    uint32_t last_kick_time;
    uint32_t timeout_ms;
    uint32_t missed_kicks;
    uint32_t max_missed_kicks;
    WatchdogState state;
    bool enabled;
    bool is_critical;
    void (*recovery_callback)(uint32_t task_id);
} WatchdogTaskEntry;

/* Watchdog Context */
typedef struct {
    WatchdogTaskEntry tasks[WATCHDOG_MAX_TASKS];
    uint32_t task_count;
    uint32_t global_timeout_ms;
    bool enabled;
    uint32_t last_check_time;
    uint32_t check_interval_ms;
    WatchdogState global_state;
    uint32_t total_timeouts;
    uint32_t total_recoveries;
} WatchdogContext;

/* ============================================================================
 * Function Prototypes
 * ============================================================================ */

/* Watchdog Management */
void watchdog_init(void);
void watchdog_start(void);
void watchdog_stop(void);
void watchdog_update(void);
void watchdog_reset(void);

/* Task Registration */
bool watchdog_register_task(uint32_t task_id, const char *name, 
                            uint32_t timeout_ms, bool is_critical,
                            void (*recovery_callback)(uint32_t));
bool watchdog_unregister_task(uint32_t task_id);

/* Task Kicking */
bool watchdog_kick(uint32_t task_id);
bool watchdog_kick_current(void);

/* Status Queries */
WatchdogState watchdog_get_state(uint32_t task_id);
WatchdogState watchdog_get_global_state(void);
uint32_t watchdog_get_timeouts(void);
uint32_t watchdog_get_recoveries(void);
const char* watchdog_get_state_string(WatchdogState state);

/* Configuration */
void watchdog_set_global_timeout(uint32_t timeout_ms);
void watchdog_set_check_interval(uint32_t interval_ms);
void watchdog_set_task_timeout(uint32_t task_id, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* WATCHDOG_H */