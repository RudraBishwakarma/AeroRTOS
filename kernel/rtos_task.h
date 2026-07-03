#ifndef AERORTOS_TASK_H
#define AERORTOS_TASK_H

#include "rtos_kernel.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Task Additional Functions */
void task_set_name(uint32_t task_id, const char *name);
uint32_t task_get_stack_usage(uint32_t task_id);
void task_set_deadline_callback(uint32_t task_id, void (*callback)(uint32_t));

/* Task Statistics */
typedef struct TaskStatistics {
    uint32_t task_id;
    char name[32];
    TaskState state;
    TaskPriority priority;
    uint32_t execution_time;
    uint32_t deadline_misses;
    uint32_t stack_peak_usage;
    uint32_t context_switches;
} TaskStatistics;

KernelError task_get_statistics(uint32_t task_id, TaskStatistics *stats);
void task_print_statistics(void);

/* Task Event Notifications */
KernelError task_notify(uint32_t task_id, uint32_t value);
KernelError task_wait_notify(uint32_t timeout_ms, uint32_t *value);
void task_clear_notify(uint32_t task_id);

#ifdef __cplusplus
}
#endif

#endif /* AERORTOS_TASK_H */