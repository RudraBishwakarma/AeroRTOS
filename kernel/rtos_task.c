#include "rtos_task.h"
#include "rtos_kernel.h"
#include <string.h>
#include <stdio.h>

extern KernelState g_kernel;
/* Task Notification Support */
typedef struct TaskNotification {
    uint32_t value;
    bool pending;
    TaskControlBlock *waiting_task;
} TaskNotification;

static TaskNotification g_task_notifications[AERORTOS_CONFIG_MAX_TASKS];

/* Task Name Setting */
void task_set_name(uint32_t task_id, const char *name) {
    TaskControlBlock *task = NULL;
    for (int i = 0; i < AERORTOS_CONFIG_MAX_TASKS; i++) {
        if (g_kernel.task_pool[i].task_id == task_id) {
            task = &g_kernel.task_pool[i];
            break;
        }
    }
    
    if (task) {
        strncpy(task->name, name, sizeof(task->name) - 1);
        task->name[sizeof(task->name) - 1] = '\0';
    }
}



/* Task Statistics */
KernelError task_get_statistics(uint32_t task_id, TaskStatistics *stats) {
    if (!stats) {
        return KERNEL_ERROR_INVALID_PARAM;
    }
    
    TaskControlBlock *task = NULL;
    for (int i = 0; i < AERORTOS_CONFIG_MAX_TASKS; i++) {
        if (g_kernel.task_pool[i].task_id == task_id) {
            task = &g_kernel.task_pool[i];
            break;
        }
    }
    
    if (!task) {
        return KERNEL_ERROR_TASK_NOT_FOUND;
    }
    
    stats->task_id = task->task_id;
    strncpy(stats->name, task->name, sizeof(stats->name) - 1);
    stats->name[sizeof(stats->name) - 1] = '\0';
    stats->state = task->state;
    stats->priority = task->priority;
    stats->execution_time = task->execution_time;
    stats->deadline_misses = task->deadline_misses;
    stats->stack_peak_usage = task_get_stack_usage(task_id);
    stats->context_switches = 0; // TODO: Track per-task context switches
    
    return KERNEL_SUCCESS;
}

/* Task Notifications */
KernelError task_notify(uint32_t task_id, uint32_t value) {
    if (task_id >= AERORTOS_CONFIG_MAX_TASKS) {
        return KERNEL_ERROR_INVALID_PARAM;
    }
    
    TaskNotification *notif = &g_task_notifications[task_id];
    notif->value = value;
    notif->pending = true;
    
    /* If task is waiting for notification, wake it up */
    if (notif->waiting_task) {
        TaskControlBlock *task = notif->waiting_task;
        /* Remove from blocked list */
        if (task->prev) {
            task->prev->next = task->next;
        } else {
            g_kernel.blocked_list = task->next;
        }
        if (task->next) {
            task->next->prev = task->prev;
        }
        task->state = TASK_STATE_READY;
        task->wake_time = 0;
        /* Add to ready queue */
        task->next = g_kernel.ready_queue[task->priority];
        if (g_kernel.ready_queue[task->priority]) {
            g_kernel.ready_queue[task->priority]->prev = task;
        }
        g_kernel.ready_queue[task->priority] = task;
        notif->waiting_task = NULL;
    }
    
    return KERNEL_SUCCESS;
}

KernelError task_wait_notify(uint32_t timeout_ms, uint32_t *value) {
    if (g_kernel.isr_nesting_depth > 0) {
        return KERNEL_ERROR_ISR_CONTEXT;
    }
    
    TaskControlBlock *task = g_kernel.current_task;
    if (!task) {
        return KERNEL_ERROR_TASK_NOT_FOUND;
    }
    
    uint32_t task_index = task->task_id - 1;
    if (task_index >= AERORTOS_CONFIG_MAX_TASKS) {
        return KERNEL_ERROR_INVALID_PARAM;
    }
    
    TaskNotification *notif = &g_task_notifications[task_index];
    
    /* Check if notification already pending */
    if (notif->pending) {
        if (value) {
            *value = notif->value;
        }
        notif->pending = false;
        return KERNEL_SUCCESS;
    }
    
    /* Wait for notification */
    notif->waiting_task = task;
    task->state = TASK_STATE_BLOCKED;
    task->wake_time = g_kernel.system_tick + timeout_ms;
    
    /* Add to blocked list */
    task->next = g_kernel.blocked_list;
    if (g_kernel.blocked_list) {
        g_kernel.blocked_list->prev = task;
    }
    g_kernel.blocked_list = task;
    
    /* Schedule next task */
    kernel_schedule();
    
    /* Check if notification was received */
    if (notif->pending) {
        if (value) {
            *value = notif->value;
        }
        notif->pending = false;
        return KERNEL_SUCCESS;
    }
    
    return KERNEL_ERROR_TIMEOUT;
}

void task_clear_notify(uint32_t task_id) {
    if (task_id >= AERORTOS_CONFIG_MAX_TASKS) {
        return;
    }
    g_task_notifications[task_id].pending = false;
}

/* Print Task Statistics */
void task_print_statistics(void) {
    printf("\n=== Task Statistics ===\n");
    printf("%-8s %-20s %-12s %-8s %-12s %-10s\n", 
           "ID", "Name", "State", "Priority", "Stack Usage", "Deadline Misses");
    printf("--------------------------------------------------------\n");
    
    for (int i = 0; i < AERORTOS_CONFIG_MAX_TASKS; i++) {
        if (g_kernel.task_pool[i].state != TASK_STATE_TERMINATED && 
            g_kernel.task_pool[i].task_id != 0) {
            TaskStatistics stats;
            if (task_get_statistics(g_kernel.task_pool[i].task_id, &stats) == KERNEL_SUCCESS) {
                printf("%-8u %-20s %-12d %-8d %-12u %-10u\n",
                       stats.task_id,
                       stats.name,
                       stats.state,
                       stats.priority,
                       stats.stack_peak_usage,
                       stats.deadline_misses);
            }
        }
    }
}
