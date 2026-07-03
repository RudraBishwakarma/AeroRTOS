#include "rtos_kernel.h"
#include <string.h>

/* External functions from kernel */
extern WaitingTaskNode* allocate_waiting_node(void);
extern void free_waiting_node(WaitingTaskNode *node);
extern void kernel_add_to_ready_queue(TaskControlBlock *task);
extern void kernel_schedule(void);

static void mutex_add_waiting_task(WaitingTaskNode **list, TaskControlBlock *task) {
    WaitingTaskNode *node = allocate_waiting_node();
    if (!node) return;
    
    node->task = task;
    node->next = NULL;
    
    if (*list == NULL) {
        *list = node;
    } else {
        WaitingTaskNode *current = *list;
        while (current->next) {
            current = current->next;
        }
        current->next = node;
    }
}

static TaskControlBlock* mutex_remove_waiting_task(WaitingTaskNode **list) {
    if (*list == NULL) return NULL;
    
    WaitingTaskNode *node = *list;
    TaskControlBlock *task = node->task;
    *list = node->next;
    free_waiting_node(node);
    return task;
}

KernelError mutex_create(const char *name, uint32_t *mutex_id) {
    critical_enter();
    
    if (!g_kernel.kernel_initialized) {
        critical_exit();
        return KERNEL_ERROR_RESOURCE_UNAVAILABLE;
    }
    
    MutexControlBlock *mutex = NULL;
    for (int i = 0; i < AERORTOS_CONFIG_MAX_MUTEXES; i++) {
        if (g_kernel.mutexes[i].mutex_id == 0) {
            mutex = &g_kernel.mutexes[i];
            break;
        }
    }
    
    if (!mutex) {
        critical_exit();
        return KERNEL_ERROR_OUT_OF_MEMORY;
    }
    
    memset(mutex, 0, sizeof(MutexControlBlock));
    mutex->mutex_id = ++g_kernel.mutex_counter;
    strncpy(mutex->name, name, sizeof(mutex->name) - 1);
    mutex->name[sizeof(mutex->name) - 1] = '\0';
    mutex->is_locked = false;
    mutex->owner = NULL;
    mutex->original_priority = TASK_PRIORITY_IDLE;
    mutex->waiting_list = NULL;
    mutex->lock_count = 0;
    
    if (mutex_id) {
        *mutex_id = mutex->mutex_id;
    }
    
    critical_exit();
    return KERNEL_SUCCESS;
}

KernelError mutex_delete(uint32_t mutex_id) {
    critical_enter();
    
    MutexControlBlock *mutex = NULL;
    for (int i = 0; i < AERORTOS_CONFIG_MAX_MUTEXES; i++) {
        if (g_kernel.mutexes[i].mutex_id == mutex_id) {
            mutex = &g_kernel.mutexes[i];
            break;
        }
    }
    
    if (!mutex) {
        critical_exit();
        return KERNEL_ERROR_RESOURCE_UNAVAILABLE;
    }
    
    while (mutex->waiting_list) {
        WaitingTaskNode *node = mutex->waiting_list;
        mutex->waiting_list = node->next;
        free_waiting_node(node);
    }
    
    memset(mutex, 0, sizeof(MutexControlBlock));
    
    critical_exit();
    return KERNEL_SUCCESS;
}

KernelError mutex_lock(uint32_t mutex_id, uint32_t timeout_ms) {
    if (g_kernel.isr_nesting_depth > 0) {
        return KERNEL_ERROR_ISR_CONTEXT;
    }
    
    critical_enter();
    
    MutexControlBlock *mutex = NULL;
    for (int i = 0; i < AERORTOS_CONFIG_MAX_MUTEXES; i++) {
        if (g_kernel.mutexes[i].mutex_id == mutex_id) {
            mutex = &g_kernel.mutexes[i];
            break;
        }
    }
    
    if (!mutex) {
        critical_exit();
        return KERNEL_ERROR_RESOURCE_UNAVAILABLE;
    }
    
    TaskControlBlock *task = g_kernel.current_task;
    if (!task) {
        critical_exit();
        return KERNEL_ERROR_RESOURCE_UNAVAILABLE;
    }
    
    /* If already owned by current task, recursive lock */
    if (mutex->is_locked && mutex->owner == task) {
        mutex->lock_count++;
        critical_exit();
        return KERNEL_SUCCESS;
    }
    
    if (!mutex->is_locked) {
        mutex->is_locked = true;
        mutex->owner = task;
        mutex->lock_count = 1;
        mutex->original_priority = task->priority;
        critical_exit();
        return KERNEL_SUCCESS;
    }
    
    if (timeout_ms == 0) {
        critical_exit();
        return KERNEL_ERROR_TIMEOUT;
    }
    
    /* Priority Inheritance: Boost owner's priority */
    if (task->priority > mutex->owner->priority) {
        mutex->owner->priority = task->priority;
    }
    
    task->state = TASK_STATE_BLOCKED;
    task->waiting_resource = mutex;
    task->wake_time = g_kernel.system_tick + timeout_ms;
    
    mutex_add_waiting_task(&mutex->waiting_list, task);
    
    critical_exit();
    kernel_schedule();
    critical_enter();
    
    /* Check if we got the mutex */
    if (mutex->owner == task) {
        critical_exit();
        return KERNEL_SUCCESS;
    }
    
    critical_exit();
    return KERNEL_ERROR_TIMEOUT;
}

KernelError mutex_unlock(uint32_t mutex_id) {
    critical_enter();
    
    MutexControlBlock *mutex = NULL;
    for (int i = 0; i < AERORTOS_CONFIG_MAX_MUTEXES; i++) {
        if (g_kernel.mutexes[i].mutex_id == mutex_id) {
            mutex = &g_kernel.mutexes[i];
            break;
        }
    }
    
    if (!mutex) {
        critical_exit();
        return KERNEL_ERROR_RESOURCE_UNAVAILABLE;
    }
    
    TaskControlBlock *task = g_kernel.current_task;
    if (!task || mutex->owner != task) {
        critical_exit();
        return KERNEL_ERROR_INVALID_PARAM;
    }
    
    mutex->lock_count--;
    
    if (mutex->lock_count > 0) {
        critical_exit();
        return KERNEL_SUCCESS;
    }
    
    /* Restore original priority */
    task->priority = mutex->original_priority;
    
    if (mutex->waiting_list) {
        TaskControlBlock *wake_task = mutex_remove_waiting_task(&mutex->waiting_list);
        if (wake_task) {
            mutex->owner = wake_task;
            mutex->lock_count = 1;
            mutex->original_priority = wake_task->priority;
            
            if (wake_task->prev) {
                wake_task->prev->next = wake_task->next;
            } else {
                g_kernel.blocked_list = wake_task->next;
            }
            if (wake_task->next) {
                wake_task->next->prev = wake_task->prev;
            }
            
            wake_task->state = TASK_STATE_READY;
            wake_task->wake_time = 0;
            wake_task->waiting_resource = NULL;
            wake_task->next = NULL;
            wake_task->prev = NULL;
            
            kernel_add_to_ready_queue(wake_task);
        }
    } else {
        mutex->is_locked = false;
        mutex->owner = NULL;
    }
    
    critical_exit();
    return KERNEL_SUCCESS;
}

bool mutex_is_owned(uint32_t mutex_id) {
    critical_enter();
    
    for (int i = 0; i < AERORTOS_CONFIG_MAX_MUTEXES; i++) {
        if (g_kernel.mutexes[i].mutex_id == mutex_id) {
            bool owned = g_kernel.mutexes[i].is_locked;
            critical_exit();
            return owned;
        }
    }
    
    critical_exit();
    return false;
}