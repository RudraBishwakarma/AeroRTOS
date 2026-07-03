#include "rtos_kernel.h"
#include <string.h>

/* External functions from kernel */
extern WaitingTaskNode* allocate_waiting_node(void);
extern void free_waiting_node(WaitingTaskNode *node);
extern void kernel_add_to_ready_queue(TaskControlBlock *task);
extern void kernel_schedule(void);

static void sem_add_waiting_task(WaitingTaskNode **list, TaskControlBlock *task) {
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

static TaskControlBlock* sem_remove_waiting_task(WaitingTaskNode **list) {
    if (*list == NULL) return NULL;
    
    WaitingTaskNode *node = *list;
    TaskControlBlock *task = node->task;
    *list = node->next;
    free_waiting_node(node);
    return task;
}

KernelError semaphore_create(const char *name, uint32_t initial_count, 
                            uint32_t max_count, bool binary, uint32_t *sem_id) {
    critical_enter();
    
    if (!g_kernel.kernel_initialized) {
        critical_exit();
        return KERNEL_ERROR_RESOURCE_UNAVAILABLE;
    }
    
    if (initial_count > max_count) {
        critical_exit();
        return KERNEL_ERROR_INVALID_PARAM;
    }
    
    SemaphoreControlBlock *sem = NULL;
    for (int i = 0; i < AERORTOS_CONFIG_MAX_SEMAPHORES; i++) {
        if (g_kernel.semaphores[i].sem_id == 0) {
            sem = &g_kernel.semaphores[i];
            break;
        }
    }
    
    if (!sem) {
        critical_exit();
        return KERNEL_ERROR_OUT_OF_MEMORY;
    }
    
    memset(sem, 0, sizeof(SemaphoreControlBlock));
    sem->sem_id = ++g_kernel.semaphore_counter;
    strncpy(sem->name, name, sizeof(sem->name) - 1);
    sem->name[sizeof(sem->name) - 1] = '\0';
    sem->count = initial_count;
    sem->max_count = max_count;
    sem->waiting_list = NULL;
    sem->is_binary = binary;
    
    if (sem_id) {
        *sem_id = sem->sem_id;
    }
    
    critical_exit();
    return KERNEL_SUCCESS;
}

KernelError semaphore_delete(uint32_t sem_id) {
    critical_enter();
    
    SemaphoreControlBlock *sem = NULL;
    for (int i = 0; i < AERORTOS_CONFIG_MAX_SEMAPHORES; i++) {
        if (g_kernel.semaphores[i].sem_id == sem_id) {
            sem = &g_kernel.semaphores[i];
            break;
        }
    }
    
    if (!sem) {
        critical_exit();
        return KERNEL_ERROR_RESOURCE_UNAVAILABLE;
    }
    
    while (sem->waiting_list) {
        WaitingTaskNode *node = sem->waiting_list;
        sem->waiting_list = node->next;
        free_waiting_node(node);
    }
    
    memset(sem, 0, sizeof(SemaphoreControlBlock));
    
    critical_exit();
    return KERNEL_SUCCESS;
}

KernelError semaphore_take(uint32_t sem_id, uint32_t timeout_ms) {
    if (g_kernel.isr_nesting_depth > 0) {
        return KERNEL_ERROR_ISR_CONTEXT;
    }
    
    critical_enter();
    
    SemaphoreControlBlock *sem = NULL;
    for (int i = 0; i < AERORTOS_CONFIG_MAX_SEMAPHORES; i++) {
        if (g_kernel.semaphores[i].sem_id == sem_id) {
            sem = &g_kernel.semaphores[i];
            break;
        }
    }
    
    if (!sem) {
        critical_exit();
        return KERNEL_ERROR_RESOURCE_UNAVAILABLE;
    }
    
    if (sem->count > 0) {
        sem->count--;
        critical_exit();
        return KERNEL_SUCCESS;
    }
    
    if (timeout_ms == 0) {
        critical_exit();
        return KERNEL_ERROR_TIMEOUT;
    }
    
    TaskControlBlock *task = g_kernel.current_task;
    if (!task) {
        critical_exit();
        return KERNEL_ERROR_RESOURCE_UNAVAILABLE;
    }
    
    task->state = TASK_STATE_BLOCKED;
    task->waiting_resource = sem;
    task->wake_time = g_kernel.system_tick + timeout_ms;
    
    sem_add_waiting_task(&sem->waiting_list, task);
    
    critical_exit();
    kernel_schedule();
    critical_enter();
    
    if (sem->count > 0) {
        sem->count--;
        critical_exit();
        return KERNEL_SUCCESS;
    }
    
    critical_exit();
    return KERNEL_ERROR_TIMEOUT;
}

KernelError semaphore_give(uint32_t sem_id) {
    critical_enter();
    
    SemaphoreControlBlock *sem = NULL;
    for (int i = 0; i < AERORTOS_CONFIG_MAX_SEMAPHORES; i++) {
        if (g_kernel.semaphores[i].sem_id == sem_id) {
            sem = &g_kernel.semaphores[i];
            break;
        }
    }
    
    if (!sem) {
        critical_exit();
        return KERNEL_ERROR_RESOURCE_UNAVAILABLE;
    }
    
    if (sem->count >= sem->max_count) {
        critical_exit();
        return KERNEL_ERROR_RESOURCE_ALREADY_TAKEN;
    }
    
    sem->count++;
    
    if (sem->waiting_list) {
        TaskControlBlock *wake_task = sem_remove_waiting_task(&sem->waiting_list);
        if (wake_task) {
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
    }
    
    critical_exit();
    return KERNEL_SUCCESS;
}

uint32_t semaphore_get_count(uint32_t sem_id) {
    critical_enter();
    
    for (int i = 0; i < AERORTOS_CONFIG_MAX_SEMAPHORES; i++) {
        if (g_kernel.semaphores[i].sem_id == sem_id) {
            uint32_t count = g_kernel.semaphores[i].count;
            critical_exit();
            return count;
        }
    }
    
    critical_exit();
    return 0;
}