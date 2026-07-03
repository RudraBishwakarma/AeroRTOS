#include "rtos_kernel.h"
#include <string.h>

/* External functions from kernel */
extern WaitingTaskNode* allocate_waiting_node(void);
extern void free_waiting_node(WaitingTaskNode *node);
extern void kernel_add_to_ready_queue(TaskControlBlock *task);
extern void kernel_schedule(void);

static void queue_add_waiting_task(WaitingTaskNode **list, TaskControlBlock *task) {
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

static TaskControlBlock* queue_remove_waiting_task(WaitingTaskNode **list) {
    if (*list == NULL) return NULL;
    
    WaitingTaskNode *node = *list;
    TaskControlBlock *task = node->task;
    *list = node->next;
    free_waiting_node(node);
    return task;
}

KernelError queue_create(const char *name, uint32_t item_size, 
                         uint32_t max_items, bool isr_safe, uint32_t *queue_id) {
    critical_enter();
    
    if (!g_kernel.kernel_initialized) {
        critical_exit();
        return KERNEL_ERROR_RESOURCE_UNAVAILABLE;
    }
    
    if (item_size == 0 || max_items == 0) {
        critical_exit();
        return KERNEL_ERROR_INVALID_PARAM;
    }
    
    if (item_size * max_items > AERORTOS_CONFIG_STATIC_BUFFER_SIZE) {
        critical_exit();
        return KERNEL_ERROR_OUT_OF_MEMORY;
    }
    
    QueueControlBlock *queue = NULL;
    for (int i = 0; i < AERORTOS_CONFIG_MAX_QUEUES; i++) {
        if (g_kernel.queues[i].queue_id == 0) {
            queue = &g_kernel.queues[i];
            break;
        }
    }
    
    if (!queue) {
        critical_exit();
        return KERNEL_ERROR_OUT_OF_MEMORY;
    }
    
    memset(queue, 0, sizeof(QueueControlBlock));
    queue->queue_id = ++g_kernel.queue_counter;
    strncpy(queue->name, name, sizeof(queue->name) - 1);
    queue->name[sizeof(queue->name) - 1] = '\0';
    queue->item_size = item_size;
    queue->max_items = max_items;
    queue->item_count = 0;
    queue->head = 0;
    queue->tail = 0;
    queue->waiting_send_list = NULL;
    queue->waiting_recv_list = NULL;
    queue->is_isr_safe = isr_safe;
    
    if (queue_id) {
        *queue_id = queue->queue_id;
    }
    
    critical_exit();
    return KERNEL_SUCCESS;
}

KernelError queue_delete(uint32_t queue_id) {
    critical_enter();
    
    QueueControlBlock *queue = NULL;
    for (int i = 0; i < AERORTOS_CONFIG_MAX_QUEUES; i++) {
        if (g_kernel.queues[i].queue_id == queue_id) {
            queue = &g_kernel.queues[i];
            break;
        }
    }
    
    if (!queue) {
        critical_exit();
        return KERNEL_ERROR_RESOURCE_UNAVAILABLE;
    }
    
    while (queue->waiting_send_list) {
        WaitingTaskNode *node = queue->waiting_send_list;
        queue->waiting_send_list = node->next;
        free_waiting_node(node);
    }
    while (queue->waiting_recv_list) {
        WaitingTaskNode *node = queue->waiting_recv_list;
        queue->waiting_recv_list = node->next;
        free_waiting_node(node);
    }
    
    memset(queue, 0, sizeof(QueueControlBlock));
    
    critical_exit();
    return KERNEL_SUCCESS;
}

KernelError queue_send(uint32_t queue_id, const void *item, uint32_t timeout_ms) {
    if (g_kernel.isr_nesting_depth > 0) {
        return KERNEL_ERROR_ISR_CONTEXT;
    }
    
    critical_enter();
    
    QueueControlBlock *queue = NULL;
    for (int i = 0; i < AERORTOS_CONFIG_MAX_QUEUES; i++) {
        if (g_kernel.queues[i].queue_id == queue_id) {
            queue = &g_kernel.queues[i];
            break;
        }
    }
    
    if (!queue || !item) {
        critical_exit();
        return KERNEL_ERROR_INVALID_PARAM;
    }
    
    if (queue->item_count >= queue->max_items) {
        if (timeout_ms == 0) {
            critical_exit();
            return KERNEL_ERROR_QUEUE_FULL;
        }
        
        TaskControlBlock *task = g_kernel.current_task;
        if (!task) {
            critical_exit();
            return KERNEL_ERROR_RESOURCE_UNAVAILABLE;
        }
        
        task->state = TASK_STATE_BLOCKED;
        task->waiting_resource = queue;
        task->wake_time = g_kernel.system_tick + timeout_ms;
        
        queue_add_waiting_task(&queue->waiting_send_list, task);
        
        critical_exit();
        kernel_schedule();
        critical_enter();
        
        if (queue->item_count >= queue->max_items) {
            critical_exit();
            return KERNEL_ERROR_TIMEOUT;
        }
    }
    
    uint8_t *dest = queue->buffer + (queue->tail * queue->item_size);
    memcpy(dest, item, queue->item_size);
    queue->tail = (queue->tail + 1) % queue->max_items;
    queue->item_count++;
    
    if (queue->waiting_recv_list) {
        TaskControlBlock *wake_task = queue_remove_waiting_task(&queue->waiting_recv_list);
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

KernelError queue_receive(uint32_t queue_id, void *item, uint32_t timeout_ms) {
    if (g_kernel.isr_nesting_depth > 0) {
        return KERNEL_ERROR_ISR_CONTEXT;
    }
    
    critical_enter();
    
    QueueControlBlock *queue = NULL;
    for (int i = 0; i < AERORTOS_CONFIG_MAX_QUEUES; i++) {
        if (g_kernel.queues[i].queue_id == queue_id) {
            queue = &g_kernel.queues[i];
            break;
        }
    }
    
    if (!queue || !item) {
        critical_exit();
        return KERNEL_ERROR_INVALID_PARAM;
    }
    
    if (queue->item_count == 0) {
        if (timeout_ms == 0) {
            critical_exit();
            return KERNEL_ERROR_QUEUE_EMPTY;
        }
        
        TaskControlBlock *task = g_kernel.current_task;
        if (!task) {
            critical_exit();
            return KERNEL_ERROR_RESOURCE_UNAVAILABLE;
        }
        
        task->state = TASK_STATE_BLOCKED;
        task->waiting_resource = queue;
        task->wake_time = g_kernel.system_tick + timeout_ms;
        
        queue_add_waiting_task(&queue->waiting_recv_list, task);
        
        critical_exit();
        kernel_schedule();
        critical_enter();
        
        if (queue->item_count == 0) {
            critical_exit();
            return KERNEL_ERROR_TIMEOUT;
        }
    }
    
    uint8_t *src = queue->buffer + (queue->head * queue->item_size);
    memcpy(item, src, queue->item_size);
    queue->head = (queue->head + 1) % queue->max_items;
    queue->item_count--;
    
    if (queue->waiting_send_list) {
        TaskControlBlock *wake_task = queue_remove_waiting_task(&queue->waiting_send_list);
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

uint32_t queue_get_count(uint32_t queue_id) {
    critical_enter();
    
    for (int i = 0; i < AERORTOS_CONFIG_MAX_QUEUES; i++) {
        if (g_kernel.queues[i].queue_id == queue_id) {
            uint32_t count = g_kernel.queues[i].item_count;
            critical_exit();
            return count;
        }
    }
    
    critical_exit();
    return 0;
}

uint32_t queue_get_space(uint32_t queue_id) {
    critical_enter();
    
    for (int i = 0; i < AERORTOS_CONFIG_MAX_QUEUES; i++) {
        if (g_kernel.queues[i].queue_id == queue_id) {
            uint32_t space = g_kernel.queues[i].max_items - g_kernel.queues[i].item_count;
            critical_exit();
            return space;
        }
    }
    
    critical_exit();
    return 0;
}