#include "rtos_kernel.h"
#include "rtos_port.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <windows.h>

/* Global Kernel State */
KernelState g_kernel = {0};

/* Error Messages */
static const char* g_error_messages[] = {
    "KERNEL_SUCCESS",
    "ERROR_INVALID_PARAM",
    "ERROR_OUT_OF_MEMORY",
    "ERROR_TIMEOUT",
    "ERROR_TASK_NOT_FOUND",
    "ERROR_RESOURCE_UNAVAILABLE",
    "ERROR_RESOURCE_ALREADY_TAKEN",
    "ERROR_ISR_CONTEXT",
    "ERROR_QUEUE_FULL",
    "ERROR_QUEUE_EMPTY",
    "ERROR_TASK_SUSPENDED",
    "ERROR_TASK_TERMINATED",
    "ERROR_WAITING_NODE_EXHAUSTED",
    "ERROR_MAX"
};

static const char* task_state_strings[] = {
    "TERMINATED",
    "READY",
    "RUNNING",
    "BLOCKED",
    "SUSPENDED",
    "WAITING"
};

/* ============================================================================
 * CRITICAL SECTION - Windows CRITICAL_SECTION
 * ============================================================================ */

static bool critical_is_owned_by_current(void) {
    return g_kernel.critical_owner == (int)GetCurrentThreadId();
}

void critical_enter(void) {
    if (critical_is_owned_by_current()) {
        g_kernel.critical_depth++;
        return;
    }
    
    rtos_port_critical_enter();
    
    g_kernel.critical_owner = (int)GetCurrentThreadId();
    g_kernel.critical_depth = 1;
}

void critical_exit(void) {
    if (!critical_is_owned_by_current()) {
        return;
    }
    
    g_kernel.critical_depth--;
    if (g_kernel.critical_depth > 0) {
        return;
    }
    
    g_kernel.critical_owner = 0;
    rtos_port_critical_exit();
}

bool critical_try_enter(void) {
    if (critical_is_owned_by_current()) {
        g_kernel.critical_depth++;
        return true;
    }
    
    if (rtos_port_critical_try_enter()) {
        g_kernel.critical_owner = (int)GetCurrentThreadId();
        g_kernel.critical_depth = 1;
        return true;
    }
    return false;
}

/* ============================================================================
 * INTERNAL FUNCTION DECLARATIONS
 * ============================================================================ */

void kernel_add_to_ready_queue(TaskControlBlock *task);
static void kernel_remove_from_ready_queue(TaskControlBlock *task);
void kernel_schedule(void);
static void kernel_cleanup_task(TaskControlBlock *task);
static void kernel_update_cpu_usage(void);
static TaskControlBlock* kernel_find_task_by_id(uint32_t task_id);
/*
static WaitingTaskNode* allocate_waiting_node(void);
static void free_waiting_node(WaitingTaskNode *node);
*/
static void task_entry_wrapper(void);

/* ============================================================================
 * WAITING NODE MANAGEMENT
 * ============================================================================ */

WaitingTaskNode* allocate_waiting_node(void) {
    critical_enter();
    
    for (int i = 0; i < AERORTOS_CONFIG_MAX_WAITING_NODES; i++) {
        if (!g_kernel.waiting_node_pool[i].used) {
            g_kernel.waiting_node_pool[i].used = true;
            g_kernel.waiting_node_pool[i].next = NULL;
            g_kernel.waiting_node_pool[i].task = NULL;
            critical_exit();
            return &g_kernel.waiting_node_pool[i];
        }
    }
    
    critical_exit();
    return NULL;
}

void free_waiting_node(WaitingTaskNode *node) {
    if (node) {
        critical_enter();
        node->used = false;
        node->task = NULL;
        node->next = NULL;
        critical_exit();
    }
}

/* ============================================================================
 * READY QUEUE MANAGEMENT
 * ============================================================================ */

void kernel_add_to_ready_queue(TaskControlBlock *task) {
    if (!task || task->state == TASK_STATE_TERMINATED) {
        return;
    }
    
    if (task->in_ready_queue) {
        return;
    }
    
    if (task->state == TASK_STATE_RUNNING) {
        task->state = TASK_STATE_READY;
    }
    
    task->in_ready_queue = true;
    
    TaskControlBlock *current = g_kernel.ready_queue[task->priority];
    if (current == NULL) {
        g_kernel.ready_queue[task->priority] = task;
        task->next = NULL;
        task->prev = NULL;
    } else {
        while (current->next != NULL) {
            current = current->next;
        }
        current->next = task;
        task->prev = current;
        task->next = NULL;
    }
}

static void kernel_remove_from_ready_queue(TaskControlBlock *task) {
    if (!task || !task->in_ready_queue) {
        return;
    }
    
    task->in_ready_queue = false;
    
    if (task->prev) {
        task->prev->next = task->next;
    } else {
        g_kernel.ready_queue[task->priority] = task->next;
    }
    
    if (task->next) {
        task->next->prev = task->prev;
    }
    
    task->next = NULL;
    task->prev = NULL;
}

/* ============================================================================
 * TASK LOOKUP
 * ============================================================================ */

static TaskControlBlock* kernel_find_task_by_id(uint32_t task_id) {
    if (task_id == 0xFFFFFFFF) {
        return &g_kernel.idle_task;
    }
    
    for (int i = 0; i < AERORTOS_CONFIG_MAX_TASKS; i++) {
        if (g_kernel.task_pool[i].task_id == task_id && 
            g_kernel.task_pool[i].state != TASK_STATE_TERMINATED) {
            return &g_kernel.task_pool[i];
        }
    }
    return NULL;
}

/* ============================================================================
 * SCHEDULER - Updated for setjmp/longjmp
 * ============================================================================ */

void kernel_schedule(void) {
    if (!g_kernel.scheduler_running || g_kernel.scheduler_lock_depth > 0) {
        return;
    }
    
    /* Process any pending timer ticks */
    if (g_kernel.timer_pending) {
        g_kernel.timer_pending = 0;
        /* Call tick handler without triggering recursive scheduling */
        g_kernel.scheduler_lock_depth++;
        kernel_tick_handler();
        g_kernel.scheduler_lock_depth--;
    }
    
    critical_enter();
    
    TaskControlBlock *next_task = NULL;
    
    /* Find highest priority ready task */
    for (int i = AERORTOS_CONFIG_PRIORITY_LEVELS - 1; i >= 0; i--) {
        if (g_kernel.ready_queue[i] != NULL) {
            next_task = g_kernel.ready_queue[i];
            kernel_remove_from_ready_queue(next_task);
            break;
        }
    }
    
    /* If no task ready and current task is running, keep it */
    if (next_task == NULL) {
        if (g_kernel.current_task && 
            g_kernel.current_task->state == TASK_STATE_RUNNING) {
            critical_exit();
            return;
        }
        /* Find idle task */
        next_task = &g_kernel.idle_task;
        if (next_task->state == TASK_STATE_READY) {
            kernel_remove_from_ready_queue(next_task);
        } else {
            critical_exit();
            return;
        }
    }
    
    /* Don't switch if same task */
    if (next_task == g_kernel.current_task) {
        critical_exit();
        return;
    }
    
    TaskControlBlock *old_task = g_kernel.current_task;
    
    /* Prepare new task */
    next_task->state = TASK_STATE_RUNNING;
    g_kernel.current_task = next_task;
    g_kernel.context_switches++;
    g_kernel.total_ticks++;
    
    kernel_update_cpu_usage();
    
    /* FIXED: First run logic with setjmp/longjmp */
    if (next_task->first_run) {
        next_task->first_run = false;
        next_task->execution_time = 0;
        
        /* Set the context for first run */
        if (setjmp(next_task->context) == 0) {
            critical_exit();
            /* Execute task entry wrapper */
            task_entry_wrapper();
            return;
        }
        /* If we get here, longjmp was called to return */
        return;
    }
    
    /* Context switch using setjmp/longjmp */
    if (old_task) {
        if (old_task->state == TASK_STATE_RUNNING) {
            old_task->state = TASK_STATE_READY;
            if (old_task->task_id != 0 && old_task->state != TASK_STATE_TERMINATED) {
                kernel_add_to_ready_queue(old_task);
            }
        }
    }
    
    critical_exit();
    
    /* Save current context and jump to next task */
    if (old_task == NULL) {
        longjmp(next_task->context, 1);
    }

    if (setjmp(old_task->context) == 0) {
        longjmp(next_task->context, 1);
    }
}

/* ============================================================================
 * TASK ENTRY WRAPPER - Updated for setjmp/longjmp
 * ============================================================================ */

static void task_entry_wrapper(void) {
    TaskControlBlock *task = g_kernel.current_task;
    
    if (task && task->entry_point) {
        task->entry_point(task->parameters);
    }
    
    /* Task returned - delete it */
    if (task) {
        critical_enter();
        task_delete(task->task_id);
        critical_exit();
        kernel_schedule();
    }
    
    /* Should never reach here */
    exit(0);
}

/* ============================================================================
 * CPU USAGE UPDATE
 * ============================================================================ */

static void kernel_update_cpu_usage(void) {
    uint32_t current = g_kernel.system_tick;
    
    if (current - g_kernel.last_cpu_update >= 1000) {
        g_kernel.last_cpu_update = current;
        
        uint32_t total_ticks = 1000;
        
        for (int i = 0; i < AERORTOS_CONFIG_MAX_TASKS; i++) {
            TaskControlBlock *task = &g_kernel.task_pool[i];
            if (task->task_id != 0 && task->state != TASK_STATE_TERMINATED) {
                uint32_t exec_time = task->execution_time - task->last_execution_time;
                task->cpu_usage_percent = (exec_time * 100) / total_ticks;
                if (task->cpu_usage_percent > 100) {
                    task->cpu_usage_percent = 100;
                }
                task->last_execution_time = task->execution_time;
            }
        }
        
        if (g_kernel.idle_task.task_id != 0) {
            uint32_t idle_time = g_kernel.idle_task.execution_time - 
                                g_kernel.idle_task.last_execution_time;
            g_kernel.idle_task.cpu_usage_percent = (idle_time * 100) / total_ticks;
            if (g_kernel.idle_task.cpu_usage_percent > 100) {
                g_kernel.idle_task.cpu_usage_percent = 100;
            }
            g_kernel.idle_task.last_execution_time = g_kernel.idle_task.execution_time;
        }
    }
}

/* ============================================================================
 * IDLE TASK - Updated for Windows
 * ============================================================================ */

static void idle_task_function(void *params) {
    (void)params;
    g_kernel.idle_task.is_idle = true;
    
    while (1) {
        g_kernel.idle_ticks++;
        
        /* Process pending timer events */
        if (g_kernel.timer_pending) {
            critical_enter();
            if (g_kernel.timer_pending) {
                g_kernel.timer_pending = 0;
                kernel_tick_handler();
            }
            critical_exit();
        }
        
        /* FIXED: Use yield + sleep for Windows */
        task_yield();
        rtos_port_sleep_ms(0);
    }
}

/* ============================================================================
 * KERNEL INITIALIZATION - Updated for setjmp/longjmp
 * ============================================================================ */

KernelError kernel_init(void) {
    if (g_kernel.kernel_initialized) {
        return KERNEL_SUCCESS;
    }

    memset(&g_kernel, 0, sizeof(KernelState));

    for (int i = 0; i < AERORTOS_CONFIG_PRIORITY_LEVELS; i++) {
        g_kernel.ready_queue[i] = NULL;
    }

    g_kernel.scheduler_running = false;
    g_kernel.system_tick = 0;
    g_kernel.context_switches = 0;
    g_kernel.task_counter = 0;
    g_kernel.active_task_count = 0;
    g_kernel.kernel_initialized = true;
    g_kernel.scheduler_lock_depth = 0;
    g_kernel.critical_owner = 0;
    g_kernel.critical_depth = 0;
    g_kernel.last_cpu_update = 0;
    g_kernel.tick_frequency = 1000;
    g_kernel.timer_pending = 0;
    g_kernel.timer_initialized = false;

    for (int i = 0; i < AERORTOS_CONFIG_MAX_WAITING_NODES; i++) {
        g_kernel.waiting_node_pool[i].used = false;
        g_kernel.waiting_node_pool[i].task = NULL;
        g_kernel.waiting_node_pool[i].next = NULL;
    }
    g_kernel.waiting_node_count = 0;

    for (int i = 0; i < AERORTOS_CONFIG_MAX_TASKS; i++) {
        g_kernel.task_pool[i].task_id = 0;
        g_kernel.task_pool[i].tcb_index = i;
        g_kernel.task_pool[i].state = TASK_STATE_TERMINATED;
        g_kernel.task_pool[i].stack = g_kernel.task_stacks[i];
        g_kernel.task_pool[i].stack_size = AERORTOS_CONFIG_STACK_SIZE_DEFAULT;
        g_kernel.task_pool[i].is_idle = false;
        g_kernel.task_pool[i].cpu_usage_percent = 0;
        g_kernel.task_pool[i].last_cpu_update = 0;
    }

    /* Initialize port - critical section is initialized here */
    rtos_port_init();

    /* Create idle task - Updated for setjmp/longjmp */
    g_kernel.idle_task.task_id = 0xFFFFFFFF;
    g_kernel.idle_task.tcb_index = AERORTOS_CONFIG_MAX_TASKS;
    strcpy(g_kernel.idle_task.name, "Idle");
    g_kernel.idle_task.entry_point = idle_task_function;
    g_kernel.idle_task.priority = TASK_PRIORITY_IDLE;
    g_kernel.idle_task.base_priority = TASK_PRIORITY_IDLE;
    g_kernel.idle_task.state = TASK_STATE_READY;
    g_kernel.idle_task.first_run = true;
    g_kernel.idle_task.stack = g_kernel.idle_stack;
    g_kernel.idle_task.stack_size = AERORTOS_CONFIG_IDLE_STACK_SIZE;
    g_kernel.idle_task.in_ready_queue = false;
    g_kernel.idle_task.is_suspended = false;
    g_kernel.idle_task.is_idle = true;
    g_kernel.idle_task.stack_canary_top = 0xDEADBEEF;
    g_kernel.idle_task.stack_canary_bottom = 0xDEADBEEF;
    g_kernel.idle_task.cpu_usage_percent = 0;
    g_kernel.idle_task.last_cpu_update = 0;
    for (uint32_t i = 0; i < AERORTOS_CONFIG_IDLE_STACK_SIZE / sizeof(uint32_t); i++) {
        ((uint32_t*)g_kernel.idle_stack)[i] = 0xDEADBEEF;
    }
    
    /* FIXED: Set up idle task context with setjmp/longjmp */
    if (setjmp(g_kernel.idle_task.context) == 0) {
        g_kernel.idle_task.first_run = true;
    }
    
    kernel_add_to_ready_queue(&g_kernel.idle_task);

    g_kernel.timer_initialized = true;
    return KERNEL_SUCCESS;
}

/* ============================================================================
 * KERNEL START - Updated for setjmp/longjmp
 * ============================================================================ */

KernelError kernel_start(void) {
    if (!g_kernel.kernel_initialized) {
        return KERNEL_ERROR_RESOURCE_UNAVAILABLE;
    }

    if (g_kernel.scheduler_running) {
        return KERNEL_SUCCESS;
    }

    TaskControlBlock *first_task = NULL;
    for (int i = AERORTOS_CONFIG_PRIORITY_LEVELS - 1; i >= 0; i--) {
        if (g_kernel.ready_queue[i] != NULL) {
            first_task = g_kernel.ready_queue[i];
            kernel_remove_from_ready_queue(first_task);
            break;
        }
    }

    if (first_task == NULL) {
        return KERNEL_ERROR_TASK_NOT_FOUND;
    }

    g_kernel.current_task = first_task;
    first_task->state = TASK_STATE_RUNNING;
    first_task->first_run = true;
    g_kernel.scheduler_running = true;

    printf("\n=== AeroRTOS Scheduler Started ===\n");
    printf("First Task: %s (ID: %u, Priority: %d)\n\n", 
           first_task->name, first_task->task_id, first_task->priority);

    /* FIXED: Start first task with setjmp/longjmp */
    if (setjmp(first_task->context) == 0) {
        /* This is the first run - execute task entry */
        task_entry_wrapper();
    }
    
    return KERNEL_SUCCESS;
}

/* ============================================================================
 * TICK HANDLER
 * ============================================================================ */

void kernel_tick_handler(void) {
    /* g_kernel.system_tick is now reliably incremented by the hardware timer directly */

    critical_enter();
    
    /* Check blocked tasks for timeout */
    TaskControlBlock *task = g_kernel.blocked_list;
    while (task != NULL) {
        TaskControlBlock *next = task->next;
        if (task->wake_time <= g_kernel.system_tick) {
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
            task->next = NULL;
            task->prev = NULL;
            task->waiting_resource = NULL;
            
            kernel_add_to_ready_queue(task);
        }
        task = next;
    }

    /* Update timers */
    for (int i = 0; i < AERORTOS_CONFIG_MAX_TIMERS; i++) {
        TimerControlBlock *timer = &g_kernel.timers[i];
        if (timer->is_active && timer->timer_id != 0) {
            if (timer->remaining > 0) {
                timer->remaining--;
                if (timer->remaining == 0) {
                    if (timer->is_periodic) {
                        timer->remaining = timer->period;
                    } else {
                        timer->is_active = false;
                    }
                    if (timer->callback) {
                        timer->callback(timer->args);
                    }
                }
            }
        }
    }

    /* Update current task execution time */
    if (g_kernel.current_task && g_kernel.current_task->state == TASK_STATE_RUNNING) {
        g_kernel.current_task->execution_time++;
        /* Check deadline */
        if (g_kernel.current_task->deadline_ms > 0 &&
            g_kernel.current_task->execution_time > g_kernel.current_task->deadline_ms) {
            g_kernel.current_task->deadline_misses++;
            if (g_kernel.current_task->deadline_callback) {
                g_kernel.current_task->deadline_callback(g_kernel.current_task->task_id);
            }
        }
    }

    critical_exit();

    /* Schedule if not in ISR */
    if (g_kernel.isr_nesting_depth == 0) {
        kernel_schedule();
    }
}

/* ============================================================================
 * TASK MANAGEMENT
 * ============================================================================ */

KernelError task_create(const char *name, TaskFunction entry, 
                        void *params, TaskPriority priority, 
                        uint32_t stack_size, uint32_t *task_id) {
    critical_enter();
    
    if (!g_kernel.kernel_initialized) {
        critical_exit();
        return KERNEL_ERROR_RESOURCE_UNAVAILABLE;
    }

    if (entry == NULL || priority >= AERORTOS_CONFIG_PRIORITY_LEVELS) {
        critical_exit();
        return KERNEL_ERROR_INVALID_PARAM;
    }

    if (stack_size < AERORTOS_CONFIG_STACK_SIZE_MIN) {
        stack_size = AERORTOS_CONFIG_STACK_SIZE_DEFAULT;
    }
    if (stack_size > AERORTOS_CONFIG_STACK_SIZE_DEFAULT) {
        stack_size = AERORTOS_CONFIG_STACK_SIZE_DEFAULT;
    }

    TaskControlBlock *task = NULL;
    int free_index = -1;
    for (int i = 0; i < AERORTOS_CONFIG_MAX_TASKS; i++) {
        if (g_kernel.task_pool[i].task_id == 0) {
            task = &g_kernel.task_pool[i];
            free_index = i;
            break;
        }
    }

    if (task == NULL) {
        critical_exit();
        return KERNEL_ERROR_OUT_OF_MEMORY;
    }

    memset(task, 0, sizeof(TaskControlBlock));
    task->task_id = ++g_kernel.task_counter;
    task->tcb_index = free_index;
    strncpy(task->name, name, sizeof(task->name) - 1);
    task->name[sizeof(task->name) - 1] = '\0';
    task->entry_point = entry;
    task->parameters = params;
    task->priority = priority;
    task->base_priority = priority;
    task->state = TASK_STATE_READY;
    task->first_run = true;
    task->in_ready_queue = false;
    task->is_suspended = false;
    task->is_idle = false;
    task->waiting_resource = NULL;
    task->stack_size = stack_size;
    task->stack = g_kernel.task_stacks[free_index];
    task->cpu_usage_percent = 0;
    task->last_cpu_update = 0;
    
    task->stack_canary_top = 0xDEADBEEF;
    task->stack_canary_bottom = 0xDEADBEEF;
    for (uint32_t i = 0; i < stack_size / sizeof(uint32_t); i++) {
        ((uint32_t*)task->stack)[i] = 0xDEADBEEF;
    }

    /* FIXED: Set up context for first run with setjmp/longjmp */
    if (setjmp(task->context) == 0) {
        task->first_run = true;
    }

    kernel_add_to_ready_queue(task);
    g_kernel.active_task_count++;

    if (task_id) {
        *task_id = task->task_id;
    }

    critical_exit();
    
    if (g_kernel.scheduler_running) {
        kernel_schedule();
    }

    return KERNEL_SUCCESS;
}

KernelError task_delete(uint32_t task_id) {
    critical_enter();
    
    if (!g_kernel.kernel_initialized) {
        critical_exit();
        return KERNEL_ERROR_RESOURCE_UNAVAILABLE;
    }

    TaskControlBlock *task = kernel_find_task_by_id(task_id);
    if (!task) {
        critical_exit();
        return KERNEL_ERROR_TASK_NOT_FOUND;
    }

    kernel_cleanup_task(task);
    if (g_kernel.active_task_count > 0) {
        g_kernel.active_task_count--;
    }

    critical_exit();
    
    if (g_kernel.scheduler_running && task != g_kernel.current_task) {
        kernel_schedule();
    }

    return KERNEL_SUCCESS;
}

static void kernel_cleanup_task(TaskControlBlock *task) {
    if (!task) return;

    if (task->in_ready_queue) {
        kernel_remove_from_ready_queue(task);
    }

    if (task->state == TASK_STATE_BLOCKED || task->state == TASK_STATE_WAITING) {
        if (task->prev) {
            task->prev->next = task->next;
        } else {
            g_kernel.blocked_list = task->next;
        }
        if (task->next) {
            task->next->prev = task->prev;
        }
    }

    if (task->waiting_resource) {
        task->waiting_resource = NULL;
    }

    task->state = TASK_STATE_TERMINATED;
    task->task_id = 0;
    task->next = NULL;
    task->prev = NULL;
    task->waiting_resource = NULL;
    task->in_ready_queue = false;
}

KernelError task_suspend(uint32_t task_id) {
    critical_enter();
    
    if (g_kernel.isr_nesting_depth > 0) {
        critical_exit();
        return KERNEL_ERROR_ISR_CONTEXT;
    }

    TaskControlBlock *task = kernel_find_task_by_id(task_id);
    if (!task) {
        critical_exit();
        return KERNEL_ERROR_TASK_NOT_FOUND;
    }

    if (task->is_suspended || task->is_idle) {
        critical_exit();
        return KERNEL_ERROR_INVALID_PARAM;
    }

    task->is_suspended = true;
    
    if (task->state == TASK_STATE_READY) {
        kernel_remove_from_ready_queue(task);
        task->state = TASK_STATE_SUSPENDED;
    } else if (task->state == TASK_STATE_RUNNING) {
        task->state = TASK_STATE_SUSPENDED;
    }

    critical_exit();
    
    if (task == g_kernel.current_task) {
        kernel_schedule();
    }

    return KERNEL_SUCCESS;
}

KernelError task_resume(uint32_t task_id) {
    critical_enter();

    TaskControlBlock *task = kernel_find_task_by_id(task_id);
    if (!task) {
        critical_exit();
        return KERNEL_ERROR_TASK_NOT_FOUND;
    }

    if (!task->is_suspended) {
        critical_exit();
        return KERNEL_SUCCESS;
    }

    task->is_suspended = false;
    
    if (task->state == TASK_STATE_SUSPENDED) {
        task->state = TASK_STATE_READY;
        kernel_add_to_ready_queue(task);
    }

    critical_exit();
    kernel_schedule();

    return KERNEL_SUCCESS;
}

KernelError task_sleep(uint32_t ms) {
    if (!g_kernel.scheduler_running) {
        return KERNEL_ERROR_RESOURCE_UNAVAILABLE;
    }

    if (g_kernel.isr_nesting_depth > 0) {
        return KERNEL_ERROR_ISR_CONTEXT;
    }

    critical_enter();

    TaskControlBlock *task = g_kernel.current_task;
    if (!task || task->task_id == 0) {
        critical_exit();
        return KERNEL_ERROR_TASK_NOT_FOUND;
    }

    if (ms == 0) {
        critical_exit();
        task_yield();
        return KERNEL_SUCCESS;
    }

    if (task->state == TASK_STATE_RUNNING) {
        task->state = TASK_STATE_BLOCKED;
        task->wake_time = g_kernel.system_tick + ms;
        task->next = NULL;
        task->prev = NULL;

        task->next = g_kernel.blocked_list;
        if (g_kernel.blocked_list) {
            g_kernel.blocked_list->prev = task;
        }
        g_kernel.blocked_list = task;

        critical_exit();
        kernel_schedule();
        return KERNEL_SUCCESS;
    }

    critical_exit();
    return KERNEL_ERROR_INVALID_PARAM;
}

KernelError task_yield(void) {
    if (!g_kernel.scheduler_running) {
        return KERNEL_ERROR_RESOURCE_UNAVAILABLE;
    }

    if (g_kernel.isr_nesting_depth > 0) {
        return KERNEL_ERROR_ISR_CONTEXT;
    }

    critical_enter();

    if (g_kernel.current_task && g_kernel.current_task->state == TASK_STATE_RUNNING) {
        kernel_add_to_ready_queue(g_kernel.current_task);
        critical_exit();
        kernel_schedule();
        return KERNEL_SUCCESS;
    }

    critical_exit();
    return KERNEL_ERROR_INVALID_PARAM;
}

/* ============================================================================
 * TASK INFORMATION
 * ============================================================================ */

uint32_t task_get_current_id(void) {
    if (!g_kernel.current_task) {
        return 0;
    }
    return g_kernel.current_task->task_id;
}

const char* task_get_name(uint32_t task_id) {
    TaskControlBlock *task = kernel_find_task_by_id(task_id);
    return task ? task->name : NULL;
}

TaskState task_get_state(uint32_t task_id) {
    TaskControlBlock *task = kernel_find_task_by_id(task_id);
    return task ? task->state : TASK_STATE_TERMINATED;
}

TaskPriority task_get_priority(uint32_t task_id) {
    TaskControlBlock *task = kernel_find_task_by_id(task_id);
    return task ? task->priority : TASK_PRIORITY_IDLE;
}

KernelError task_set_priority(uint32_t task_id, TaskPriority priority) {
    if (priority >= AERORTOS_CONFIG_PRIORITY_LEVELS) {
        return KERNEL_ERROR_INVALID_PARAM;
    }

    critical_enter();

    TaskControlBlock *task = kernel_find_task_by_id(task_id);
    if (!task) {
        critical_exit();
        return KERNEL_ERROR_TASK_NOT_FOUND;
    }

    bool was_ready = task->in_ready_queue;
    bool was_running = (task == g_kernel.current_task);

    if (was_ready) {
        kernel_remove_from_ready_queue(task);
    }
    
    task->priority = priority;
    task->base_priority = priority;
    
    if (was_ready) {
        kernel_add_to_ready_queue(task);
    }

    critical_exit();

    if (was_running) {
        kernel_schedule();
    }

    return KERNEL_SUCCESS;
}

void task_set_deadline(uint32_t task_id, uint32_t deadline_ms, DeadlineCallback callback) {
    critical_enter();
    
    TaskControlBlock *task = kernel_find_task_by_id(task_id);
    if (task) {
        task->deadline_ms = deadline_ms;
        task->deadline_callback = callback;
    }
    
    critical_exit();
}

uint32_t task_get_stack_usage(uint32_t task_id) {
    critical_enter();
    
    TaskControlBlock *task = kernel_find_task_by_id(task_id);
    if (!task || !task->stack) {
        critical_exit();
        return 0;
    }
    
    if (task->stack_canary_top != 0xDEADBEEF || 
        task->stack_canary_bottom != 0xDEADBEEF) {
        critical_exit();
        return 0;
    }
    
    uint32_t *stack = (uint32_t*)task->stack;
    uint32_t stack_words = task->stack_size / sizeof(uint32_t);
    uint32_t used = 0;
    
    for (int i = stack_words - 1; i >= 0; i--) {
        if (stack[i] != 0xDEADBEEF) {
            used = i + 1;
            break;
        }
    }
    
    critical_exit();
    return used * sizeof(uint32_t);
}

uint32_t task_get_cpu_usage(uint32_t task_id) {
    critical_enter();
    
    TaskControlBlock *task = kernel_find_task_by_id(task_id);
    uint32_t cpu = task ? task->cpu_usage_percent : 0;
    
    critical_exit();
    return cpu;
}

uint32_t task_get_context_switches(uint32_t task_id) {
    critical_enter();
    
    TaskControlBlock *task = kernel_find_task_by_id(task_id);
    uint32_t cs = task ? task->context_switches : 0;
    
    critical_exit();
    return cs;
}

/* ============================================================================
 * SYSTEM INFORMATION
 * ============================================================================ */

void kernel_lock_scheduler(void) {
    g_kernel.scheduler_lock_depth++;
}

void kernel_unlock_scheduler(void) {
    if (g_kernel.scheduler_lock_depth > 0) {
        g_kernel.scheduler_lock_depth--;
    }

    if (g_kernel.scheduler_lock_depth == 0) {
        kernel_schedule();
    }
}

uint32_t kernel_get_tick(void) {
    return g_kernel.system_tick;
}

uint32_t kernel_get_context_switches(void) {
    return g_kernel.context_switches;
}

uint32_t kernel_get_task_count(void) {
    return g_kernel.task_counter;
}

uint32_t kernel_get_active_task_count(void) {
    return g_kernel.active_task_count;
}

const char* kernel_get_error_string(KernelError error) {
    if (error >= 0 && error < sizeof(g_error_messages) / sizeof(g_error_messages[0])) {
        return g_error_messages[error];
    }
    return "UNKNOWN_ERROR";
}

/* ============================================================================
 * STATUS PRINTING
 * ============================================================================ */

void kernel_print_status(void) {
    critical_enter();

    printf("\nAeroRTOS kernel status\n");
    printf("System tick:      %u\n", g_kernel.system_tick);
    printf("Context switches: %u\n", g_kernel.context_switches);
    printf("Tasks created:    %u\n", g_kernel.task_counter);
    printf("Active tasks:     %u\n", g_kernel.active_task_count);
    printf("ISR nesting:      %u\n", g_kernel.isr_nesting_depth);
    printf("Idle ticks:       %u\n", g_kernel.idle_ticks);
    printf("Current task:     %s (ID %u, priority %d)\n",
           g_kernel.current_task ? g_kernel.current_task->name : "none",
           g_kernel.current_task ? g_kernel.current_task->task_id : 0,
           g_kernel.current_task ? (int)g_kernel.current_task->priority : -1);

    printf("\nTasks:\n");
    printf("  %-6s %-18s %-10s %-8s %-10s %-6s\n", "ID", "Name", "State", "Priority", "Stack", "CPU");
    for (int i = 0; i < AERORTOS_CONFIG_MAX_TASKS; i++) {
        TaskControlBlock *task = &g_kernel.task_pool[i];
        if (task->task_id != 0 && task->state != TASK_STATE_TERMINATED) {
            const char *state_str = task->state <= TASK_STATE_WAITING ? task_state_strings[task->state] : "UNKNOWN";
            printf("  %-6u %-18s %-10s %-8d %-10u %u%%\n",
                   task->task_id,
                   task->name,
                   state_str,
                   task->priority,
                   task_get_stack_usage(task->task_id),
                   task->cpu_usage_percent);
        }
    }

    printf("  %-6s %-18s %-10s %-8d %-10u %u%%\n",
           "IDLE",
           "Idle",
           task_state_strings[g_kernel.idle_task.state],
           g_kernel.idle_task.priority,
           task_get_stack_usage(0xFFFFFFFF),
           g_kernel.idle_task.cpu_usage_percent);

    critical_exit();
}

void kernel_print_cpu_usage(void) {
    critical_enter();

    printf("\nAeroRTOS CPU usage\n");
    for (int i = 0; i < AERORTOS_CONFIG_MAX_TASKS; i++) {
        TaskControlBlock *task = &g_kernel.task_pool[i];
        if (task->task_id != 0 && task->state != TASK_STATE_TERMINATED) {
            printf("  %-18s %u%%\n", task->name, task->cpu_usage_percent);
        }
    }
    printf("  %-18s %u%%\n", "Idle", g_kernel.idle_task.cpu_usage_percent);

    critical_exit();
}
