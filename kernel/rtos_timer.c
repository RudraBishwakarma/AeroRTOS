#include "rtos_kernel.h"
#include <string.h>

KernelError timer_create(const char *name, uint32_t period_ms, 
                         bool periodic, TimerCallback callback, 
                         void *args, uint32_t *timer_id) {
    critical_enter();
    
    if (!g_kernel.kernel_initialized) {
        critical_exit();
        return KERNEL_ERROR_RESOURCE_UNAVAILABLE;
    }
    
    if (period_ms == 0 || callback == NULL) {
        critical_exit();
        return KERNEL_ERROR_INVALID_PARAM;
    }
    
    TimerControlBlock *timer = NULL;
    for (int i = 0; i < AERORTOS_CONFIG_MAX_TIMERS; i++) {
        if (g_kernel.timers[i].timer_id == 0) {
            timer = &g_kernel.timers[i];
            break;
        }
    }
    
    if (!timer) {
        critical_exit();
        return KERNEL_ERROR_OUT_OF_MEMORY;
    }
    
    memset(timer, 0, sizeof(TimerControlBlock));
    timer->timer_id = ++g_kernel.timer_counter;
    strncpy(timer->name, name, sizeof(timer->name) - 1);
    timer->name[sizeof(timer->name) - 1] = '\0';
    timer->period = period_ms;
    timer->remaining = period_ms;
    timer->callback = callback;
    timer->args = args;
    timer->is_periodic = periodic;
    timer->is_active = false;
    
    if (timer_id) {
        *timer_id = timer->timer_id;
    }
    
    critical_exit();
    return KERNEL_SUCCESS;
}

KernelError timer_delete(uint32_t timer_id) {
    critical_enter();
    
    for (int i = 0; i < AERORTOS_CONFIG_MAX_TIMERS; i++) {
        if (g_kernel.timers[i].timer_id == timer_id) {
            memset(&g_kernel.timers[i], 0, sizeof(TimerControlBlock));
            critical_exit();
            return KERNEL_SUCCESS;
        }
    }
    
    critical_exit();
    return KERNEL_ERROR_RESOURCE_UNAVAILABLE;
}

KernelError timer_start(uint32_t timer_id) {
    critical_enter();
    
    for (int i = 0; i < AERORTOS_CONFIG_MAX_TIMERS; i++) {
        if (g_kernel.timers[i].timer_id == timer_id) {
            g_kernel.timers[i].is_active = true;
            g_kernel.timers[i].remaining = g_kernel.timers[i].period;
            critical_exit();
            return KERNEL_SUCCESS;
        }
    }
    
    critical_exit();
    return KERNEL_ERROR_RESOURCE_UNAVAILABLE;
}

KernelError timer_stop(uint32_t timer_id) {
    critical_enter();
    
    for (int i = 0; i < AERORTOS_CONFIG_MAX_TIMERS; i++) {
        if (g_kernel.timers[i].timer_id == timer_id) {
            g_kernel.timers[i].is_active = false;
            critical_exit();
            return KERNEL_SUCCESS;
        }
    }
    
    critical_exit();
    return KERNEL_ERROR_RESOURCE_UNAVAILABLE;
}

KernelError timer_reset(uint32_t timer_id) {
    critical_enter();
    
    for (int i = 0; i < AERORTOS_CONFIG_MAX_TIMERS; i++) {
        if (g_kernel.timers[i].timer_id == timer_id) {
            g_kernel.timers[i].remaining = g_kernel.timers[i].period;
            critical_exit();
            return KERNEL_SUCCESS;
        }
    }
    
    critical_exit();
    return KERNEL_ERROR_RESOURCE_UNAVAILABLE;
}