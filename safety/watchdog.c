#include "watchdog.h"
#include "rtos_kernel.h"
#include <string.h>
#include <stdio.h>

/* ============================================================================
 * Global Watchdog Context
 * ============================================================================ */

static WatchdogContext g_watchdog = {0};
static bool g_watchdog_initialized = false;

/* ============================================================================
 * State Strings
 * ============================================================================ */

static const char* g_watchdog_state_strings[] = {
    "OK",
    "WARNING",
    "CRITICAL",
    "TIMEOUT"
};

/* ============================================================================
 * Watchdog Management
 * ============================================================================ */

void watchdog_init(void) {
    if (g_watchdog_initialized) {
        return;
    }
    
    memset(&g_watchdog, 0, sizeof(WatchdogContext));
    g_watchdog.global_timeout_ms = WATCHDOG_DEFAULT_TIMEOUT_MS;
    g_watchdog.check_interval_ms = 100;
    g_watchdog.enabled = true;
    g_watchdog.global_state = WATCHDOG_STATE_OK;
    g_watchdog.last_check_time = 0;
    g_watchdog_initialized = true;
    
    printf("[WATCHDOG] Initialized (Global Timeout: %u ms, Check Interval: %u ms)\n",
           g_watchdog.global_timeout_ms, g_watchdog.check_interval_ms);
}

void watchdog_start(void) {
    if (!g_watchdog_initialized) {
        watchdog_init();
    }
    g_watchdog.enabled = true;
    g_watchdog.last_check_time = kernel_get_tick();
    printf("[WATCHDOG] Started\n");
}

void watchdog_stop(void) {
    g_watchdog.enabled = false;
    printf("[WATCHDOG] Stopped\n");
}

void watchdog_update(void) {
    if (!g_watchdog_initialized || !g_watchdog.enabled) {
        return;
    }
    
    uint32_t current_time = kernel_get_tick();
    if (current_time - g_watchdog.last_check_time < g_watchdog.check_interval_ms) {
        return;
    }
    
    g_watchdog.last_check_time = current_time;
    g_watchdog.global_state = WATCHDOG_STATE_OK;
    
    for (uint32_t i = 0; i < g_watchdog.task_count; i++) {
        WatchdogTaskEntry *task = &g_watchdog.tasks[i];
        if (!task->enabled) {
            continue;
        }
        
        uint32_t elapsed = current_time - task->last_kick_time;
        
        /* Check if task is still responsive */
        if (elapsed > task->timeout_ms) {
            task->missed_kicks++;
            task->state = WATCHDOG_STATE_TIMEOUT;
            g_watchdog.total_timeouts++;
            
            printf("[WATCHDOG] TASK TIMEOUT: %s (ID: %u, Missed: %u)\n",
                   task->task_name, task->task_id, task->missed_kicks);
            
            /* Critical task timeout - trigger recovery */
            if (task->is_critical && task->recovery_callback) {
                printf("[WATCHDOG] CRITICAL: Executing recovery for %s\n", task->task_name);
                task->recovery_callback(task->task_id);
                g_watchdog.total_recoveries++;
            }
            
            /* Update global state */
            if (task->is_critical) {
                g_watchdog.global_state = WATCHDOG_STATE_CRITICAL;
            } else if (g_watchdog.global_state < WATCHDOG_STATE_WARNING) {
                g_watchdog.global_state = WATCHDOG_STATE_WARNING;
            }
            
            /* Check if task has exceeded max missed kicks */
            if (task->missed_kicks >= task->max_missed_kicks) {
                printf("[WATCHDOG] FATAL: Task %s exceeded max missed kicks (%u)\n",
                       task->task_name, task->max_missed_kicks);
                task->state = WATCHDOG_STATE_CRITICAL;
            }
        } else if (elapsed > task->timeout_ms * 0.7f) {
            /* Warning threshold (70% of timeout) */
            if (task->state == WATCHDOG_STATE_OK) {
                task->state = WATCHDOG_STATE_WARNING;
                printf("[WATCHDOG] WARNING: %s approaching timeout (%u ms)\n",
                       task->task_name, elapsed);
            }
        } else {
            task->state = WATCHDOG_STATE_OK;
        }
    }
}

void watchdog_reset(void) {
    for (uint32_t i = 0; i < g_watchdog.task_count; i++) {
        g_watchdog.tasks[i].missed_kicks = 0;
        g_watchdog.tasks[i].state = WATCHDOG_STATE_OK;
        g_watchdog.tasks[i].last_kick_time = kernel_get_tick();
    }
    g_watchdog.global_state = WATCHDOG_STATE_OK;
    g_watchdog.total_timeouts = 0;
    g_watchdog.total_recoveries = 0;
    printf("[WATCHDOG] Reset\n");
}

/* ============================================================================
 * Task Registration
 * ============================================================================ */

bool watchdog_register_task(uint32_t task_id, const char *name, 
                            uint32_t timeout_ms, bool is_critical,
                            void (*recovery_callback)(uint32_t)) {
    if (!g_watchdog_initialized) {
        watchdog_init();
    }
    
    if (g_watchdog.task_count >= WATCHDOG_MAX_TASKS) {
        printf("[WATCHDOG] ERROR: Max tasks exceeded\n");
        return false;
    }
    
    /* Check if task already registered */
    for (uint32_t i = 0; i < g_watchdog.task_count; i++) {
        if (g_watchdog.tasks[i].task_id == task_id) {
            return false;
        }
    }
    
    WatchdogTaskEntry *entry = &g_watchdog.tasks[g_watchdog.task_count];
    entry->task_id = task_id;
    strncpy(entry->task_name, name, sizeof(entry->task_name) - 1);
    entry->task_name[sizeof(entry->task_name) - 1] = '\0';
    entry->timeout_ms = timeout_ms > 0 ? timeout_ms : g_watchdog.global_timeout_ms;
    entry->last_kick_time = kernel_get_tick();
    entry->missed_kicks = 0;
    entry->max_missed_kicks = 3;
    entry->state = WATCHDOG_STATE_OK;
    entry->enabled = true;
    entry->is_critical = is_critical;
    entry->recovery_callback = recovery_callback;
    
    g_watchdog.task_count++;
    
    printf("[WATCHDOG] Registered task: %s (ID: %u, Timeout: %u ms, Critical: %s)\n",
           name, task_id, entry->timeout_ms, is_critical ? "YES" : "NO");
    
    return true;
}

bool watchdog_unregister_task(uint32_t task_id) {
    for (uint32_t i = 0; i < g_watchdog.task_count; i++) {
        if (g_watchdog.tasks[i].task_id == task_id) {
            /* Remove by shifting */
            for (uint32_t j = i; j < g_watchdog.task_count - 1; j++) {
                g_watchdog.tasks[j] = g_watchdog.tasks[j + 1];
            }
            g_watchdog.task_count--;
            return true;
        }
    }
    return false;
}

/* ============================================================================
 * Task Kicking
 * ============================================================================ */

bool watchdog_kick(uint32_t task_id) {
    if (!g_watchdog_initialized) {
        return false;
    }
    
    for (uint32_t i = 0; i < g_watchdog.task_count; i++) {
        if (g_watchdog.tasks[i].task_id == task_id) {
            g_watchdog.tasks[i].last_kick_time = kernel_get_tick();
            g_watchdog.tasks[i].missed_kicks = 0;
            if (g_watchdog.tasks[i].state == WATCHDOG_STATE_WARNING ||
                g_watchdog.tasks[i].state == WATCHDOG_STATE_TIMEOUT) {
                g_watchdog.tasks[i].state = WATCHDOG_STATE_OK;
                printf("[WATCHDOG] Task %s recovered\n", g_watchdog.tasks[i].task_name);
            }
            return true;
        }
    }
    return false;
}

bool watchdog_kick_current(void) {
    uint32_t current_id = task_get_current_id();
    return watchdog_kick(current_id);
}

/* ============================================================================
 * Status Queries
 * ============================================================================ */

WatchdogState watchdog_get_state(uint32_t task_id) {
    for (uint32_t i = 0; i < g_watchdog.task_count; i++) {
        if (g_watchdog.tasks[i].task_id == task_id) {
            return g_watchdog.tasks[i].state;
        }
    }
    return WATCHDOG_STATE_OK;
}

WatchdogState watchdog_get_global_state(void) {
    return g_watchdog.global_state;
}

uint32_t watchdog_get_timeouts(void) {
    return g_watchdog.total_timeouts;
}

uint32_t watchdog_get_recoveries(void) {
    return g_watchdog.total_recoveries;
}

const char* watchdog_get_state_string(WatchdogState state) {
    if (state >= 0 && state < sizeof(g_watchdog_state_strings) / sizeof(g_watchdog_state_strings[0])) {
        return g_watchdog_state_strings[state];
    }
    return "UNKNOWN";
}

/* ============================================================================
 * Configuration
 * ============================================================================ */

void watchdog_set_global_timeout(uint32_t timeout_ms) {
    g_watchdog.global_timeout_ms = timeout_ms;
    printf("[WATCHDOG] Global timeout set to %u ms\n", timeout_ms);
}

void watchdog_set_check_interval(uint32_t interval_ms) {
    g_watchdog.check_interval_ms = interval_ms;
    printf("[WATCHDOG] Check interval set to %u ms\n", interval_ms);
}

void watchdog_set_task_timeout(uint32_t task_id, uint32_t timeout_ms) {
    for (uint32_t i = 0; i < g_watchdog.task_count; i++) {
        if (g_watchdog.tasks[i].task_id == task_id) {
            g_watchdog.tasks[i].timeout_ms = timeout_ms;
            printf("[WATCHDOG] Task %s timeout set to %u ms\n", 
                   g_watchdog.tasks[i].task_name, timeout_ms);
            return;
        }
    }
}