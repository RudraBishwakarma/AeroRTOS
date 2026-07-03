#include "rtos_port.h"
#include "rtos_kernel.h"
#include <stdio.h>
#include <stdlib.h>
#include <windows.h>

/* ============================================================================
 * GLOBALS
 * ============================================================================ */

/* Critical Section - Exported for kernel use */
CRITICAL_SECTION g_kernel_critical_section;
static bool g_critical_initialized = false;

/* Timer Thread */
static HANDLE g_timer_thread = NULL;
static volatile bool g_timer_running = false;
static volatile bool g_timer_initialized = false;

/* High Resolution Timer */
static LARGE_INTEGER g_frequency;
static volatile uint64_t g_tick_count = 0;
static volatile uint64_t g_last_tick_time = 0;

/* ============================================================================
 * CRITICAL SECTION - Windows CRITICAL_SECTION
 * ============================================================================ */

void rtos_port_critical_init(void) {
    if (!g_critical_initialized) {
        InitializeCriticalSection(&g_kernel_critical_section);
        g_critical_initialized = true;
    }
}

void rtos_port_critical_enter(void) {
    if (g_critical_initialized) {
        EnterCriticalSection(&g_kernel_critical_section);
    }
}

void rtos_port_critical_exit(void) {
    if (g_critical_initialized) {
        LeaveCriticalSection(&g_kernel_critical_section);
    }
}

bool rtos_port_critical_try_enter(void) {
    if (g_critical_initialized) {
        return TryEnterCriticalSection(&g_kernel_critical_section) != 0;
    }
    return false;
}

/* ============================================================================
 * CONTEXT SWITCHING - setjmp/longjmp
 * ============================================================================ */

void rtos_port_context_switch(jmp_buf *old_context, jmp_buf *new_context) {
    if (setjmp(*old_context) == 0) {
        longjmp(*new_context, 1);
    }
}

/* ============================================================================
 * TASK ENTRY WRAPPER - Called from kernel
 * ============================================================================ */

void rtos_port_task_entry(void) {
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
 * TIMER - Windows Thread
 * ============================================================================ */

static DWORD WINAPI timer_thread_function(LPVOID lpParam) {
    (void)lpParam;
    
    LARGE_INTEGER current_time, last_time;
    QueryPerformanceCounter(&last_time);
    
    uint64_t interval_ticks = (g_frequency.QuadPart * 1000) / 1000000; /* 1ms in ticks */
    
    while (g_timer_running) {
        QueryPerformanceCounter(&current_time);
        
        uint64_t elapsed = (current_time.QuadPart - last_time.QuadPart);
        
        if (elapsed >= interval_ticks) {
            last_time = current_time;
            g_tick_count++;
            
            /* Directly increment the RTOS tick to ensure reliable timing */
            g_kernel.system_tick++;
            
            /* Set timer pending flag - will be processed in task context */
            g_kernel.timer_pending = 1;
        }
        
        /* Sleep for 1ms to prevent CPU spinning and yield properly */
        Sleep(1);
    }
    
    return 0;
}

void rtos_port_timer_init(void) {
    if (!g_timer_initialized) {
        QueryPerformanceFrequency(&g_frequency);
        rtos_port_critical_init();
        g_timer_initialized = true;
    }
}

void rtos_port_timer_start(void) {
    if (g_timer_thread == NULL && g_timer_initialized) {
        g_timer_running = true;
        g_timer_thread = CreateThread(
            NULL,           /* Security attributes */
            0,              /* Stack size (0 = default) */
            timer_thread_function,
            NULL,           /* Thread parameter */
            0,              /* Creation flags */
            NULL            /* Thread ID (not needed) */
        );
        
        if (g_timer_thread == NULL) {
            printf("[ERROR] Failed to create timer thread\n");
        } else {
            SetThreadPriority(g_timer_thread, THREAD_PRIORITY_TIME_CRITICAL);
        }
    }
}

void rtos_port_timer_stop(void) {
    if (g_timer_thread != NULL) {
        g_timer_running = false;
        WaitForSingleObject(g_timer_thread, 100);
        CloseHandle(g_timer_thread);
        g_timer_thread = NULL;
    }
}

void rtos_port_process_timer(void) {
    /* Process timer events - called from critical section */
    if (g_kernel.timer_pending) {
        g_kernel.timer_pending = 0;
        g_kernel.isr_nesting_depth++;
        kernel_tick_handler();
        g_kernel.isr_nesting_depth--;
    }
}

/* ============================================================================
 * SYSTEM TIME - Windows High Resolution Timer
 * ============================================================================ */

uint64_t rtos_port_get_time_us(void) {
    LARGE_INTEGER current_time;
    QueryPerformanceCounter(&current_time);
    return (uint64_t)((current_time.QuadPart * 1000000) / g_frequency.QuadPart);
}

uint32_t rtos_port_get_tick_ms(void) {
    return (uint32_t)g_tick_count;
}

/* ============================================================================
 * INTERRUPT CONTROL - No-op on Windows
 * ============================================================================ */

void rtos_port_disable_interrupts(void) {
    /* No-op for Windows simulation */
}

void rtos_port_enable_interrupts(void) {
    /* No-op for Windows simulation */
}

/* ============================================================================
 * SLEEP/YIELD
 * ============================================================================ */

void rtos_port_sleep_ms(uint32_t ms) {
    if (ms == 0) {
        Sleep(1);
    } else {
        Sleep(ms);
    }
}

void rtos_port_yield(void) {
    if (g_kernel.timer_pending) {
        g_kernel.timer_pending = 0;
        kernel_tick_handler();
    }
    Sleep(1);
}

/* ============================================================================
 * PORT INITIALIZATION - Called from kernel_init()
 * ============================================================================ */

void rtos_port_init(void) {
    rtos_port_timer_init();
    rtos_port_timer_start();
}

/* ============================================================================
 * PORT SHUTDOWN - Called on system exit
 * ============================================================================ */

void rtos_port_shutdown(void) {
    rtos_port_timer_stop();
    if (g_critical_initialized) {
        DeleteCriticalSection(&g_kernel_critical_section);
        g_critical_initialized = false;
    }
}