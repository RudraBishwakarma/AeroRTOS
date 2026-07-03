#ifndef AERORTOS_PORT_H
#define AERORTOS_PORT_H

#include <windows.h>
#include <setjmp.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Windows Port Layer for AeroRTOS
 * Uses setjmp/longjmp for context switching
 * Uses Windows Critical Sections for synchronization
 * Uses Windows Thread for timer
 * ============================================================================ */

/* ============================================================================
 * Critical Section - Windows CRITICAL_SECTION
 * ============================================================================ */
void rtos_port_critical_init(void);
void rtos_port_critical_enter(void);
void rtos_port_critical_exit(void);
bool rtos_port_critical_try_enter(void);

/* ============================================================================
 * Context Switching - setjmp/longjmp
 * ============================================================================ */
void rtos_port_context_switch(jmp_buf *old_context, jmp_buf *new_context);

/* ============================================================================
 * Task Entry Wrapper
 * ============================================================================ */
void rtos_port_task_entry(void);

/* ============================================================================
 * Timer - Windows Thread
 * ============================================================================ */
void rtos_port_timer_init(void);
void rtos_port_timer_start(void);
void rtos_port_timer_stop(void);
void rtos_port_process_timer(void);

/* ============================================================================
 * System Time - Windows High Resolution Timer
 * ============================================================================ */
uint64_t rtos_port_get_time_us(void);
uint32_t rtos_port_get_tick_ms(void);

/* ============================================================================
 * Interrupt Control - No-op on Windows
 * ============================================================================ */
void rtos_port_disable_interrupts(void);
void rtos_port_enable_interrupts(void);

/* ============================================================================
 * Sleep/Yield
 * ============================================================================ */
void rtos_port_sleep_ms(uint32_t ms);
void rtos_port_yield(void);

/* ============================================================================
 * Port Init/Shutdown
 * ============================================================================ */
void rtos_port_init(void);
void rtos_port_shutdown(void);

/* ============================================================================
 * Global Critical Section - For kernel use
 * ============================================================================ */
extern CRITICAL_SECTION g_kernel_critical_section;

#ifdef __cplusplus
}
#endif

#endif /* AERORTOS_PORT_H */