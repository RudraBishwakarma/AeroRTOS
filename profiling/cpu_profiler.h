#ifndef CPU_PROFILER_H
#define CPU_PROFILER_H

#include <stdint.h>
#include <stdbool.h>
#include "rtos_kernel.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * CPU Profiler Configuration
 * ============================================================================ */

#define CPU_PROFILER_MAX_TASKS 32
#define CPU_PROFILER_HISTORY_SIZE 100
#define CPU_PROFILER_MAX_SAMPLES 1000

/* CPU Usage Sample */
typedef struct {
    uint32_t timestamp;
    uint32_t task_id;
    char task_name[32];
    float cpu_usage;
    uint32_t execution_time;
    uint32_t context_switches;
    uint32_t deadline_misses;
} CpuSample;

/* Task CPU Statistics */
typedef struct {
    uint32_t task_id;
    char task_name[32];
    uint32_t total_execution_time;
    uint32_t min_execution_time;
    uint32_t max_execution_time;
    uint32_t avg_execution_time;
    uint32_t context_switches;
    uint32_t deadline_misses;
    float cpu_usage_percent;
    float peak_cpu_usage;
    float avg_cpu_usage;
    uint32_t sample_count;
    uint32_t priority;
    TaskState state;
} TaskCpuStats;

/* CPU Profiler Context */
typedef struct {
    TaskCpuStats tasks[CPU_PROFILER_MAX_TASKS];
    CpuSample history[CPU_PROFILER_HISTORY_SIZE];
    uint32_t history_count;
    uint32_t history_index;
    uint32_t total_samples;
    uint32_t profiler_start_time;
    uint32_t profiler_duration_ms;
    float total_cpu_usage;
    float idle_cpu_usage;
    float system_cpu_usage;
    bool enabled;
    uint32_t sample_interval_ms;
    uint32_t last_sample_time;
} CpuProfiler;

/* ============================================================================
 * Function Prototypes
 * ============================================================================ */

/* Profiler Management */
void cpu_profiler_init(void);
void cpu_profiler_start(void);
void cpu_profiler_stop(void);
void cpu_profiler_update(void);
void cpu_profiler_reset(void);

/* Task Registration */
bool cpu_profiler_register_task(uint32_t task_id, const char *name, uint32_t priority);
bool cpu_profiler_unregister_task(uint32_t task_id);

/* Statistics Queries */
TaskCpuStats cpu_profiler_get_task_stats(uint32_t task_id);
float cpu_profiler_get_total_cpu_usage(void);
float cpu_profiler_get_idle_cpu_usage(void);
float cpu_profiler_get_system_cpu_usage(void);
uint32_t cpu_profiler_get_sample_count(void);

/* History */
bool cpu_profiler_get_history(uint32_t start_index, uint32_t count, CpuSample *samples);
uint32_t cpu_profiler_get_history_count(void);

/* Reporting */
void cpu_profiler_print_stats(void);
void cpu_profiler_print_task_stats(uint32_t task_id);
void cpu_profiler_print_summary(void);
void cpu_profiler_generate_report(void);

/* Optimization Suggestions */
void cpu_profiler_get_suggestions(char *buffer, uint32_t buffer_size);
bool cpu_profiler_identify_bottlenecks(uint32_t *task_ids, uint32_t *count);

#ifdef __cplusplus
}
#endif

#endif /* CPU_PROFILER_H */