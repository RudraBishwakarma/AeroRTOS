#include "cpu_profiler.h"
#include "rtos_kernel.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* ============================================================================
 * Global Profiler Context
 * ============================================================================ */

static CpuProfiler g_profiler = {0};
static bool g_profiler_initialized = false;

/* ============================================================================
 * Profiler Management
 * ============================================================================ */

void cpu_profiler_init(void) {
    if (g_profiler_initialized) {
        return;
    }
    
    memset(&g_profiler, 0, sizeof(CpuProfiler));
    g_profiler.enabled = true;
    g_profiler.sample_interval_ms = 100;  /* 10Hz sampling */
    g_profiler.last_sample_time = 0;
    g_profiler.profiler_start_time = kernel_get_tick();
    g_profiler_initialized = true;
    
    printf("[CPU-PROFILER] Initialized (Sample Interval: %u ms)\n", 
           g_profiler.sample_interval_ms);
}

void cpu_profiler_start(void) {
    if (!g_profiler_initialized) {
        cpu_profiler_init();
    }
    g_profiler.enabled = true;
    g_profiler.profiler_start_time = kernel_get_tick();
    g_profiler.last_sample_time = 0;
    printf("[CPU-PROFILER] Started\n");
}

void cpu_profiler_stop(void) {
    g_profiler.enabled = false;
    g_profiler.profiler_duration_ms = kernel_get_tick() - g_profiler.profiler_start_time;
    printf("[CPU-PROFILER] Stopped (Duration: %u ms)\n", g_profiler.profiler_duration_ms);
}

void cpu_profiler_update(void) {
    if (!g_profiler_initialized || !g_profiler.enabled) {
        return;
    }
    
    uint32_t current_time = kernel_get_tick();
    if (current_time - g_profiler.last_sample_time < g_profiler.sample_interval_ms) {
        return;
    }
    
    g_profiler.last_sample_time = current_time;
    g_profiler.total_samples++;
    g_profiler.total_cpu_usage = 0.0f;
    
    /* Update statistics for each task */
    for (uint32_t i = 0; i < CPU_PROFILER_MAX_TASKS; i++) {
        TaskCpuStats *stats = &g_profiler.tasks[i];
        if (stats->task_id == 0) {
            continue;
        }
        
        /* Get current task state */
        stats->state = task_get_state(stats->task_id);
        stats->priority = task_get_priority(stats->task_id);
        
        /* Get CPU usage from kernel */
        stats->cpu_usage_percent = task_get_cpu_usage(stats->task_id);
        stats->context_switches = task_get_context_switches(stats->task_id);
        
        /* Update min/max/avg */
        if (stats->cpu_usage_percent > stats->peak_cpu_usage) {
            stats->peak_cpu_usage = stats->cpu_usage_percent;
        }
        
        /* Track execution time */
        uint32_t current_exec_time = stats->total_execution_time;
        (void)current_exec_time;  /* Placeholder for actual measurement */
        
        stats->sample_count++;
        
        /* Add to total CPU usage */
        g_profiler.total_cpu_usage += stats->cpu_usage_percent;
    }
    
    /* Calculate idle CPU usage */
    g_profiler.idle_cpu_usage = 100.0f - g_profiler.total_cpu_usage;
    if (g_profiler.idle_cpu_usage < 0) {
        g_profiler.idle_cpu_usage = 0;
    }
    
    /* Store history sample */
    if (g_profiler.history_count < CPU_PROFILER_HISTORY_SIZE) {
        CpuSample *sample = &g_profiler.history[g_profiler.history_count];
        sample->timestamp = current_time;
        sample->cpu_usage = g_profiler.total_cpu_usage;
        g_profiler.history_count++;
    } else {
        /* Rotate history */
        for (uint32_t i = 0; i < CPU_PROFILER_HISTORY_SIZE - 1; i++) {
            g_profiler.history[i] = g_profiler.history[i + 1];
        }
        CpuSample *sample = &g_profiler.history[CPU_PROFILER_HISTORY_SIZE - 1];
        sample->timestamp = current_time;
        sample->cpu_usage = g_profiler.total_cpu_usage;
    }
}

void cpu_profiler_reset(void) {
    memset(&g_profiler, 0, sizeof(CpuProfiler));
    g_profiler.enabled = true;
    g_profiler.sample_interval_ms = 100;
    g_profiler.profiler_start_time = kernel_get_tick();
    printf("[CPU-PROFILER] Reset\n");
}

/* ============================================================================
 * Task Registration
 * ============================================================================ */

bool cpu_profiler_register_task(uint32_t task_id, const char *name, uint32_t priority) {
    if (!g_profiler_initialized) {
        cpu_profiler_init();
    }
    
    for (uint32_t i = 0; i < CPU_PROFILER_MAX_TASKS; i++) {
        if (g_profiler.tasks[i].task_id == 0) {
            g_profiler.tasks[i].task_id = task_id;
            strncpy(g_profiler.tasks[i].task_name, name, sizeof(g_profiler.tasks[i].task_name) - 1);
            g_profiler.tasks[i].priority = priority;
            g_profiler.tasks[i].sample_count = 0;
            g_profiler.tasks[i].cpu_usage_percent = 0.0f;
            g_profiler.tasks[i].peak_cpu_usage = 0.0f;
            g_profiler.tasks[i].min_execution_time = 0xFFFFFFFF;
            g_profiler.tasks[i].max_execution_time = 0;
            return true;
        }
    }
    return false;
}

bool cpu_profiler_unregister_task(uint32_t task_id) {
    for (uint32_t i = 0; i < CPU_PROFILER_MAX_TASKS; i++) {
        if (g_profiler.tasks[i].task_id == task_id) {
            memset(&g_profiler.tasks[i], 0, sizeof(TaskCpuStats));
            return true;
        }
    }
    return false;
}

/* ============================================================================
 * Statistics Queries
 * ============================================================================ */

TaskCpuStats cpu_profiler_get_task_stats(uint32_t task_id) {
    TaskCpuStats empty = {0};
    for (uint32_t i = 0; i < CPU_PROFILER_MAX_TASKS; i++) {
        if (g_profiler.tasks[i].task_id == task_id) {
            return g_profiler.tasks[i];
        }
    }
    return empty;
}

float cpu_profiler_get_total_cpu_usage(void) {
    return g_profiler.total_cpu_usage;
}

float cpu_profiler_get_idle_cpu_usage(void) {
    return g_profiler.idle_cpu_usage;
}

float cpu_profiler_get_system_cpu_usage(void) {
    return g_profiler.system_cpu_usage;
}

uint32_t cpu_profiler_get_sample_count(void) {
    return g_profiler.total_samples;
}

/* ============================================================================
 * History
 * ============================================================================ */

bool cpu_profiler_get_history(uint32_t start_index, uint32_t count, CpuSample *samples) {
    if (!samples || start_index >= g_profiler.history_count) {
        return false;
    }
    
    uint32_t available = g_profiler.history_count - start_index;
    uint32_t copy_count = count < available ? count : available;
    
    for (uint32_t i = 0; i < copy_count; i++) {
        samples[i] = g_profiler.history[start_index + i];
    }
    
    return true;
}

uint32_t cpu_profiler_get_history_count(void) {
    return g_profiler.history_count;
}

/* ============================================================================
 * Reporting
 * ============================================================================ */

void cpu_profiler_print_stats(void) {
    if (!g_profiler_initialized) {
        return;
    }

    printf("\nAeroRTOS CPU profiler\n");
    printf("Samples:    %u\n", g_profiler.total_samples);
    printf("Total CPU:  %.1f%%\n", g_profiler.total_cpu_usage);
    printf("Idle CPU:   %.1f%%\n", g_profiler.idle_cpu_usage);
    printf("System CPU: %.1f%%\n", g_profiler.system_cpu_usage);
    printf("Duration:   %u ms\n", g_profiler.profiler_duration_ms);

    printf("\nTasks:\n");
    printf("  %-6s %-18s %-9s %-9s %-10s %-9s\n", "ID", "Name", "CPU", "Peak", "Switches", "Misses");
    for (uint32_t i = 0; i < CPU_PROFILER_MAX_TASKS; i++) {
        TaskCpuStats *stats = &g_profiler.tasks[i];
        if (stats->task_id == 0) {
            continue;
        }
        printf("  %-6u %-18s %8.1f%% %8.1f%% %-10u %-9u\n",
               stats->task_id,
               stats->task_name,
               stats->cpu_usage_percent,
               stats->peak_cpu_usage,
               stats->context_switches,
               stats->deadline_misses);
    }
}

void cpu_profiler_print_task_stats(uint32_t task_id) {
    TaskCpuStats stats = cpu_profiler_get_task_stats(task_id);
    if (stats.task_id == 0) {
        printf("[CPU-PROFILER] Task %u not found\n", task_id);
        return;
    }
    
    printf("\n=== Task CPU Statistics: %s (ID: %u) ===\n", stats.task_name, stats.task_id);
    printf("  CPU Usage:      %.1f%% (Peak: %.1f%%)\n", 
           stats.cpu_usage_percent, stats.peak_cpu_usage);
    printf("  Context Switches: %u\n", stats.context_switches);
    printf("  Deadline Misses:  %u\n", stats.deadline_misses);
    printf("  Priority:        %u\n", stats.priority);
    printf("  State:           %d\n", stats.state);
    printf("  Samples:         %u\n", stats.sample_count);
}

void cpu_profiler_print_summary(void) {
    if (!g_profiler_initialized) {
        return;
    }
    
    printf("\n=== CPU Profiler Summary ===\n");
    printf("Total Samples: %u\n", g_profiler.total_samples);
    printf("Active Tasks:  %u\n", kernel_get_active_task_count());
    printf("CPU Usage:\n");
    printf("  Total:  %.1f%%\n", g_profiler.total_cpu_usage);
    printf("  Idle:   %.1f%%\n", g_profiler.idle_cpu_usage);
    printf("  System: %.1f%%\n", g_profiler.system_cpu_usage);
    
    /* Find highest CPU task */
    float max_cpu = 0;
    char max_name[32] = "None";
    for (uint32_t i = 0; i < CPU_PROFILER_MAX_TASKS; i++) {
        if (g_profiler.tasks[i].cpu_usage_percent > max_cpu) {
            max_cpu = g_profiler.tasks[i].cpu_usage_percent;
            strncpy(max_name, g_profiler.tasks[i].task_name, sizeof(max_name) - 1);
        }
    }
    printf("  Highest CPU: %s (%.1f%%)\n", max_name, max_cpu);
}

void cpu_profiler_generate_report(void) {
    if (!g_profiler_initialized) {
        return;
    }
    
    printf("\n");
    
    cpu_profiler_print_summary();
    cpu_profiler_print_stats();
    
    /* Print optimization suggestions */
    char suggestions[512];
    cpu_profiler_get_suggestions(suggestions, sizeof(suggestions));
    printf("\n=== Optimization Suggestions ===\n%s\n", suggestions);
}

/* ============================================================================
 * Optimization Suggestions
 * ============================================================================ */

void cpu_profiler_get_suggestions(char *buffer, uint32_t buffer_size) {
    if (!buffer || buffer_size == 0) {
        return;
    }

    char *ptr = buffer;
    uint32_t remaining = buffer_size;
    int written = 0;

    if (g_profiler.total_cpu_usage > 80.0f) {
        written = snprintf(ptr, remaining,
                           "WARNING: High CPU usage (%.1f%%). Consider:\n"
                           "  - Increase task sleep intervals\n"
                           "  - Optimize critical sections\n"
                           "  - Reduce task priorities if possible\n",
                           g_profiler.total_cpu_usage);
        if (written > 0 && (uint32_t)written < remaining) {
            ptr += written;
            remaining -= (uint32_t)written;
        }
    }

    if (g_profiler.idle_cpu_usage < 10.0f && remaining > 0) {
        written = snprintf(ptr, remaining,
                           "WARNING: Low idle CPU (%.1f%%). System may be overloaded. Consider:\n"
                           "  - Check for CPU-bound tasks\n"
                           "  - Consider using DMA for I/O\n"
                           "  - Optimize algorithms\n",
                           g_profiler.idle_cpu_usage);
        if (written > 0 && (uint32_t)written < remaining) {
            ptr += written;
            remaining -= (uint32_t)written;
        }
    }

    for (uint32_t i = 0; i < CPU_PROFILER_MAX_TASKS && remaining > 0; i++) {
        if (g_profiler.tasks[i].cpu_usage_percent > 30.0f) {
            written = snprintf(ptr, remaining,
                               "Task '%s' uses %.1f%% CPU. Consider:\n"
                               "  - Reduce sampling rate\n"
                               "  - Optimize processing algorithm\n"
                               "  - Move to lower priority\n",
                               g_profiler.tasks[i].task_name,
                               g_profiler.tasks[i].cpu_usage_percent);
            if (written > 0 && (uint32_t)written < remaining) {
                ptr += written;
                remaining -= (uint32_t)written;
            }
        }
    }

    uint32_t total_deadline_misses = 0;
    for (uint32_t i = 0; i < CPU_PROFILER_MAX_TASKS; i++) {
        total_deadline_misses += g_profiler.tasks[i].deadline_misses;
    }
    if (total_deadline_misses > 0 && remaining > 0) {
        written = snprintf(ptr, remaining,
                           "%u deadline misses detected. Consider:\n"
                           "  - Increase deadline values\n"
                           "  - Optimize task execution\n"
                           "  - Use DMA or hardware acceleration\n",
                           total_deadline_misses);
        if (written > 0 && (uint32_t)written < remaining) {
            ptr += written;
            remaining -= (uint32_t)written;
        }
    }

    if (ptr == buffer) {
        snprintf(buffer, buffer_size, "No optimization suggestions. System performance is good.\n");
    }
}

bool cpu_profiler_identify_bottlenecks(uint32_t *task_ids, uint32_t *count) {
    if (!task_ids || !count) {
        return false;
    }
    
    *count = 0;
    
    /* Find tasks with high CPU usage or deadline misses */
    for (uint32_t i = 0; i < CPU_PROFILER_MAX_TASKS; i++) {
        if (g_profiler.tasks[i].task_id == 0) {
            continue;
        }
        
        bool is_bottleneck = false;
        
        /* High CPU usage */
        if (g_profiler.tasks[i].cpu_usage_percent > 25.0f) {
            is_bottleneck = true;
        }
        
        /* Deadline misses */
        if (g_profiler.tasks[i].deadline_misses > 0) {
            is_bottleneck = true;
        }
        
        if (is_bottleneck && *count < 32) {
            task_ids[*count] = g_profiler.tasks[i].task_id;
            (*count)++;
        }
    }
    
    return *count > 0;
}

/* ============================================================================
 * Memory Profiler - Stack and Memory Tracking
 * ============================================================================ */

typedef struct {
    uint32_t task_id;
    char task_name[32];
    uint32_t stack_size;
    uint32_t stack_used;
    uint32_t stack_peak;
    uint32_t heap_used;
    uint32_t heap_peak;
    uint32_t allocation_count;
    uint32_t free_count;
} MemoryStats;

static MemoryStats g_memory_stats[CPU_PROFILER_MAX_TASKS];
static uint32_t g_memory_task_count = 0;

void memory_profiler_init(void) {
    memset(g_memory_stats, 0, sizeof(g_memory_stats));
    g_memory_task_count = 0;
    printf("[MEMORY-PROFILER] Initialized\n");
}

bool memory_profiler_register_task(uint32_t task_id, const char *name, uint32_t stack_size) {
    for (uint32_t i = 0; i < CPU_PROFILER_MAX_TASKS; i++) {
        if (g_memory_stats[i].task_id == 0) {
            g_memory_stats[i].task_id = task_id;
            strncpy(g_memory_stats[i].task_name, name, sizeof(g_memory_stats[i].task_name) - 1);
            g_memory_stats[i].stack_size = stack_size;
            g_memory_stats[i].stack_used = 0;
            g_memory_stats[i].stack_peak = 0;
            g_memory_task_count++;
            return true;
        }
    }
    return false;
}

void memory_profiler_update_task(uint32_t task_id) {
    for (uint32_t i = 0; i < CPU_PROFILER_MAX_TASKS; i++) {
        if (g_memory_stats[i].task_id == task_id) {
            g_memory_stats[i].stack_used = task_get_stack_usage(task_id);
            if (g_memory_stats[i].stack_used > g_memory_stats[i].stack_peak) {
                g_memory_stats[i].stack_peak = g_memory_stats[i].stack_used;
            }
            break;
        }
    }
}

void memory_profiler_print_stats(void) {
    printf("\nAeroRTOS memory profiler\n");
    printf("  %-6s %-18s %-10s %-10s %-10s\n", "ID", "Name", "Stack", "Used", "Peak");

    for (uint32_t i = 0; i < CPU_PROFILER_MAX_TASKS; i++) {
        if (g_memory_stats[i].task_id == 0) {
            continue;
        }
        printf("  %-6u %-18s %-10u %-10u %-10u\n",
               g_memory_stats[i].task_id,
               g_memory_stats[i].task_name,
               g_memory_stats[i].stack_size,
               g_memory_stats[i].stack_used,
               g_memory_stats[i].stack_peak);
    }

    for (uint32_t i = 0; i < CPU_PROFILER_MAX_TASKS; i++) {
        if (g_memory_stats[i].task_id == 0 || g_memory_stats[i].stack_size == 0) {
            continue;
        }
        float usage_percent = (float)g_memory_stats[i].stack_peak /
                              (float)g_memory_stats[i].stack_size * 100.0f;
        if (usage_percent > 80.0f) {
            printf("WARNING: Task '%s' stack usage is %.1f%% (Peak: %u/%u)\n",
                   g_memory_stats[i].task_name,
                   usage_percent,
                   g_memory_stats[i].stack_peak,
                   g_memory_stats[i].stack_size);
        }
    }
    printf("\n");
}
