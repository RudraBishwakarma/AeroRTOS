#include "deadline_analyzer.h"
#include "rtos_kernel.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* ============================================================================
 * Global Analyzer Context
 * ============================================================================ */

static DeadlineAnalyzer g_analyzer = {0};
static bool g_analyzer_initialized = false;

/* ============================================================================
 * Analyzer Management
 * ============================================================================ */

void deadline_analyzer_init(void) {
    if (g_analyzer_initialized) {
        return;
    }
    
    memset(&g_analyzer, 0, sizeof(DeadlineAnalyzer));
    g_analyzer.enabled = true;
    g_analyzer.analysis_interval_ms = 1000;
    g_analyzer.last_analysis_time = 0;
    g_analyzer_initialized = true;
    
    printf("[DEADLINE-ANALYZER] Initialized\n");
}

void deadline_analyzer_start(void) {
    if (!g_analyzer_initialized) {
        deadline_analyzer_init();
    }
    g_analyzer.enabled = true;
    printf("[DEADLINE-ANALYZER] Started\n");
}

void deadline_analyzer_stop(void) {
    g_analyzer.enabled = false;
    printf("[DEADLINE-ANALYZER] Stopped\n");
}

void deadline_analyzer_update(void) {
    if (!g_analyzer_initialized || !g_analyzer.enabled) {
        return;
    }
    
    uint32_t current_time = kernel_get_tick();
    if (current_time - g_analyzer.last_analysis_time < g_analyzer.analysis_interval_ms) {
        return;
    }
    
    g_analyzer.last_analysis_time = current_time;
    
    /* Update statistics and check schedulability */
    float total_utilization = 0.0f;
    g_analyzer.total_deadline_misses = 0;
    g_analyzer.total_executions = 0;
    
    for (uint32_t i = 0; i < DEADLINE_ANALYZER_MAX_TASKS; i++) {
        DeadlineStats *stats = &g_analyzer.tasks[i];
        if (stats->task_id == 0) {
            continue;
        }
        
        /* Calculate utilization (WCET / Period) */
        if (stats->period_ms > 0 && stats->wcet > 0) {
            stats->utilization = (float)stats->wcet / (float)stats->period_ms;
            total_utilization += stats->utilization;
        }
        
        /* Calculate deadline met ratio */
        if (stats->total_executions > 0) {
            stats->deadline_met_ratio = 1.0f - ((float)stats->deadline_misses / 
                                                (float)stats->total_executions);
        }
        
        g_analyzer.total_deadline_misses += stats->deadline_misses;
        g_analyzer.total_executions += stats->total_executions;
    }
    
    g_analyzer.system_utilization = total_utilization;
    
    /* Check if system is schedulable (utilization <= 1.0 for EDF) */
    g_analyzer.schedulable = total_utilization <= 1.0f;
}

void deadline_analyzer_reset(void) {
    memset(&g_analyzer, 0, sizeof(DeadlineAnalyzer));
    g_analyzer.enabled = true;
    g_analyzer.analysis_interval_ms = 1000;
    printf("[DEADLINE-ANALYZER] Reset\n");
}

/* ============================================================================
 * Task Registration
 * ============================================================================ */

bool deadline_analyzer_register_task(uint32_t task_id, const char *name,
                                     uint32_t deadline_ms, uint32_t period_ms,
                                     bool is_critical) {
    if (!g_analyzer_initialized) {
        deadline_analyzer_init();
    }
    
    for (uint32_t i = 0; i < DEADLINE_ANALYZER_MAX_TASKS; i++) {
        if (g_analyzer.tasks[i].task_id == 0) {
            g_analyzer.tasks[i].task_id = task_id;
            strncpy(g_analyzer.tasks[i].task_name, name, 
                    sizeof(g_analyzer.tasks[i].task_name) - 1);
            g_analyzer.tasks[i].deadline_ms = deadline_ms;
            g_analyzer.tasks[i].period_ms = period_ms;
            g_analyzer.tasks[i].is_critical = is_critical;
            g_analyzer.tasks[i].min_execution_time = 0xFFFFFFFF;
            g_analyzer.tasks[i].wcet = 0;
            g_analyzer.tasks[i].bcet = 0xFFFFFFFF;
            g_analyzer.tasks[i].worst_slack = 0xFFFFFFFF;
            return true;
        }
    }
    return false;
}

bool deadline_analyzer_unregister_task(uint32_t task_id) {
    for (uint32_t i = 0; i < DEADLINE_ANALYZER_MAX_TASKS; i++) {
        if (g_analyzer.tasks[i].task_id == task_id) {
            memset(&g_analyzer.tasks[i], 0, sizeof(DeadlineStats));
            return true;
        }
    }
    return false;
}

/* ============================================================================
 * Execution Tracking
 * ============================================================================ */

void deadline_analyzer_track_execution(uint32_t task_id, uint32_t execution_time) {
    if (!g_analyzer_initialized || !g_analyzer.enabled) {
        return;
    }
    
    for (uint32_t i = 0; i < DEADLINE_ANALYZER_MAX_TASKS; i++) {
        DeadlineStats *stats = &g_analyzer.tasks[i];
        if (stats->task_id != task_id) {
            continue;
        }
        
        /* Update min/max */
        if (execution_time < stats->min_execution_time) {
            stats->min_execution_time = execution_time;
        }
        if (execution_time > stats->max_execution_time) {
            stats->max_execution_time = execution_time;
        }
        
        /* Update BCET/WCET */
        if (execution_time < stats->bcet) {
            stats->bcet = execution_time;
        }
        if (execution_time > stats->wcet) {
            stats->wcet = execution_time;
        }
        
        /* Update average */
        stats->avg_execution_time = ((stats->avg_execution_time * stats->total_executions) + 
                                    execution_time) / (stats->total_executions + 1);
        
        stats->total_executions++;
        stats->last_execution_time = execution_time;
        g_analyzer.total_executions++;
        
        /* Store in history */
        if (g_analyzer.history_count < DEADLINE_ANALYZER_HISTORY_SIZE) {
            ExecutionSample *sample = &g_analyzer.history[g_analyzer.history_count];
            sample->timestamp = kernel_get_tick();
            sample->task_id = task_id;
            sample->execution_time = execution_time;
            sample->deadline = stats->deadline_ms;
            sample->period = stats->period_ms;
            g_analyzer.history_count++;
        } else {
            /* Rotate history */
            for (uint32_t j = 0; j < DEADLINE_ANALYZER_HISTORY_SIZE - 1; j++) {
                g_analyzer.history[j] = g_analyzer.history[j + 1];
            }
            ExecutionSample *sample = &g_analyzer.history[DEADLINE_ANALYZER_HISTORY_SIZE - 1];
            sample->timestamp = kernel_get_tick();
            sample->task_id = task_id;
            sample->execution_time = execution_time;
            sample->deadline = stats->deadline_ms;
            sample->period = stats->period_ms;
        }
        break;
    }
}

void deadline_analyzer_track_deadline(uint32_t task_id, bool met, uint32_t slack) {
    if (!g_analyzer_initialized || !g_analyzer.enabled) {
        return;
    }
    
    for (uint32_t i = 0; i < DEADLINE_ANALYZER_MAX_TASKS; i++) {
        DeadlineStats *stats = &g_analyzer.tasks[i];
        if (stats->task_id != task_id) {
            continue;
        }
        
        if (!met) {
            stats->deadline_misses++;
            stats->consecutive_misses++;
            if (stats->consecutive_misses > stats->max_consecutive_misses) {
                stats->max_consecutive_misses = stats->consecutive_misses;
            }
            g_analyzer.total_deadline_misses++;
        } else {
            stats->consecutive_misses = 0;
        }
        
        /* Track slack */
        if (slack < stats->worst_slack) {
            stats->worst_slack = slack;
        }
        stats->avg_slack = ((stats->avg_slack * (stats->total_executions - 1)) + slack) / 
                           stats->total_executions;
        
        /* Update history sample */
        if (g_analyzer.history_count > 0) {
            ExecutionSample *sample = &g_analyzer.history[g_analyzer.history_count - 1];
            sample->deadline_met = met;
            sample->slack_time = slack;
        }
        break;
    }
}

/* ============================================================================
 * Statistics Queries
 * ============================================================================ */

DeadlineStats deadline_analyzer_get_task_stats(uint32_t task_id) {
    DeadlineStats empty = {0};
    for (uint32_t i = 0; i < DEADLINE_ANALYZER_MAX_TASKS; i++) {
        if (g_analyzer.tasks[i].task_id == task_id) {
            return g_analyzer.tasks[i];
        }
    }
    return empty;
}

uint32_t deadline_analyzer_get_total_misses(void) {
    return g_analyzer.total_deadline_misses;
}

uint32_t deadline_analyzer_get_total_executions(void) {
    return g_analyzer.total_executions;
}

float deadline_analyzer_get_system_utilization(void) {
    return g_analyzer.system_utilization;
}

bool deadline_analyzer_is_schedulable(void) {
    return g_analyzer.schedulable;
}

/* ============================================================================
 * WCET Analysis
 * ============================================================================ */

uint32_t deadline_analyzer_get_wcet(uint32_t task_id) {
    for (uint32_t i = 0; i < DEADLINE_ANALYZER_MAX_TASKS; i++) {
        if (g_analyzer.tasks[i].task_id == task_id) {
            return g_analyzer.tasks[i].wcet;
        }
    }
    return 0;
}

uint32_t deadline_analyzer_get_bcet(uint32_t task_id) {
    for (uint32_t i = 0; i < DEADLINE_ANALYZER_MAX_TASKS; i++) {
        if (g_analyzer.tasks[i].task_id == task_id) {
            return g_analyzer.tasks[i].bcet;
        }
    }
    return 0;
}

float deadline_analyzer_get_utilization(uint32_t task_id) {
    for (uint32_t i = 0; i < DEADLINE_ANALYZER_MAX_TASKS; i++) {
        if (g_analyzer.tasks[i].task_id == task_id) {
            return g_analyzer.tasks[i].utilization;
        }
    }
    return 0.0f;
}

/* ============================================================================
 * Prediction
 * ============================================================================ */

bool deadline_analyzer_predict_deadline_miss(uint32_t task_id) {
    DeadlineStats stats = deadline_analyzer_get_task_stats(task_id);
    if (stats.task_id == 0) {
        return false;
    }
    
    /* Predict miss if recent execution time exceeds 80% of deadline */
    if (stats.last_execution_time > (stats.deadline_ms * 0.8f)) {
        return true;
    }
    
    /* Predict miss if consecutive misses are increasing */
    if (stats.consecutive_misses > 3) {
        return true;
    }
    
    /* Predict miss if WCET > deadline */
    if (stats.wcet > stats.deadline_ms) {
        return true;
    }
    
    return false;
}

uint32_t deadline_analyzer_predict_execution_time(uint32_t task_id) {
    DeadlineStats stats = deadline_analyzer_get_task_stats(task_id);
    if (stats.task_id == 0) {
        return 0;
    }
    
    /* Use weighted average with WCET */
    if (stats.total_executions > 10) {
        return (stats.avg_execution_time * 7 + stats.wcet * 3) / 10;
    }
    
    return stats.avg_execution_time;
}

/* ============================================================================
 * Reporting
 * ============================================================================ */

void deadline_analyzer_print_stats(void) {
    if (!g_analyzer_initialized) {
        return;
    }
    
    printf("\n╔═══════════════════════════════════════════════════════════╗\n");
    printf("║           AeroRTOS Deadline Analyzer                    ║\n");
    printf("╚═══════════════════════════════════════════════════════════╝\n");
    printf("  Total Executions: %u\n", g_analyzer.total_executions);
    printf("  Total Misses:     %u\n", g_analyzer.total_deadline_misses);
    printf("  Miss Rate:        %.1f%%\n", 
           g_analyzer.total_executions > 0 ? 
           ((float)g_analyzer.total_deadline_misses / g_analyzer.total_executions * 100.0f) : 0.0f);
    printf("  System Util:      %.1f%%\n", g_analyzer.system_utilization * 100.0f);
    printf("  Schedulable:      %s\n", g_analyzer.schedulable ? "YES" : "NO");
    
    printf("\n  ┌──────┬────────────────────┬──────────┬──────────┬──────────┬──────────┬──────────┐\n");
    printf("  │ ID   │ Name               │ Deadline │ WCET     │ Util %%  │ Misses   │ Critical │\n");
    printf("  ├──────┼────────────────────┼──────────┼──────────┼──────────┼──────────┼──────────┤\n");
    
    for (uint32_t i = 0; i < DEADLINE_ANALYZER_MAX_TASKS; i++) {
        DeadlineStats *stats = &g_analyzer.tasks[i];
        if (stats->task_id == 0) {
            continue;
        }
        printf("  │ %4u │ %-18s │ %8u │ %8u │ %7.1f%% │ %8u │ %8s │\n",
               stats->task_id,
               stats->task_name,
               stats->deadline_ms,
               stats->wcet,
               stats->utilization * 100.0f,
               stats->deadline_misses,
               stats->is_critical ? "YES" : "NO");
    }
    
    printf("  └──────┴────────────────────┴──────────┴──────────┴──────────┴──────────┴──────────┘\n\n");
}

void deadline_analyzer_print_task_stats(uint32_t task_id) {
    DeadlineStats stats = deadline_analyzer_get_task_stats(task_id);
    if (stats.task_id == 0) {
        printf("[DEADLINE-ANALYZER] Task %u not found\n", task_id);
        return;
    }
    
    printf("\n=== Deadline Statistics: %s (ID: %u) ===\n", stats.task_name, stats.task_id);
    printf("  Deadline:       %u ms\n", stats.deadline_ms);
    printf("  Period:         %u ms\n", stats.period_ms);
    printf("  WCET:           %u ms\n", stats.wcet);
    printf("  BCET:           %u ms\n", stats.bcet);
    printf("  Avg Exec:       %u ms\n", stats.avg_execution_time);
    printf("  Utilization:    %.1f%%\n", stats.utilization * 100.0f);
    printf("  Total Exec:     %u\n", stats.total_executions);
    printf("  Deadline Misses: %u\n", stats.deadline_misses);
    printf("  Met Ratio:      %.1f%%\n", stats.deadline_met_ratio * 100.0f);
    printf("  Consecutive:    %u\n", stats.consecutive_misses);
    printf("  Max Consec:     %u\n", stats.max_consecutive_misses);
    printf("  Critical:       %s\n", stats.is_critical ? "YES" : "NO");
    printf("  Predict Miss:   %s\n", deadline_analyzer_predict_deadline_miss(task_id) ? "YES" : "NO");
}

void deadline_analyzer_print_summary(void) {
    if (!g_analyzer_initialized) {
        return;
    }
    
    printf("\n=== Deadline Analyzer Summary ===\n");
    printf("Total Executions:   %u\n", g_analyzer.total_executions);
    printf("Total Misses:       %u\n", g_analyzer.total_deadline_misses);
    printf("Miss Rate:          %.1f%%\n", 
           g_analyzer.total_executions > 0 ? 
           ((float)g_analyzer.total_deadline_misses / g_analyzer.total_executions * 100.0f) : 0.0f);
    printf("System Utilization: %.1f%%\n", g_analyzer.system_utilization * 100.0f);
    printf("System Schedulable: %s\n", g_analyzer.schedulable ? "YES" : "NO");
}

void deadline_analyzer_generate_report(void) {
    if (!g_analyzer_initialized) {
        return;
    }
    
    printf("\n╔═══════════════════════════════════════════════════════════╗\n");
    printf("║           AeroRTOS Deadline Analysis Report             ║\n");
    printf("╚═══════════════════════════════════════════════════════════╝\n");
    printf("\n");
    
    deadline_analyzer_print_summary();
    deadline_analyzer_print_stats();
    
    /* Print recommendations */
    printf("\n=== Recommendations ===\n");
    
    if (!g_analyzer.schedulable) {
        printf("⚠ System is NOT schedulable. Consider:\n");
        printf("  - Increase task deadlines\n");
        printf("  - Reduce task periods\n");
        printf("  - Optimize execution paths\n");
        printf("  - Use multi-core processing\n");
    }
    
    for (uint32_t i = 0; i < DEADLINE_ANALYZER_MAX_TASKS; i++) {
        DeadlineStats *stats = &g_analyzer.tasks[i];
        if (stats->task_id == 0) {
            continue;
        }
        
        if (stats->utilization > 0.5f) {
            printf("⚠ Task '%s' has high utilization (%.1f%%). Consider:\n",
                   stats->task_name, stats->utilization * 100.0f);
            printf("  - Split task into smaller tasks\n");
            printf("  - Use DMA for I/O operations\n");
            printf("  - Optimize algorithm complexity\n");
        }
        
        if (stats->deadline_misses > 10) {
            printf("⚠ Task '%s' has %u deadline misses. Consider:\n",
                   stats->task_name, stats->deadline_misses);
            printf("  - Increase deadline to %u ms\n", (uint32_t)(stats->wcet * 1.5f));
            printf("  - Reduce task frequency\n");
            printf("  - Move to higher priority\n");
        }
    }
    printf("\n");
}