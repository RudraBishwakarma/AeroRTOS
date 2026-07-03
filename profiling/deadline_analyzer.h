#ifndef DEADLINE_ANALYZER_H
#define DEADLINE_ANALYZER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Deadline Analyzer Configuration
 * ============================================================================ */

#define DEADLINE_ANALYZER_MAX_TASKS 32
#define DEADLINE_ANALYZER_HISTORY_SIZE 100

/* Execution Time Sample */
typedef struct {
    uint32_t timestamp;
    uint32_t task_id;
    uint32_t execution_time;
    uint32_t deadline;
    uint32_t period;
    bool deadline_met;
    uint32_t slack_time;
} ExecutionSample;

/* Task Deadline Statistics */
typedef struct {
    uint32_t task_id;
    char task_name[32];
    uint32_t deadline_ms;
    uint32_t period_ms;
    uint32_t min_execution_time;
    uint32_t max_execution_time;
    uint32_t avg_execution_time;
    uint32_t wcet;  /* Worst-Case Execution Time */
    uint32_t bcet;  /* Best-Case Execution Time */
    uint32_t total_executions;
    uint32_t deadline_misses;
    uint32_t consecutive_misses;
    uint32_t max_consecutive_misses;
    float utilization;
    float deadline_met_ratio;
    bool is_critical;
    uint32_t last_execution_time;
    uint32_t worst_slack;
    uint32_t avg_slack;
} DeadlineStats;

/* Deadline Analyzer Context */
typedef struct {
    DeadlineStats tasks[DEADLINE_ANALYZER_MAX_TASKS];
    ExecutionSample history[DEADLINE_ANALYZER_HISTORY_SIZE];
    uint32_t history_count;
    uint32_t history_index;
    bool enabled;
    uint32_t analysis_interval_ms;
    uint32_t last_analysis_time;
    uint32_t total_deadline_misses;
    uint32_t total_executions;
    float system_utilization;
    bool schedulable;
} DeadlineAnalyzer;

/* ============================================================================
 * Function Prototypes
 * ============================================================================ */

/* Analyzer Management */
void deadline_analyzer_init(void);
void deadline_analyzer_start(void);
void deadline_analyzer_stop(void);
void deadline_analyzer_update(void);
void deadline_analyzer_reset(void);

/* Task Registration */
bool deadline_analyzer_register_task(uint32_t task_id, const char *name,
                                     uint32_t deadline_ms, uint32_t period_ms,
                                     bool is_critical);
bool deadline_analyzer_unregister_task(uint32_t task_id);

/* Execution Tracking */
void deadline_analyzer_track_execution(uint32_t task_id, uint32_t execution_time);
void deadline_analyzer_track_deadline(uint32_t task_id, bool met, uint32_t slack);

/* Statistics Queries */
DeadlineStats deadline_analyzer_get_task_stats(uint32_t task_id);
uint32_t deadline_analyzer_get_total_misses(void);
uint32_t deadline_analyzer_get_total_executions(void);
float deadline_analyzer_get_system_utilization(void);
bool deadline_analyzer_is_schedulable(void);

/* WCET Analysis */
uint32_t deadline_analyzer_get_wcet(uint32_t task_id);
uint32_t deadline_analyzer_get_bcet(uint32_t task_id);
float deadline_analyzer_get_utilization(uint32_t task_id);

/* Reporting */
void deadline_analyzer_print_stats(void);
void deadline_analyzer_print_task_stats(uint32_t task_id);
void deadline_analyzer_print_summary(void);
void deadline_analyzer_generate_report(void);

/* Prediction */
bool deadline_analyzer_predict_deadline_miss(uint32_t task_id);
uint32_t deadline_analyzer_predict_execution_time(uint32_t task_id);

#ifdef __cplusplus
}
#endif

#endif /* DEADLINE_ANALYZER_H */