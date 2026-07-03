#ifndef HEALTH_MONITOR_H
#define HEALTH_MONITOR_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Health Monitor Configuration
 * ============================================================================ */

#define HEALTH_MONITOR_MAX_LOGS 100

/* Health Status */
typedef enum {
    HEALTH_STATUS_OK = 0,
    HEALTH_STATUS_WARNING,
    HEALTH_STATUS_ERROR,
    HEALTH_STATUS_CRITICAL
} HealthStatus;

/* Sensor Health */
typedef struct {
    bool imu_healthy;
    bool gps_healthy;
    bool baro_healthy;
    bool mag_healthy;
    bool ekf_healthy;
    uint32_t imu_last_update;
    uint32_t gps_last_update;
    uint32_t baro_last_update;
    uint32_t mag_last_update;
    uint32_t ekf_last_update;
    uint32_t imu_error_count;
    uint32_t gps_error_count;
    uint32_t baro_error_count;
    uint32_t mag_error_count;
} SensorHealth;

/* System Health */
typedef struct {
    bool cpu_healthy;
    bool memory_healthy;
    bool stack_healthy;
    bool communication_healthy;
    float cpu_usage_percent;
    float memory_usage_percent;
    uint32_t stack_usage_peak;
    uint32_t task_count;
    uint32_t deadline_misses;
} SystemHealth;

/* Log Entry */
typedef struct {
    uint32_t timestamp;
    HealthStatus status;
    char module[32];
    char message[128];
    float value;
} HealthLogEntry;

/* Health Monitor Context */
typedef struct {
    SensorHealth sensors;
    SystemHealth system;
    HealthLogEntry logs[HEALTH_MONITOR_MAX_LOGS];
    uint32_t log_count;
    uint32_t error_count;
    uint32_t warning_count;
    HealthStatus overall_status;
    bool initialized;
    uint32_t last_check_time;
    uint32_t check_interval_ms;
} HealthMonitorContext;

/* ============================================================================
 * Function Prototypes
 * ============================================================================ */

/* Health Monitor Management */
void health_monitor_init(void);
void health_monitor_update(void);
void health_monitor_check(void);

/* Sensor Health */
void health_monitor_update_imu(bool healthy, uint32_t last_update);
void health_monitor_update_gps(bool healthy, uint32_t last_update);
void health_monitor_update_baro(bool healthy, uint32_t last_update);
void health_monitor_update_mag(bool healthy, uint32_t last_update);
void health_monitor_update_ekf(bool healthy, uint32_t last_update);
SensorHealth health_monitor_get_sensor_health(void);

/* System Health */
void health_monitor_update_system(float cpu_usage, float memory_usage, 
                                  uint32_t stack_usage, uint32_t task_count);
SystemHealth health_monitor_get_system_health(void);

/* Status Queries */
HealthStatus health_monitor_get_overall_status(void);
const char* health_monitor_get_status_string(HealthStatus status);
bool health_monitor_is_system_healthy(void);
bool health_monitor_is_sensor_healthy(void);

/* Logging */
void health_monitor_log(HealthStatus status, const char *module, 
                        const char *message, float value);
void health_monitor_print_logs(void);
void health_monitor_clear_logs(void);

/* Reporting */
void health_monitor_print_status(void);
void health_monitor_print_sensor_health(void);
void health_monitor_print_system_health(void);
void health_monitor_generate_report(void);

#ifdef __cplusplus
}
#endif

#endif /* HEALTH_MONITOR_H */