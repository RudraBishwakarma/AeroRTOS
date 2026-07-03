#include "health_monitor.h"
#include "rtos_kernel.h"
#include "app_state.h"
#include "watchdog.h"
#include "failsafe_actions.h"
#include <string.h>
#include <stdio.h>
#include <time.h>

/* ============================================================================
 * Global Health Monitor
 * ============================================================================ */

static HealthMonitorContext g_health = {0};
static bool g_health_initialized = false;

/* ============================================================================
 * Status Strings
 * ============================================================================ */

static const char* g_health_status_strings[] = {
    "OK",
    "WARNING",
    "ERROR",
    "CRITICAL"
};

/* ============================================================================
 * Health Monitor Management
 * ============================================================================ */

void health_monitor_init(void) {
    if (g_health_initialized) {
        return;
    }
    
    memset(&g_health, 0, sizeof(HealthMonitorContext));
    g_health.overall_status = HEALTH_STATUS_OK;
    g_health.check_interval_ms = 1000;
    g_health.last_check_time = 0;
    g_health.initialized = true;
    
    /* Initialize sensor health */
    g_health.sensors.imu_healthy = true;
    g_health.sensors.gps_healthy = true;
    g_health.sensors.baro_healthy = true;
    g_health.sensors.mag_healthy = true;
    g_health.sensors.ekf_healthy = true;
    
    /* Initialize system health */
    g_health.system.cpu_healthy = true;
    g_health.system.memory_healthy = true;
    g_health.system.stack_healthy = true;
    g_health.system.communication_healthy = true;
    
    printf("[HEALTH] Health Monitor Initialized\n");
}

void health_monitor_update(void) {
    if (!g_health_initialized) {
        return;
    }
    
    uint32_t current_time = kernel_get_tick();
    if (current_time - g_health.last_check_time < g_health.check_interval_ms) {
        return;
    }
    
    g_health.last_check_time = current_time;
    health_monitor_check();
}

void health_monitor_check(void) {
    /* Check sensor health */
    bool all_sensors_ok = g_health.sensors.imu_healthy &&
                          g_health.sensors.gps_healthy &&
                          g_health.sensors.baro_healthy &&
                          g_health.sensors.mag_healthy &&
                          g_health.sensors.ekf_healthy;
    
    /* Check if any sensor is critical */
    bool critical_sensor = !g_health.sensors.imu_healthy ||
                           !g_health.sensors.ekf_healthy;
    
    /* Determine overall status */
    if (!all_sensors_ok) {
        if (critical_sensor) {
            g_health.overall_status = HEALTH_STATUS_CRITICAL;
            health_monitor_log(HEALTH_STATUS_CRITICAL, "SENSORS", 
                              "Critical sensor failure", 0);
        } else {
            g_health.overall_status = HEALTH_STATUS_ERROR;
            health_monitor_log(HEALTH_STATUS_ERROR, "SENSORS", 
                              "Sensor health degraded", 0);
        }
    } else if (g_health.system.cpu_usage_percent > 80.0f) {
        g_health.overall_status = HEALTH_STATUS_WARNING;
        health_monitor_log(HEALTH_STATUS_WARNING, "CPU", 
                          "High CPU usage", g_health.system.cpu_usage_percent);
    } else if (g_health.system.memory_usage_percent > 80.0f) {
        g_health.overall_status = HEALTH_STATUS_WARNING;
        health_monitor_log(HEALTH_STATUS_WARNING, "MEMORY", 
                          "High memory usage", g_health.system.memory_usage_percent);
    } else if (g_health.system.deadline_misses > 0) {
        g_health.overall_status = HEALTH_STATUS_WARNING;
        health_monitor_log(HEALTH_STATUS_WARNING, "DEADLINE", 
                          "Deadline misses detected", g_health.system.deadline_misses);
    } else {
        g_health.overall_status = HEALTH_STATUS_OK;
    }
}

/* ============================================================================
 * Sensor Health
 * ============================================================================ */

void health_monitor_update_imu(bool healthy, uint32_t last_update) {
    if (!g_health_initialized) return;
    g_health.sensors.imu_healthy = healthy;
    g_health.sensors.imu_last_update = last_update;
    if (!healthy) {
        g_health.sensors.imu_error_count++;
        health_monitor_log(HEALTH_STATUS_ERROR, "IMU", "IMU sensor failed", 0);
    }
}

void health_monitor_update_gps(bool healthy, uint32_t last_update) {
    if (!g_health_initialized) return;
    g_health.sensors.gps_healthy = healthy;
    g_health.sensors.gps_last_update = last_update;
    if (!healthy) {
        g_health.sensors.gps_error_count++;
        health_monitor_log(HEALTH_STATUS_WARNING, "GPS", "GPS sensor failed", 0);
    }
}

void health_monitor_update_baro(bool healthy, uint32_t last_update) {
    if (!g_health_initialized) return;
    g_health.sensors.baro_healthy = healthy;
    g_health.sensors.baro_last_update = last_update;
    if (!healthy) {
        g_health.sensors.baro_error_count++;
        health_monitor_log(HEALTH_STATUS_WARNING, "BARO", "Baro sensor failed", 0);
    }
}

void health_monitor_update_mag(bool healthy, uint32_t last_update) {
    if (!g_health_initialized) return;
    g_health.sensors.mag_healthy = healthy;
    g_health.sensors.mag_last_update = last_update;
    if (!healthy) {
        g_health.sensors.mag_error_count++;
        health_monitor_log(HEALTH_STATUS_WARNING, "MAG", "Mag sensor failed", 0);
    }
}

void health_monitor_update_ekf(bool healthy, uint32_t last_update) {
    if (!g_health_initialized) return;
    g_health.sensors.ekf_healthy = healthy;
    g_health.sensors.ekf_last_update = last_update;
    if (!healthy) {
        health_monitor_log(HEALTH_STATUS_ERROR, "EKF", "EKF unhealthy", 0);
    }
}

SensorHealth health_monitor_get_sensor_health(void) {
    return g_health.sensors;
}

/* ============================================================================
 * System Health
 * ============================================================================ */

void health_monitor_update_system(float cpu_usage, float memory_usage, 
                                  uint32_t stack_usage, uint32_t task_count) {
    if (!g_health_initialized) return;
    
    g_health.system.cpu_usage_percent = cpu_usage;
    g_health.system.memory_usage_percent = memory_usage;
    g_health.system.stack_usage_peak = stack_usage;
    g_health.system.task_count = task_count;
    
    g_health.system.cpu_healthy = cpu_usage < 80.0f;
    g_health.system.memory_healthy = memory_usage < 80.0f;
    g_health.system.stack_healthy = stack_usage < 8000;
    g_health.system.communication_healthy = true;
}

SystemHealth health_monitor_get_system_health(void) {
    return g_health.system;
}

/* ============================================================================
 * Status Queries
 * ============================================================================ */

HealthStatus health_monitor_get_overall_status(void) {
    return g_health.overall_status;
}

const char* health_monitor_get_status_string(HealthStatus status) {
    if (status >= 0 && status < sizeof(g_health_status_strings) / sizeof(g_health_status_strings[0])) {
        return g_health_status_strings[status];
    }
    return "UNKNOWN";
}

bool health_monitor_is_system_healthy(void) {
    return g_health.overall_status <= HEALTH_STATUS_WARNING;
}

bool health_monitor_is_sensor_healthy(void) {
    return g_health.sensors.imu_healthy &&
           g_health.sensors.gps_healthy &&
           g_health.sensors.baro_healthy &&
           g_health.sensors.mag_healthy &&
           g_health.sensors.ekf_healthy;
}

/* ============================================================================
 * Logging
 * ============================================================================ */

void health_monitor_log(HealthStatus status, const char *module, 
                        const char *message, float value) {
    if (!g_health_initialized) return;
    
    if (g_health.log_count >= HEALTH_MONITOR_MAX_LOGS) {
        /* Shift logs */
        for (uint32_t i = 0; i < HEALTH_MONITOR_MAX_LOGS - 1; i++) {
            g_health.logs[i] = g_health.logs[i + 1];
        }
        g_health.log_count = HEALTH_MONITOR_MAX_LOGS - 1;
    }
    
    HealthLogEntry *entry = &g_health.logs[g_health.log_count];
    entry->timestamp = kernel_get_tick();
    entry->status = status;
    strncpy(entry->module, module, sizeof(entry->module) - 1);
    strncpy(entry->message, message, sizeof(entry->message) - 1);
    entry->value = value;
    g_health.log_count++;
    
    if (status >= HEALTH_STATUS_ERROR) {
        g_health.error_count++;
    } else if (status == HEALTH_STATUS_WARNING) {
        g_health.warning_count++;
    }
}

void health_monitor_print_logs(void) {
    if (!g_health_initialized) return;
    
    printf("\n=== Health Monitor Logs ===\n");
    printf("Total Logs: %u (Errors: %u, Warnings: %u)\n\n",
           g_health.log_count, g_health.error_count, g_health.warning_count);
    
    for (uint32_t i = 0; i < g_health.log_count && i < 20; i++) {
        HealthLogEntry *entry = &g_health.logs[i];
        printf("[%u] %s: %s - %s (%.2f)\n",
               entry->timestamp,
               health_monitor_get_status_string(entry->status),
               entry->module, entry->message, entry->value);
    }
}

void health_monitor_clear_logs(void) {
    g_health.log_count = 0;
    g_health.error_count = 0;
    g_health.warning_count = 0;
}

/* ============================================================================
 * Reporting
 * ============================================================================ */

void health_monitor_print_status(void) {
    if (!g_health_initialized) return;
    
    printf("  Overall Status: %s\n", 
           health_monitor_get_status_string(g_health.overall_status));
    printf("  System Healthy: %s\n", 
           health_monitor_is_system_healthy() ? "YES" : "NO");
    printf("  Sensors Healthy: %s\n", 
           health_monitor_is_sensor_healthy() ? "YES" : "NO");
    printf("  Logs: %u (Errors: %u, Warnings: %u)\n",
           g_health.log_count, g_health.error_count, g_health.warning_count);
}

void health_monitor_print_sensor_health(void) {
    if (!g_health_initialized) return;
    
    printf("\n=== Sensor Health ===\n");
    printf("  IMU:  %s (Last: %u ms, Errors: %u)\n",
           g_health.sensors.imu_healthy ? "OK" : "FAIL",
           g_health.sensors.imu_last_update,
           g_health.sensors.imu_error_count);
    printf("  GPS:  %s (Last: %u ms, Errors: %u)\n",
           g_health.sensors.gps_healthy ? "OK" : "FAIL",
           g_health.sensors.gps_last_update,
           g_health.sensors.gps_error_count);
    printf("  BARO: %s (Last: %u ms, Errors: %u)\n",
           g_health.sensors.baro_healthy ? "OK" : "FAIL",
           g_health.sensors.baro_last_update,
           g_health.sensors.baro_error_count);
    printf("  MAG:  %s (Last: %u ms, Errors: %u)\n",
           g_health.sensors.mag_healthy ? "OK" : "FAIL",
           g_health.sensors.mag_last_update,
           g_health.sensors.mag_error_count);
    printf("  EKF:  %s (Last: %u ms)\n",
           g_health.sensors.ekf_healthy ? "OK" : "FAIL",
           g_health.sensors.ekf_last_update);
}

void health_monitor_print_system_health(void) {
    if (!g_health_initialized) return;
    
    printf("\n=== System Health ===\n");
    printf("  CPU Usage:     %.1f%% (%s)\n",
           g_health.system.cpu_usage_percent,
           g_health.system.cpu_healthy ? "OK" : "WARNING");
    printf("  Memory Usage:  %.1f%% (%s)\n",
           g_health.system.memory_usage_percent,
           g_health.system.memory_healthy ? "OK" : "WARNING");
    printf("  Stack Usage:   %u bytes (%s)\n",
           g_health.system.stack_usage_peak,
           g_health.system.stack_healthy ? "OK" : "WARNING");
    printf("  Task Count:    %u\n", g_health.system.task_count);
    printf("  Deadline Misses: %u\n", g_health.system.deadline_misses);
}

void health_monitor_generate_report(void) {
    if (!g_health_initialized) return;
    
    printf("\n");

    health_monitor_print_sensor_health();
    health_monitor_print_system_health();
    health_monitor_print_logs();
    
    printf("\n");
}

