#include "csv_logger.h"
#include "rtos_kernel.h"
#include <stdbool.h>
#include <stdio.h>

static FILE *g_flight_file = NULL;
static FILE *g_health_file = NULL;
static bool g_csv_initialized = false;

void csv_logger_init(void) {
    if (g_csv_initialized) {
        return;
    }

    g_flight_file = fopen("flight_data.csv", "w");
    if (g_flight_file) {
        fprintf(g_flight_file, "timestamp,roll,pitch,yaw,altitude,pos_x,pos_y,pos_z,motor1,motor2,motor3,motor4\n");
        fflush(g_flight_file);
    } else {
        printf("[CSV] Failed to open flight_data.csv\n");
    }

    g_health_file = fopen("health_data.csv", "w");
    if (g_health_file) {
        fprintf(g_health_file, "timestamp,cpu_usage,battery_voltage,battery_remaining,deadline_misses,task_count\n");
        fflush(g_health_file);
    } else {
        printf("[CSV] Failed to open health_data.csv\n");
    }

    g_csv_initialized = true;
    printf("[CSV] Logging initialized\n");
}

void csv_logger_log_flight(float roll, float pitch, float yaw, float altitude,
                           float pos_x, float pos_y, float pos_z,
                           float motor1, float motor2, float motor3, float motor4) {
    if (!g_flight_file) {
        return;
    }

    fprintf(g_flight_file, "%u,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
            kernel_get_tick(), roll, pitch, yaw, altitude,
            pos_x, pos_y, pos_z, motor1, motor2, motor3, motor4);
    fflush(g_flight_file);
}

void csv_logger_log_health(float cpu_usage, float battery_voltage, float battery_remaining,
                           uint32_t deadline_misses, uint32_t task_count) {
    if (!g_health_file) {
        return;
    }

    fprintf(g_health_file, "%u,%.2f,%.2f,%.2f,%u,%u\n",
            kernel_get_tick(), cpu_usage, battery_voltage, battery_remaining,
            deadline_misses, task_count);
    fflush(g_health_file);
}

void csv_logger_close(void) {
    if (g_flight_file) {
        fclose(g_flight_file);
        g_flight_file = NULL;
    }
    if (g_health_file) {
        fclose(g_health_file);
        g_health_file = NULL;
    }
    g_csv_initialized = false;
    printf("[CSV] Logging closed\n");
}