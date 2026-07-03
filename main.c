#include "kernel/rtos_kernel.h"
#include "estimation/ekf.h"
#include "flight_controller/cascaded_pid.h"
#include "flight_controller/motor_mixer.h"
#include "communication/mavlink.h"
#include "tools/csv_logger.h"
#include "communication/mavlink_udp.h"
#include "safety/watchdog.h"
#include "safety/failsafe_actions.h"
#include "safety/health_monitor.h"
#include "profiling/cpu_profiler.h"
#include "profiling/deadline_analyzer.h"
#include "profiling/memory_profiler.h"
#include "app_state.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <windows.h>
#include <math.h>

/* ============================================================================
 * Task IDs
 * ============================================================================ */
static uint32_t sensor_task_id;
static uint32_t estimator_task_id;
static uint32_t control_task_id;
static uint32_t telemetry_task_id;
static uint32_t safety_task_id;
static uint32_t navigation_task_id;
static uint32_t gcs_task_id;
static uint32_t watchdog_task_id;
static uint32_t perf_monitor_task_id;

/* ============================================================================
 * Synchronization
 * ============================================================================ */
static uint32_t sensor_data_sem;
static uint32_t control_update_sem;
static uint32_t navigation_update_sem;
static uint32_t telemetry_sem;

/* ============================================================================
 * Queues
 * ============================================================================ */
static uint32_t imu_data_queue;
static uint32_t gps_data_queue;
static uint32_t baro_data_queue;
static uint32_t telemetry_queue;
static uint32_t command_queue;
static uint32_t health_queue;

/* ============================================================================
 * Global Data
 * ============================================================================ */
volatile bool g_system_running = true;
static uint32_t g_frame_counter = 0;
EKFState g_ekf_state;
static CascadedPID g_pid;
static MotorMixer g_mixer;
static float g_motor_outputs[4];

/* Battery simulation */
static float g_battery_voltage = 12.6f;
static float g_battery_current = 0.0f;
static float g_battery_remaining = 100.0f;

/* ============================================================================
 * Sensor Data Structures
 * ============================================================================ */
typedef struct {
    float accel[3];
    float gyro[3];
    float mag[3];
    uint32_t timestamp;
} IMUData;

typedef struct {
    double lat;
    double lon;
    float alt;
    float velocity[3];
    uint32_t timestamp;
    bool fix;
} GPSData;

typedef struct {
    float pressure;
    float temperature;
    float altitude;
    uint32_t timestamp;
} BaroData;

/* ============================================================================
 * Health Data Structure
 * ============================================================================ */
typedef struct {
    uint32_t timestamp;
    float cpu_usage;
    float memory_usage;
    uint32_t stack_usage;
    uint32_t task_count;
    uint32_t deadline_misses;
    bool ekf_healthy;
    float battery_voltage;
    float battery_remaining;
    uint32_t watchdog_timeouts;
} HealthData;

/* ============================================================================
 * Function Prototypes
 * ============================================================================ */
void safety_task_function(void *params);
void watchdog_monitor_task(void *params);
void performance_monitor_task(void *params);

/* ============================================================================
 * Sensor Task (1kHz)
 * ============================================================================ */
void sensor_task_function(void *params) {
    (void)params;
    printf("[SENSOR] Started (ID: %u)\n", task_get_current_id());
    
    IMUData imu = {0};
    GPSData gps = {0};
    BaroData baro = {0};
    uint32_t counter = 0;
    uint32_t last_gps = 0;
    uint32_t last_baro = 0;
    float time = 0;
    uint32_t start_time, end_time, exec_time;
    
    while (g_system_running) {
        start_time = kernel_get_tick();
        counter++;
        time += 0.001f;
        
        /* Generate IMU data */
        imu.accel[0] = sin(time * 2.0f) * 0.1f + (rand() % 100 - 50) * 0.001f;
        imu.accel[1] = cos(time * 1.5f) * 0.1f + (rand() % 100 - 50) * 0.001f;
        imu.accel[2] = -9.81f + (rand() % 100 - 50) * 0.005f;
        
        imu.gyro[0] = sin(time * 2.0f) * 0.05f + (rand() % 100 - 50) * 0.0001f;
        imu.gyro[1] = cos(time * 1.5f) * 0.05f + (rand() % 100 - 50) * 0.0001f;
        imu.gyro[2] = sin(time * 0.8f) * 0.02f + (rand() % 100 - 50) * 0.0001f;
        imu.mag[0] = 0.3f + (rand() % 100 - 50) * 0.0001f;
        imu.mag[1] = 0.1f + (rand() % 100 - 50) * 0.0001f;
        imu.mag[2] = -0.5f + (rand() % 100 - 50) * 0.0001f;
        imu.timestamp = kernel_get_tick();
        
        queue_send(imu_data_queue, &imu, 0);
        semaphore_give(sensor_data_sem);
        
        /* Generate GPS data (10Hz) */
        if (counter - last_gps >= 100) {
            last_gps = counter;
            gps.lat = 37.7749 + (rand() % 100 - 50) * 1e-6;
            gps.lon = -122.4194 + (rand() % 100 - 50) * 1e-6;
            gps.alt = (rand() % 1000) / 10.0f;
            gps.velocity[0] = (rand() % 100 - 50) * 0.01f;
            gps.velocity[1] = (rand() % 100 - 50) * 0.01f;
            gps.velocity[2] = (rand() % 100 - 50) * 0.01f;
            gps.timestamp = kernel_get_tick();
            gps.fix = (rand() % 100) > 5;
            
            queue_send(gps_data_queue, &gps, 0);
            semaphore_give(sensor_data_sem);
        }
        
        /* Generate Baro data (50Hz) */
        if (counter - last_baro >= 20) {
            last_baro = counter;
            baro.pressure = 101325.0f + (rand() % 100 - 50) * 0.1f;
            baro.temperature = 25.0f + (rand() % 100 - 50) * 0.01f;
            baro.timestamp = kernel_get_tick();
            
            queue_send(baro_data_queue, &baro, 0);
            semaphore_give(sensor_data_sem);
        }
        
        /* Kick watchdog every 100 cycles */
        if (counter % 100 == 0) {
            watchdog_kick_current();
        }
        
        if (counter % 1000 == 0) {
            printf("[SENSOR] %u samples sent\n", counter);
        }
        
        /* Track execution time for profiler */
        end_time = kernel_get_tick();
        exec_time = end_time - start_time;
        deadline_analyzer_track_execution(task_get_current_id(), exec_time);
        
        task_sleep(1);
    }
}

/* ============================================================================
 * Estimator Task (500Hz)
 * ============================================================================ */
void estimator_task_function(void *params) {
    (void)params;
    printf("[ESTIMATOR] Started (ID: %u)\n", task_get_current_id());
    
    IMUData imu;
    GPSData gps;
    BaroData baro;
    uint32_t counter = 0;
    float dt = 0.002f;
    bool imu_received = false;
    bool gps_received = false;
    bool baro_received = false;
    uint32_t start_time, end_time, exec_time;
    
    while (g_system_running) {
        start_time = kernel_get_tick();
        
        if (semaphore_take(sensor_data_sem, 10) == KERNEL_SUCCESS) {
            /* Process IMU data (highest priority) */
            imu_received = false;
            while (queue_receive(imu_data_queue, &imu, 0) == KERNEL_SUCCESS) {
                ekf_predict(&g_ekf_state, imu.gyro, imu.accel, dt);
                counter++;
                imu_received = true;
            }
            
            /* Process GPS data */
            gps_received = false;
            while (queue_receive(gps_data_queue, &gps, 0) == KERNEL_SUCCESS) {
                ekf_update_gps(&g_ekf_state, gps.lat, gps.lon, gps.alt);
                gps_received = true;
            }
            
            /* Process Baro data */
            baro_received = false;
            while (queue_receive(baro_data_queue, &baro, 0) == KERNEL_SUCCESS) {
                ekf_update_baro(&g_ekf_state, baro.pressure, baro.temperature);
                baro_received = true;
            }
            
            /* Update health monitor with sensor status */
            health_monitor_update_imu(imu_received, kernel_get_tick());
            health_monitor_update_gps(gps_received, kernel_get_tick());
            health_monitor_update_baro(baro_received, kernel_get_tick());
            health_monitor_update_mag(true, kernel_get_tick());
            health_monitor_update_ekf(ekf_is_healthy(&g_ekf_state), kernel_get_tick());
            
            semaphore_give(control_update_sem);
            
            if (counter % 10 == 0) {
                semaphore_give(navigation_update_sem);
            }
            
            if (counter % 1000 == 0) {
                float roll, pitch, yaw;
                ekf_get_attitude(&g_ekf_state, &roll, &pitch, &yaw);
                printf("[ESTIMATOR] Att: %.2f %.2f %.2f | Alt: %.2fm\n",
                       roll * 57.2958f, pitch * 57.2958f, yaw * 57.2958f,
                       g_ekf_state.x[12]);
            }
            
            /* Kick watchdog */
            watchdog_kick_current();
        }
        
        /* Track execution time for profiler */
        end_time = kernel_get_tick();
        exec_time = end_time - start_time;
        deadline_analyzer_track_execution(task_get_current_id(), exec_time);
        
        task_sleep(2);
    }
}

/* ============================================================================
 * Control Task (200Hz)
 * ============================================================================ */
void control_task_function(void *params) {
    (void)params;
    printf("[CONTROL] Started (ID: %u)\n", task_get_current_id());
    
    float roll, pitch, yaw, alt;
    float target_roll = 0.0f;
    float target_pitch = 0.0f;
    float target_yaw = 0.0f;
    float target_alt = 10.0f;
    float velocity = 0.0f;
    float dt = 0.005f;
    uint32_t counter = 0;
    uint32_t start_time, end_time, exec_time;
    bool deadline_met;
    
    while (g_system_running) {
        start_time = kernel_get_tick();
        
        if (semaphore_take(control_update_sem, 5) == KERNEL_SUCCESS) {
            counter++;
            g_frame_counter++;
            
            /* Get current state from EKF */
            ekf_get_attitude(&g_ekf_state, &roll, &pitch, &yaw);
            ekf_get_position(&g_ekf_state, NULL, NULL, &alt);
            ekf_get_velocity(&g_ekf_state, NULL, NULL, &velocity);
            
            /* Mission: Takeoff to 10m */
            if (counter < 1000) {
                target_alt = 10.0f * (float)counter / 1000.0f;
            } else {
                target_alt = 10.0f;
            }
            
            /* Check if failsafe is active - override control */
            if (failsafe_actions_is_active()) {
                static float failsafe_throttle = 0.5f;
                failsafe_throttle -= 0.001f;
                if (failsafe_throttle < 0.1f) {
                    failsafe_throttle = 0.1f;
                }
                
                for (int i = 0; i < 4; i++) {
                    g_motor_outputs[i] = failsafe_throttle;
                }
            } else {
                /* Normal control */
                cascaded_pid_update(&g_pid,
                                   target_roll, target_pitch, target_yaw,
                                   target_alt, 0.0f,
                                   roll, pitch, yaw,
                                   alt, velocity,
                                   g_motor_outputs, dt);
                
                float roll_out = g_motor_outputs[0] * 0.5f;
                float pitch_out = g_motor_outputs[1] * 0.5f;
                float yaw_out = g_motor_outputs[2] * 0.5f;
                float throttle = g_motor_outputs[3];
                
                motor_mixer_update(&g_mixer, g_motor_outputs,
                                  roll_out, pitch_out, yaw_out, throttle);
            }
            
            /* Send telemetry */
            float telemetry_data[8] = {
                roll, pitch, yaw, alt,
                g_motor_outputs[0], g_motor_outputs[1],
                g_motor_outputs[2], g_motor_outputs[3]
            };
            queue_send(telemetry_queue, telemetry_data, 0);
            semaphore_give(telemetry_sem);
            
            if (counter % 200 == 0) {
                printf("[CONTROL] Alt: %.2f | Motors: %.2f %.2f %.2f %.2f\n",
                       alt,
                       g_motor_outputs[0], g_motor_outputs[1],
                       g_motor_outputs[2], g_motor_outputs[3]);
            }
            
            /* Kick watchdog */
            watchdog_kick_current();
        }
        
        /* Track execution time and deadline */
        end_time = kernel_get_tick();
        exec_time = end_time - start_time;
        deadline_analyzer_track_execution(task_get_current_id(), exec_time);
        
        deadline_met = (exec_time <= 5); /* 5ms deadline */
        deadline_analyzer_track_deadline(task_get_current_id(), deadline_met, 
                                         5 - exec_time);
        
        task_sleep(5);
    }
}

/* ============================================================================
 * MAVLink Telemetry Task (10Hz)
 * ============================================================================ */
void telemetry_task_function(void *params) {
    (void)params;
    printf("[TELEMETRY] Started (ID: %u)\n", task_get_current_id());
    
    uint32_t counter = 0;
    uint32_t current_time = 0;
    uint32_t start_time, end_time, exec_time;
    
    gcs_command_init();
    
    mavlink_telemetry_init(1, 1);
    
    while (g_system_running) {
        start_time = kernel_get_tick();
        counter++;
        current_time = kernel_get_tick();
        
        float roll, pitch, yaw, alt;
        float vx, vy, vz;
        float pos_x, pos_y, pos_z;
        int32_t lat = 37774900;
        int32_t lon = -122419400;
        
        ekf_get_attitude(&g_ekf_state, &roll, &pitch, &yaw);
        ekf_get_position(&g_ekf_state, &pos_x, &pos_y, &pos_z);
        ekf_get_velocity(&g_ekf_state, &vx, &vy, &vz);
        alt = pos_z;
        
        mavlink_telemetry_update(current_time,
                                 roll, pitch, yaw,
                                 0.0f, 0.0f, 0.0f,
                                 lat, lon, (int32_t)(alt * 1000.0f),
                                 (int32_t)(alt * 1000.0f),
                                 (int16_t)(vx * 100.0f),
                                 (int16_t)(vy * 100.0f),
                                 (int16_t)(vz * 100.0f),
                                 0);
        
        if (counter % 100 == 0) {
            printf("[TELEMETRY] MAVLink heartbeat sent over UDP\n");
        }
        
        watchdog_kick_current();
        
        /* Track execution time for profiler */
        end_time = kernel_get_tick();
        exec_time = end_time - start_time;
        deadline_analyzer_track_execution(task_get_current_id(), exec_time);
        
        task_sleep(100);  /* 10Hz */
    }
    
    /* Cleanup UDP */
    mavlink_udp_cleanup();
}

/* ============================================================================
 * GCS Command Task (5Hz)
 * ============================================================================ */
void gcs_task_function(void *params) {
    (void)params;
    printf("[GCS] Started (ID: %u)\n", task_get_current_id());
    
    uint32_t counter = 0;
    uint32_t start_time, end_time, exec_time;
    
    /* Simulate GCS commands */
    float waypoints[][3] = {
        {0.0f, 0.0f, 10.0f},
        {5.0f, 0.0f, 10.0f},
        {5.0f, 5.0f, 10.0f},
        {0.0f, 5.0f, 10.0f},
        {0.0f, 0.0f, 10.0f}
    };
    int num_waypoints = sizeof(waypoints) / sizeof(waypoints[0]);
    int current_waypoint = 0;
    
    while (g_system_running) {
        start_time = kernel_get_tick();
        counter++;
        
        /* Simulate GCS commands */
        if (counter == 100) {
            printf("[GCS] Command: ARM\n");
            float params[1] = {1.0f};
            gcs_process_command(MAVLINK_CMD_COMPONENT_ARM_DISARM, params, 1);
        }
        
        if (counter == 200) {
            printf("[GCS] Command: TAKEOFF to 10m\n");
            float params[7] = {0, 0, 0, 0, 0, 0, 10.0f};
            gcs_process_command(MAVLINK_CMD_NAV_TAKEOFF, params, 7);
        }
        
        if (counter == 500) {
            printf("[GCS] Command: START MISSION\n");
            float params[1] = {0};
            gcs_process_command(MAVLINK_CMD_MISSION_START, params, 1);
        }
        
        if (counter % 200 == 0 && gcs_is_mission_active()) {
            current_waypoint = (current_waypoint + 1) % num_waypoints;
            float progress = (float)current_waypoint / num_waypoints * 100.0f;
            gcs_set_mission_progress(progress);
            printf("[GCS] Mission progress: %.1f%% | Waypoint: %d\n", 
                   progress, current_waypoint);
        }
        
        watchdog_kick_current();
        
        /* Track execution time for profiler */
        end_time = kernel_get_tick();
        exec_time = end_time - start_time;
        deadline_analyzer_track_execution(task_get_current_id(), exec_time);
        
        task_sleep(200);
    }
}

/* ============================================================================
 * Watchdog Monitor Task (10Hz)
 * ============================================================================ */
void watchdog_monitor_task(void *params) {
    (void)params;
    printf("[WATCHDOG-MONITOR] Started (ID: %u)\n", task_get_current_id());
    
    uint32_t counter = 0;
    uint32_t start_time, end_time, exec_time;
    
    while (g_system_running) {
        start_time = kernel_get_tick();
        counter++;
        
        /* Update watchdog */
        watchdog_update();
        
        /* Check global watchdog state */
        WatchdogState global_state = watchdog_get_global_state();
        if (global_state == WATCHDOG_STATE_CRITICAL) {
            printf("[WATCHDOG-MONITOR] CRITICAL: System watchdog triggered!\n");
            failsafe_actions_trigger(FAILSAFE_TRIGGER_WATCHDOG, 
                                    (float)watchdog_get_timeouts(), 
                                    "Watchdog timeout");
        }
        
        /* Print status every 10 cycles */
        if (counter % 10 == 0) {
            printf("[WATCHDOG-MONITOR] Timeouts: %u, Recoveries: %u, State: %s\n",
                   watchdog_get_timeouts(),
                   watchdog_get_recoveries(),
                   watchdog_get_state_string(watchdog_get_global_state()));
        }
        
        /* Kick watchdog */
        watchdog_kick_current();
        
        /* Track execution time for profiler */
        end_time = kernel_get_tick();
        exec_time = end_time - start_time;
        deadline_analyzer_track_execution(task_get_current_id(), exec_time);
        
        task_sleep(100);
    }
}

/* ============================================================================
 * Performance Monitor Task (2Hz)
 * ============================================================================ */
void performance_monitor_task(void *params) {
    (void)params;
    printf("[PERF-MONITOR] Started (ID: %u)\n", task_get_current_id());
    
    uint32_t counter = 0;
    uint32_t start_time, end_time, exec_time;
    
    /* Register with profilers */
    cpu_profiler_register_task(task_get_current_id(), "PerfMonitor", TASK_PRIORITY_LOW);
    deadline_analyzer_register_task(task_get_current_id(), "PerfMonitor", 
                                    1000, 2000, false);
    memory_profiler_register_task(task_get_current_id(), "PerfMonitor", 
                                  AERORTOS_CONFIG_STACK_SIZE_DEFAULT);
    
    while (g_system_running) {
        start_time = kernel_get_tick();
        counter++;
        
        /* Update profilers */
        cpu_profiler_update();
        deadline_analyzer_update();
        
        /* Update memory stats for all tasks */
        for (uint32_t i = 0; i < AERORTOS_CONFIG_MAX_TASKS; i++) {
            TaskControlBlock *task = &g_kernel.task_pool[i];
            if (task->task_id != 0 && task->state != TASK_STATE_TERMINATED) {
                memory_profiler_update_task(task->task_id);
            }
        }
        
        /* Check for performance issues */
        float total_cpu = cpu_profiler_get_total_cpu_usage();
        if (total_cpu > 80.0f) {
            printf("[PERF-MONITOR] WARNING: High CPU usage: %.1f%%\n", total_cpu);
        }
        
        if (!deadline_analyzer_is_schedulable()) {
            printf("[PERF-MONITOR] WARNING: System is NOT schedulable!\n");
            printf("  System Utilization: %.1f%%\n", 
                   deadline_analyzer_get_system_utilization() * 100.0f);
        }
        
        /* Periodic full report */
        if (counter % 30 == 0) {
            printf("\n");
            
            cpu_profiler_print_summary();
            deadline_analyzer_print_summary();
            
            /* Get optimization suggestions */
            char suggestions[512];
            cpu_profiler_get_suggestions(suggestions, sizeof(suggestions));
            printf("\n=== Optimization Suggestions ===\n%s\n", suggestions);
        }
        
        /* Kick watchdog */
        watchdog_kick_current();
        
        /* Track execution time for profiler */
        end_time = kernel_get_tick();
        exec_time = end_time - start_time;
        deadline_analyzer_track_execution(task_get_current_id(), exec_time);
        deadline_analyzer_track_deadline(task_get_current_id(), exec_time <= 500, 
                                         500 - exec_time);
        
        task_sleep(500);  /* 2Hz */
    }
}

/* ============================================================================
 * Safety Task (20Hz)
 * ============================================================================ */
void safety_task_function(void *params) {
    (void)params;
    printf("[SAFETY] Started (ID: %u)\n", task_get_current_id());
    
    /* Register with watchdog */
    watchdog_register_task(task_get_current_id(), "Safety", 2000, true, NULL);
    
    uint32_t counter = 0;
    float altitude;
    float position[3];
    float velocity[3];
    uint32_t start_time, end_time, exec_time;
    
    while (g_system_running) {
        start_time = kernel_get_tick();
        counter++;
        
        /* Kick watchdog */
        watchdog_kick_current();
        
        /* Update health monitor */
        health_monitor_update();
        
        /* Get current state */
        ekf_get_position(&g_ekf_state, &position[0], &position[1], &position[2]);
        ekf_get_velocity(&g_ekf_state, &velocity[0], &velocity[1], &velocity[2]);
        altitude = position[2];
        float speed = sqrtf(velocity[0]*velocity[0] + velocity[1]*velocity[1] + velocity[2]*velocity[2]);
        float roll, pitch, yaw;
        ekf_get_attitude(&g_ekf_state, &roll, &pitch, &yaw);
        
        /* Simulate battery discharge */
        if (counter % 20 == 0) {
            g_battery_voltage -= 0.001f;
            g_battery_current = (rand() % 1000) / 1000.0f * 5.0f;
            g_battery_remaining = ((g_battery_voltage - 9.0f) / 3.6f) * 100.0f;
            if (g_battery_remaining < 0) g_battery_remaining = 0;
        }
        
        /* Check all failsafe conditions */
        failsafe_check_battery(g_battery_voltage, g_battery_current);
        failsafe_check_gps(true, 1.0f, kernel_get_tick());
        failsafe_check_rc(kernel_get_tick());
        failsafe_check_comm(kernel_get_tick());
        failsafe_check_sensors(kernel_get_tick());
        failsafe_check_altitude(altitude);
        failsafe_check_speed(speed);
        failsafe_check_attitude(roll, pitch, yaw);
        failsafe_check_geofence(position[0], position[1], position[2], 100.0f, 150.0f);
        
        /* Update health monitor with system status */
        float cpu_usage = cpu_profiler_get_total_cpu_usage();
        if (cpu_usage == 0) cpu_usage = (float)(rand() % 50) + 20.0f;
        
        float memory_usage = (float)(rand() % 30) + 30.0f;
        uint32_t stack_usage = task_get_stack_usage(task_get_current_id());
        uint32_t active_tasks = kernel_get_active_task_count();
        
        static uint32_t deadline_misses = 0;
        deadline_misses = deadline_analyzer_get_total_misses();
        
        health_monitor_update_system(cpu_usage, memory_usage, stack_usage, active_tasks);
        csv_logger_log_health(cpu_usage, g_battery_voltage, g_battery_remaining,
                              deadline_misses, active_tasks);
        
        /* Send health data to queue */
        HealthData health_data = {
            .timestamp = kernel_get_tick(),
            .cpu_usage = cpu_usage,
            .memory_usage = memory_usage,
            .stack_usage = stack_usage,
            .task_count = active_tasks,
            .deadline_misses = deadline_misses,
            .ekf_healthy = ekf_is_healthy(&g_ekf_state),
            .battery_voltage = g_battery_voltage,
            .battery_remaining = g_battery_remaining,
            .watchdog_timeouts = watchdog_get_timeouts()
        };
        queue_send(health_queue, &health_data, 0);
        
        /* Periodic status report */
        if (counter % 20 == 0) {
            printf("[SAFETY] Status: %s | Battery: %.2fV (%.0f%%) | "
                   "Alt: %.2fm | Speed: %.2fm/s | CPU: %.1f%%\n",
                   health_monitor_get_status_string(health_monitor_get_overall_status()),
                   g_battery_voltage, g_battery_remaining,
                   altitude, speed, cpu_usage);
        }
        
        /* Generate full report every 100 iterations */
        if (counter % 100 == 0) {
            health_monitor_generate_report();
            
            printf("[SAFETY] Failsafe Stats - Triggers: %u, Executions: %u, Recoveries: %u\n",
                   failsafe_get_trigger_count(),
                   failsafe_get_execution_count(),
                   failsafe_get_recovery_count());
        }
        
        /* If system is critical, take action */
        if (health_monitor_get_overall_status() == HEALTH_STATUS_CRITICAL) {
            printf("[SAFETY] CRITICAL: System in critical state!\n");
        }
        
        /* Track execution time for profiler */
        end_time = kernel_get_tick();
        exec_time = end_time - start_time;
        deadline_analyzer_track_execution(task_get_current_id(), exec_time);
        
        task_sleep(50);
    }
}

/* ============================================================================
 * Navigation Task (10Hz)
 * ============================================================================ */
void navigation_task_function(void *params) {
    (void)params;
    printf("[NAVIGATION] Started (ID: %u)\n", task_get_current_id());
    
    float waypoints[][3] = {
        {0.0f, 0.0f, 10.0f},
        {5.0f, 0.0f, 10.0f},
        {5.0f, 5.0f, 10.0f},
        {0.0f, 5.0f, 10.0f},
        {0.0f, 0.0f, 10.0f}
    };
    int num_waypoints = sizeof(waypoints) / sizeof(waypoints[0]);
    int current_waypoint = 0;
    uint32_t counter = 0;
    float position[3];
    float distance_threshold = 0.5f;
    uint32_t start_time, end_time, exec_time;
    
    while (g_system_running) {
        start_time = kernel_get_tick();
        
        if (semaphore_take(navigation_update_sem, 100) == KERNEL_SUCCESS) {
            counter++;
            
            /* Don't navigate if failsafe is active */
            if (failsafe_actions_is_active()) {
                printf("[NAVIGATION] Paused - Failsafe active\n");
                task_sleep(100);
                continue;
            }
            
            ekf_get_position(&g_ekf_state, &position[0], &position[1], &position[2]);
            
            float dx = position[0] - waypoints[current_waypoint][0];
            float dy = position[1] - waypoints[current_waypoint][1];
            float dz = position[2] - waypoints[current_waypoint][2];
            float distance = sqrtf(dx*dx + dy*dy + dz*dz);
            
            if (distance < distance_threshold) {
                current_waypoint++;
                if (current_waypoint >= num_waypoints) {
                    current_waypoint = 0;
                }
                printf("[NAVIGATION] Waypoint %d reached\n", current_waypoint);
            }
            
            if (counter % 10 == 0) {
                printf("[NAVIGATION] Pos: %.1f %.1f %.1f | Target: %.1f %.1f %.1f\n",
                       position[0], position[1], position[2],
                       waypoints[current_waypoint][0],
                       waypoints[current_waypoint][1],
                       waypoints[current_waypoint][2]);
            }
            
            watchdog_kick_current();
        }
        
        /* Track execution time for profiler */
        end_time = kernel_get_tick();
        exec_time = end_time - start_time;
        deadline_analyzer_track_execution(task_get_current_id(), exec_time);
        
        task_sleep(100);
    }
}

/* ============================================================================
 * System Initialization
 * ============================================================================ */
int system_init(void) {
    KernelError err;
    
    printf("\nAeroRTOS system initialization\n");
    printf("Performance profiling, CSV logging, and MAVLink UDP are enabled.\n");
    
    err = kernel_init();
    if (err != KERNEL_SUCCESS) {
        printf("[ERROR] Kernel init failed: %s\n", kernel_get_error_string(err));
        return -1;
    }
    printf("[OK] Kernel initialized\n");
    
    /* Create synchronization primitives */
    semaphore_create("SensorData", 0, 10, false, &sensor_data_sem);
    semaphore_create("ControlUpdate", 0, 1, true, &control_update_sem);
    semaphore_create("NavUpdate", 0, 1, true, &navigation_update_sem);
    semaphore_create("Telemetry", 0, 1, true, &telemetry_sem);
    printf("[OK] Semaphores created\n");
    
    /* Create queues */
    queue_create("IMUData", sizeof(IMUData), 20, false, &imu_data_queue);
    queue_create("GPSData", sizeof(GPSData), 10, false, &gps_data_queue);
    queue_create("BaroData", sizeof(BaroData), 10, false, &baro_data_queue);
    queue_create("Telemetry", sizeof(float[8]), 30, false, &telemetry_queue);
    queue_create("Command", sizeof(float[3]), 10, false, &command_queue);
    queue_create("Health", sizeof(HealthData), 20, false, &health_queue);
    printf("[OK] Message queues created\n");

    /* Initialize Flight Controller */
    ekf_init(&g_ekf_state);
    cascaded_pid_init(&g_pid);
    motor_mixer_init(&g_mixer, MIXER_QUAD_X);
    printf("[OK] Flight Controller initialized\n");

    /* Initialize Safety Systems */
    watchdog_init();
    watchdog_start();
    printf("[OK] Watchdog initialized\n");

    FailsafeSystemConfig failsafe_config;
    failsafe_config_init(&failsafe_config);
    failsafe_actions_init(&failsafe_config);
    printf("[OK] Failsafe system initialized\n");

    health_monitor_init();
    printf("[OK] Health monitor initialized\n");

    if (!mavlink_udp_init("127.0.0.1", 14550)) {
        printf("[WARNING] UDP telemetry initialization failed\n");
    } else {
        printf("[OK] UDP telemetry enabled (127.0.0.1:14550)\n");
    }

    /* Initialize Performance Profilers */
    cpu_profiler_init();
    cpu_profiler_start();
    printf("[OK] CPU profiler initialized\n");

    deadline_analyzer_init();
    deadline_analyzer_start();
    printf("[OK] Deadline analyzer initialized\n");

    memory_profiler_init();
    printf("[OK] Memory profiler initialized\n");

    csv_logger_init();
    printf("[OK] CSV logging enabled\n");

    /* Create Tasks */
    err = task_create("Sensor", sensor_task_function, NULL, 
                      TASK_PRIORITY_CRITICAL, 2048, &sensor_task_id);
    if (err != KERNEL_SUCCESS) return -1;

    err = task_create("Control", control_task_function, NULL, 
                      TASK_PRIORITY_REAL_TIME, 4096, &control_task_id);
    if (err != KERNEL_SUCCESS) return -1;

    err = task_create("Estimator", estimator_task_function, NULL, 
                      TASK_PRIORITY_HIGH, 8192, &estimator_task_id);
    if (err != KERNEL_SUCCESS) return -1;
    
    err = task_create("Safety", safety_task_function, NULL, 
                      TASK_PRIORITY_HIGH, 4096, &safety_task_id);
    if (err != KERNEL_SUCCESS) return -1;
    
    err = task_create("Watchdog", watchdog_monitor_task, NULL, 
                      TASK_PRIORITY_NORMAL, 2048, &watchdog_task_id);
    if (err != KERNEL_SUCCESS) return -1;
    
    err = task_create("PerfMonitor", performance_monitor_task, NULL, 
                      TASK_PRIORITY_LOW, 4096, &perf_monitor_task_id);
    if (err != KERNEL_SUCCESS) return -1;
    
    err = task_create("Navigation", navigation_task_function, NULL, 
                      TASK_PRIORITY_NORMAL, 4096, &navigation_task_id);
    if (err != KERNEL_SUCCESS) return -1;
    
    err = task_create("Telemetry", telemetry_task_function, NULL, 
                      TASK_PRIORITY_LOW, 2048, &telemetry_task_id);
    if (err != KERNEL_SUCCESS) return -1;
    
    err = task_create("GCS", gcs_task_function, NULL, 
                      TASK_PRIORITY_LOW, 2048, &gcs_task_id);
    if (err != KERNEL_SUCCESS) return -1;
    
    /* Register tasks with profilers */
    for (uint32_t i = 0; i < AERORTOS_CONFIG_MAX_TASKS; i++) {
        TaskControlBlock *task = &g_kernel.task_pool[i];
        if (task->task_id != 0 && task->state != TASK_STATE_TERMINATED) {
            cpu_profiler_register_task(task->task_id, task->name, task->priority);
            deadline_analyzer_register_task(task->task_id, task->name, 
                                            task->deadline_ms, task->deadline_ms * 2,
                                            task->priority >= TASK_PRIORITY_CRITICAL);
            memory_profiler_register_task(task->task_id, task->name, task->stack_size);
        }
    }
    printf("[OK] Tasks registered with profilers\n");
    
    /* Register critical tasks with watchdog */
    watchdog_register_task(sensor_task_id, "Sensor", 2000, true, NULL);
    watchdog_register_task(estimator_task_id, "Estimator", 3000, true, NULL);
    watchdog_register_task(control_task_id, "Control", 1000, true, NULL);
    watchdog_register_task(safety_task_id, "Safety", 2000, true, NULL);
    watchdog_register_task(navigation_task_id, "Navigation", 5000, false, NULL);
    watchdog_register_task(telemetry_task_id, "Telemetry", 5000, false, NULL);
    watchdog_register_task(gcs_task_id, "GCS", 5000, false, NULL);
    watchdog_register_task(perf_monitor_task_id, "PerfMonitor", 5000, false, NULL);
    printf("[OK] Tasks registered with watchdog\n");
    
    /* Set deadlines for real-time tasks */
    task_set_deadline(control_task_id, 5, NULL);
    task_set_deadline(estimator_task_id, 2, NULL);
    task_set_deadline(sensor_task_id, 2, NULL);
    
    printf("[OK] %u tasks created\n", kernel_get_task_count());
    printf("      Sensor:     %u (Critical)\n", sensor_task_id);
    printf("      Control:    %u (Real-Time)\n", control_task_id);
    printf("      Estimator:  %u (High)\n", estimator_task_id);
    printf("      Safety:     %u (High)\n", safety_task_id);
    printf("      Watchdog:   %u (Normal)\n", watchdog_task_id);
    printf("      PerfMonitor:%u (Low)\n", perf_monitor_task_id);
    printf("      Navigation: %u (Normal)\n", navigation_task_id);
    printf("      Telemetry:  %u (Low)\n", telemetry_task_id);
    printf("      GCS:        %u (Low)\n", gcs_task_id);
    
    return 0;
}

/* ============================================================================
 * Signal Handler
 * ============================================================================ */
BOOL WINAPI console_handler(DWORD signal) {
    if (signal == CTRL_C_EVENT) {
        printf("\n\n");
        
        g_system_running = false;
        
        /* Stop profilers */
        cpu_profiler_stop();
        deadline_analyzer_stop();
        
        /* Stop watchdog */
        watchdog_stop();
        mavlink_udp_cleanup();
        csv_logger_close();
        
        /* Print final performance report */
        printf("\n");
        cpu_profiler_generate_report();
        
        printf("\n");
        deadline_analyzer_generate_report();
        
        printf("\n");
        memory_profiler_print_stats();
        
        /* Print final health report */
        health_monitor_generate_report();
        
        /* Print final kernel status */
        kernel_print_status();
        
        printf("\nSystem shutdown complete.\n");
        exit(0);
    }
    return TRUE;
}

/* ============================================================================
 * Main
 * ============================================================================ */
int main(void) {
    printf("\nAeroRTOS Flight Controller\n");
    printf("Windows 11 / UCRT64 simulation build\n");
    printf("Features: RTOS scheduler, flight control, safety monitor, CSV logs, MAVLink UDP\n\n");
    
    SetConsoleCtrlHandler(console_handler, TRUE);
    srand((unsigned int)time(NULL));
    
    if (system_init() != 0) {
        printf("\n[FATAL] System initialization failed!\n");
        return -1;
    }
    
    printf("\nStarting RTOS scheduler...\n\n");
    
    KernelError err = kernel_start();
    if (err != KERNEL_SUCCESS) {
        printf("[FATAL] Kernel start failed: %s\n", kernel_get_error_string(err));
        return -1;
    }
    
    return 0;
}
