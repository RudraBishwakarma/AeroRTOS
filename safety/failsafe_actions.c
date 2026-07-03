#include "failsafe_actions.h"
#include "rtos_kernel.h"
#include "app_state.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* ============================================================================
 * Global Failsafe State
 * ============================================================================ */

static FailsafeSystemConfig g_failsafe_config;
static FailsafeState g_failsafe_state;
static bool g_failsafe_initialized = false;

/* Statistics */
static uint32_t g_failsafe_triggers = 0;
static uint32_t g_failsafe_executions = 0;
static uint32_t g_failsafe_recoveries = 0;

/* ============================================================================
 * Failsafe Configuration
 * ============================================================================ */

void failsafe_config_init(FailsafeSystemConfig *config) {
    if (!config) return;
    memset(config, 0, sizeof(FailsafeSystemConfig));
    failsafe_config_set_default(config);
}

void failsafe_config_set_default(FailsafeSystemConfig *config) {
    if (!config) return;
    
    /* Battery failsafe */
    config->battery_low.enabled = true;
    config->battery_low.action = FAILSAFE_ACTION_LAND;
    config->battery_low.delay_ms = 1000;
    config->battery_low.hold_time_ms = 5000;
    config->battery_voltage_min = 10.5f;
    config->battery_voltage_critical = 10.0f;
    
    /* GPS failsafe */
    config->gps_loss.enabled = true;
    config->gps_loss.action = FAILSAFE_ACTION_HOLD;
    config->gps_loss.delay_ms = 5000;
    config->gps_timeout_ms = 10000;
    config->gps_hdop_max = 2.0f;
    
    /* RC failsafe */
    config->rc_loss.enabled = true;
    config->rc_loss.action = FAILSAFE_ACTION_LAND;
    config->rc_loss.delay_ms = 3000;
    config->rc_timeout_ms = 3000;
    
    /* Communication failsafe */
    config->comm_loss.enabled = true;
    config->comm_loss.action = FAILSAFE_ACTION_RTL;
    config->comm_loss.delay_ms = 5000;
    config->comm_timeout_ms = 10000;
    
    /* Sensor failsafe */
    config->sensor_failure.enabled = true;
    config->sensor_failure.action = FAILSAFE_ACTION_LAND;
    config->sensor_failure.delay_ms = 1000;
    config->sensor_timeout_ms = 1000;
    
    /* Altitude limit */
    config->altitude_limit.enabled = true;
    config->altitude_limit.action = FAILSAFE_ACTION_LAND;
    config->altitude_limit.delay_ms = 500;
    config->altitude_max = 100.0f;
    config->altitude_min = 0.0f;
    
    /* Speed limit */
    config->speed_limit.enabled = true;
    config->speed_limit.action = FAILSAFE_ACTION_HOLD;
    config->speed_limit.delay_ms = 1000;
    config->speed_max = 10.0f;
    
    /* Attitude limit */
    config->attitude_limit.enabled = true;
    config->attitude_limit.action = FAILSAFE_ACTION_HOLD;
    config->attitude_limit.delay_ms = 500;
    config->attitude_max = 60.0f;
    
    /* Geo-fence */
    config->geofence.enabled = false;
    config->geofence.action = FAILSAFE_ACTION_RTL;
    config->geofence.delay_ms = 1000;
    config->fence_radius = 100.0f;
    config->fence_altitude = 150.0f;
    
    /* System */
    config->watchdog.enabled = true;
    config->watchdog.action = FAILSAFE_ACTION_LAND;
    config->auto_arm = true;
    config->auto_disarm = true;
    config->disarm_timeout_ms = 10000;
}

void failsafe_config_print(FailsafeSystemConfig *config) {
    if (!config) return;
    
    printf("\n=== Failsafe Configuration ===\n");
    printf("Battery Low:     %s (Action: %d, Voltage: %.1fV)\n",
           config->battery_low.enabled ? "Enabled" : "Disabled",
           config->battery_low.action, config->battery_voltage_min);
    printf("GPS Loss:        %s (Action: %d, Timeout: %u ms)\n",
           config->gps_loss.enabled ? "Enabled" : "Disabled",
           config->gps_loss.action, config->gps_timeout_ms);
    printf("RC Loss:         %s (Action: %d, Timeout: %u ms)\n",
           config->rc_loss.enabled ? "Enabled" : "Disabled",
           config->rc_loss.action, config->rc_timeout_ms);
    printf("Comm Loss:       %s (Action: %d, Timeout: %u ms)\n",
           config->comm_loss.enabled ? "Enabled" : "Disabled",
           config->comm_loss.action, config->comm_timeout_ms);
    printf("Altitude Limit:  %s (Action: %d, Max: %.1fm)\n",
           config->altitude_limit.enabled ? "Enabled" : "Disabled",
           config->altitude_limit.action, config->altitude_max);
    printf("Speed Limit:     %s (Action: %d, Max: %.1fm/s)\n",
           config->speed_limit.enabled ? "Enabled" : "Disabled",
           config->speed_limit.action, config->speed_max);
    printf("Attitude Limit:  %s (Action: %d, Max: %.1f deg)\n",
           config->attitude_limit.enabled ? "Enabled" : "Disabled",
           config->attitude_limit.action, config->attitude_max);
    printf("Geo-fence:       %s (Action: %d, Radius: %.1fm)\n",
           config->geofence.enabled ? "Enabled" : "Disabled",
           config->geofence.action, config->fence_radius);
    printf("Watchdog:        %s (Action: %d)\n",
           config->watchdog.enabled ? "Enabled" : "Disabled",
           config->watchdog.action);
    printf("Auto Arm:        %s\n", config->auto_arm ? "Yes" : "No");
    printf("Auto Disarm:     %s (Timeout: %u ms)\n",
           config->auto_disarm ? "Yes" : "No", config->disarm_timeout_ms);
}

/* ============================================================================
 * Failsafe Management
 * ============================================================================ */

void failsafe_actions_init(FailsafeSystemConfig *config) {
    if (!g_failsafe_initialized) {
        if (config) {
            memcpy(&g_failsafe_config, config, sizeof(FailsafeSystemConfig));
        } else {
            failsafe_config_init(&g_failsafe_config);
        }
        
        memset(&g_failsafe_state, 0, sizeof(FailsafeState));
        g_failsafe_state.recovery_possible = true;
        g_failsafe_initialized = true;
        
        printf("[FAILSAFE] Initialized\n");
        failsafe_config_print(&g_failsafe_config);
    }
}

void failsafe_actions_update(uint32_t current_time_ms) {
    if (!g_failsafe_initialized || !g_failsafe_state.active) {
        return;
    }
    
    /* Check if action duration has elapsed */
    if (g_failsafe_state.action_duration_ms > 0) {
        uint32_t elapsed = current_time_ms - g_failsafe_state.action_start_time;
        if (elapsed >= g_failsafe_state.action_duration_ms) {
            printf("[FAILSAFE] Action duration elapsed (Action: %d, Duration: %u ms)\n",
                   g_failsafe_state.current_action, g_failsafe_state.action_duration_ms);
            g_failsafe_state.executed = true;
        }
    }
}

bool failsafe_actions_trigger(FailsafeTriggerType trigger, float value, const char *description) {
    if (!g_failsafe_initialized) {
        return false;
    }
    
    /* Check if already in failsafe */
    if (g_failsafe_state.active) {
        printf("[FAILSAFE] Already active (Trigger: %d)\n", g_failsafe_state.trigger_type);
        return false;
    }
    
    g_failsafe_triggers++;
    g_failsafe_state.active = true;
    g_failsafe_state.trigger_type = trigger;
    g_failsafe_state.trigger_time = kernel_get_tick();
    g_failsafe_state.action_start_time = kernel_get_tick();
    g_failsafe_state.trigger_value = value;
    
    if (description) {
        strncpy(g_failsafe_state.trigger_description, description, sizeof(g_failsafe_state.trigger_description) - 1);
    } else {
        strcpy(g_failsafe_state.trigger_description, "Unknown trigger");
    }
    
    printf("[FAILSAFE] TRIGGERED: %s (Trigger: %d, Value: %.2f)\n",
           g_failsafe_state.trigger_description, trigger, value);
    
    /* Determine action based on trigger type */
    FailsafeActionType action = FAILSAFE_ACTION_NONE;
    uint32_t delay_ms = 0;
    
    switch (trigger) {
        case FAILSAFE_TRIGGER_BATTERY:
            action = g_failsafe_config.battery_low.action;
            delay_ms = g_failsafe_config.battery_low.delay_ms;
            break;
        case FAILSAFE_TRIGGER_GPS_LOSS:
            action = g_failsafe_config.gps_loss.action;
            delay_ms = g_failsafe_config.gps_loss.delay_ms;
            break;
        case FAILSAFE_TRIGGER_RC_LOSS:
            action = g_failsafe_config.rc_loss.action;
            delay_ms = g_failsafe_config.rc_loss.delay_ms;
            break;
        case FAILSAFE_TRIGGER_COMM_LOSS:
            action = g_failsafe_config.comm_loss.action;
            delay_ms = g_failsafe_config.comm_loss.delay_ms;
            break;
        case FAILSAFE_TRIGGER_SENSOR_FAILURE:
            action = g_failsafe_config.sensor_failure.action;
            delay_ms = g_failsafe_config.sensor_failure.delay_ms;
            break;
        case FAILSAFE_TRIGGER_ALTITUDE_LIMIT:
            action = g_failsafe_config.altitude_limit.action;
            delay_ms = g_failsafe_config.altitude_limit.delay_ms;
            break;
        case FAILSAFE_TRIGGER_SPEED_LIMIT:
            action = g_failsafe_config.speed_limit.action;
            delay_ms = g_failsafe_config.speed_limit.delay_ms;
            break;
        case FAILSAFE_TRIGGER_ATTITUDE_LIMIT:
            action = g_failsafe_config.attitude_limit.action;
            delay_ms = g_failsafe_config.attitude_limit.delay_ms;
            break;
        case FAILSAFE_TRIGGER_WATCHDOG:
            action = g_failsafe_config.watchdog.action;
            delay_ms = g_failsafe_config.watchdog.delay_ms;
            break;
        default:
            action = FAILSAFE_ACTION_LAND;
            delay_ms = 1000;
            break;
    }
    
    g_failsafe_state.current_action = action;
    g_failsafe_state.action_duration_ms = delay_ms;
    
    /* Execute action after delay */
    printf("[FAILSAFE] Executing action: %d (Delay: %u ms)\n", action, delay_ms);
    
    bool result = failsafe_execute_action(action, value, 0, 0);
    if (result) {
        g_failsafe_executions++;
    }
    
    return result;
}

void failsafe_actions_reset(void) {
    if (!g_failsafe_initialized) {
        return;
    }
    
    g_failsafe_state.active = false;
    g_failsafe_state.current_action = FAILSAFE_ACTION_NONE;
    g_failsafe_state.executed = false;
    g_failsafe_state.recovery_possible = true;
    printf("[FAILSAFE] Reset\n");
}

bool failsafe_actions_is_active(void) {
    return g_failsafe_state.active;
}

FailsafeActionType failsafe_actions_get_current_action(void) {
    return g_failsafe_state.current_action;
}

FailsafeTriggerType failsafe_actions_get_trigger(void) {
    return g_failsafe_state.trigger_type;
}

/* ============================================================================
 * Failsafe Action Execution
 * ============================================================================ */

bool failsafe_execute_action(FailsafeActionType action, float param1, float param2, float param3) {
    (void)param1;
    (void)param2;
    (void)param3;
    
    switch (action) {
        case FAILSAFE_ACTION_LAND:
            failsafe_action_land();
            return true;
        case FAILSAFE_ACTION_RTL:
            failsafe_action_rtl();
            return true;
        case FAILSAFE_ACTION_HOLD:
            failsafe_action_hold();
            return true;
        case FAILSAFE_ACTION_DISARM:
            failsafe_action_disarm();
            return true;
        case FAILSAFE_ACTION_PARACHUTE:
            failsafe_action_parachute();
            return true;
        case FAILSAFE_ACTION_CIRCLE:
            failsafe_action_circle();
            return true;
        case FAILSAFE_ACTION_LOITER:
            failsafe_action_loiter();
            return true;
        case FAILSAFE_ACTION_TERMINATE:
            failsafe_action_terminate();
            return true;
        default:
            printf("[FAILSAFE] Unknown action: %d\n", action);
            return false;
    }
}

void failsafe_action_land(void) {
    printf("[FAILSAFE] ACTION: LANDING\n");
    /* In real implementation: Set landing mode, reduce throttle */
}

void failsafe_action_rtl(void) {
    printf("[FAILSAFE] ACTION: RETURN TO LAUNCH\n");
    /* In real implementation: Navigate to home position */
}

void failsafe_action_hold(void) {
    printf("[FAILSAFE] ACTION: HOLD POSITION\n");
    /* In real implementation: Hold current position */
}

void failsafe_action_disarm(void) {
    printf("[FAILSAFE] ACTION: DISARM\n");
    /* In real implementation: Disarm motors */
}

void failsafe_action_parachute(void) {
    printf("[FAILSAFE] ACTION: DEPLOY PARACHUTE\n");
    /* In real implementation: Deploy parachute */
}

void failsafe_action_circle(void) {
    printf("[FAILSAFE] ACTION: CIRCLE\n");
    /* In real implementation: Circle at current position */
}

void failsafe_action_loiter(void) {
    printf("[FAILSAFE] ACTION: LOITER\n");
    /* In real implementation: Loiter in place */
}

void failsafe_action_terminate(void) {
    printf("[FAILSAFE] ACTION: TERMINATE\n");
    /* In real implementation: Emergency stop */
    g_system_running = false;
}

/* ============================================================================
 * Recovery
 * ============================================================================ */

bool failsafe_recover(void) {
    if (!g_failsafe_initialized || !g_failsafe_state.active) {
        return false;
    }
    
    if (!g_failsafe_state.recovery_possible) {
        printf("[FAILSAFE] Recovery not possible\n");
        return false;
    }
    
    printf("[FAILSAFE] Recovery initiated\n");
    g_failsafe_recoveries++;
    failsafe_actions_reset();
    return true;
}

bool failsafe_can_recover(void) {
    return g_failsafe_state.recovery_possible && g_failsafe_state.active;
}

/* ============================================================================
 * Monitoring Functions
 * ============================================================================ */

bool failsafe_check_battery(float voltage, float current) {
    (void)current;
    
    if (!g_failsafe_config.battery_low.enabled) {
        return false;
    }
    
    if (voltage <= g_failsafe_config.battery_voltage_critical) {
        printf("[FAILSAFE] Battery critical: %.2fV\n", voltage);
        failsafe_actions_trigger(FAILSAFE_TRIGGER_BATTERY, voltage, "Battery critical");
        return true;
    } else if (voltage <= g_failsafe_config.battery_voltage_min) {
        printf("[FAILSAFE] Battery low: %.2fV\n", voltage);
        failsafe_actions_trigger(FAILSAFE_TRIGGER_BATTERY, voltage, "Battery low");
        return true;
    }
    
    return false;
}

bool failsafe_check_gps(bool has_fix, float hdop, uint32_t last_update_ms) {
    (void)hdop;
    
    if (!g_failsafe_config.gps_loss.enabled || !has_fix) {
        return false;
    }
    
    uint32_t elapsed = kernel_get_tick() - last_update_ms;
    if (elapsed > g_failsafe_config.gps_timeout_ms) {
        printf("[FAILSAFE] GPS loss: %u ms\n", elapsed);
        failsafe_actions_trigger(FAILSAFE_TRIGGER_GPS_LOSS, (float)elapsed, "GPS timeout");
        return true;
    }
    
    return false;
}

bool failsafe_check_rc(uint32_t last_update_ms) {
    if (!g_failsafe_config.rc_loss.enabled) {
        return false;
    }
    
    uint32_t elapsed = kernel_get_tick() - last_update_ms;
    if (elapsed > g_failsafe_config.rc_timeout_ms) {
        printf("[FAILSAFE] RC loss: %u ms\n", elapsed);
        failsafe_actions_trigger(FAILSAFE_TRIGGER_RC_LOSS, (float)elapsed, "RC timeout");
        return true;
    }
    
    return false;
}

bool failsafe_check_comm(uint32_t last_heartbeat_ms) {
    if (!g_failsafe_config.comm_loss.enabled) {
        return false;
    }
    
    uint32_t elapsed = kernel_get_tick() - last_heartbeat_ms;
    if (elapsed > g_failsafe_config.comm_timeout_ms) {
        printf("[FAILSAFE] Communication loss: %u ms\n", elapsed);
        failsafe_actions_trigger(FAILSAFE_TRIGGER_COMM_LOSS, (float)elapsed, "Comm timeout");
        return true;
    }
    
    return false;
}

bool failsafe_check_sensors(uint32_t last_update_ms) {
    if (!g_failsafe_config.sensor_failure.enabled) {
        return false;
    }
    
    uint32_t elapsed = kernel_get_tick() - last_update_ms;
    if (elapsed > g_failsafe_config.sensor_timeout_ms) {
        printf("[FAILSAFE] Sensor timeout: %u ms\n", elapsed);
        failsafe_actions_trigger(FAILSAFE_TRIGGER_SENSOR_FAILURE, (float)elapsed, "Sensor timeout");
        return true;
    }
    
    return false;
}

bool failsafe_check_altitude(float altitude) {
    if (!g_failsafe_config.altitude_limit.enabled) {
        return false;
    }
    
    if (altitude > g_failsafe_config.altitude_max) {
        printf("[FAILSAFE] Altitude limit exceeded: %.2fm\n", altitude);
        failsafe_actions_trigger(FAILSAFE_TRIGGER_ALTITUDE_LIMIT, altitude, "Altitude limit");
        return true;
    }
    
    if (altitude < g_failsafe_config.altitude_min) {
        printf("[FAILSAFE] Altitude below minimum: %.2fm\n", altitude);
        failsafe_actions_trigger(FAILSAFE_TRIGGER_ALTITUDE_LIMIT, altitude, "Altitude below min");
        return true;
    }
    
    return false;
}

bool failsafe_check_speed(float speed) {
    if (!g_failsafe_config.speed_limit.enabled) {
        return false;
    }
    
    if (speed > g_failsafe_config.speed_max) {
        printf("[FAILSAFE] Speed limit exceeded: %.2fm/s\n", speed);
        failsafe_actions_trigger(FAILSAFE_TRIGGER_SPEED_LIMIT, speed, "Speed limit");
        return true;
    }
    
    return false;
}

bool failsafe_check_attitude(float roll, float pitch, float yaw) {
    (void)yaw;
    
    if (!g_failsafe_config.attitude_limit.enabled) {
        return false;
    }
    
    float roll_deg = roll * 57.2958f;
    float pitch_deg = pitch * 57.2958f;
    
    if (fabsf(roll_deg) > g_failsafe_config.attitude_max ||
        fabsf(pitch_deg) > g_failsafe_config.attitude_max) {
        printf("[FAILSAFE] Attitude limit exceeded: Roll %.2f, Pitch %.2f\n",
               roll_deg, pitch_deg);
        failsafe_actions_trigger(FAILSAFE_TRIGGER_ATTITUDE_LIMIT, 
                                 fmaxf(fabsf(roll_deg), fabsf(pitch_deg)), 
                                 "Attitude limit");
        return true;
    }
    
    return false;
}

bool failsafe_check_geofence(float x, float y, float z, float radius, float altitude) {
    (void)x;
    (void)y;
    (void)z;
    
    if (!g_failsafe_config.geofence.enabled) {
        return false;
    }
    
    float distance = sqrtf(x*x + y*y);
    if (distance > radius || z > altitude) {
        printf("[FAILSAFE] Geofence violation: Dist %.1fm, Alt %.1fm\n", distance, z);
        failsafe_actions_trigger(FAILSAFE_TRIGGER_GE_FENCE, distance, "Geofence violation");
        return true;
    }
    
    return false;
}

/* ============================================================================
 * Statistics
 * ============================================================================ */

uint32_t failsafe_get_trigger_count(void) {
    return g_failsafe_triggers;
}

uint32_t failsafe_get_execution_count(void) {
    return g_failsafe_executions;
}

uint32_t failsafe_get_recovery_count(void) {
    return g_failsafe_recoveries;
}