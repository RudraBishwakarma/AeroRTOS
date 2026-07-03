#ifndef FAILSAFE_ACTIONS_H
#define FAILSAFE_ACTIONS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Failsafe Action Types
 * ============================================================================ */

typedef enum {
    FAILSAFE_ACTION_NONE = 0,
    FAILSAFE_ACTION_LAND,
    FAILSAFE_ACTION_RTL,           /* Return to Launch */
    FAILSAFE_ACTION_HOLD,
    FAILSAFE_ACTION_DISARM,
    FAILSAFE_ACTION_PARACHUTE,
    FAILSAFE_ACTION_GUIDED,        /* Guided mode with GCS control */
    FAILSAFE_ACTION_CIRCLE,
    FAILSAFE_ACTION_LOITER,
    FAILSAFE_ACTION_TERMINATE
} FailsafeActionType;

/* ============================================================================
 * Failsafe Trigger Types
 * ============================================================================ */

typedef enum {
    FAILSAFE_TRIGGER_BATTERY = 0,
    FAILSAFE_TRIGGER_GPS_LOSS,
    FAILSAFE_TRIGGER_RC_LOSS,
    FAILSAFE_TRIGGER_SENSOR_FAILURE,
    FAILSAFE_TRIGGER_COMM_LOSS,
    FAILSAFE_TRIGGER_ALTITUDE_LIMIT,
    FAILSAFE_TRIGGER_SPEED_LIMIT,
    FAILSAFE_TRIGGER_ATTITUDE_LIMIT,
    FAILSAFE_TRIGGER_WATCHDOG,
    FAILSAFE_TRIGGER_GE_FENCE,     /* Geo-fence violation */
    FAILSAFE_TRIGGER_MANUAL        /* Manual trigger */
} FailsafeTriggerType;

/* ============================================================================
 * Failsafe Configuration
 * ============================================================================ */

typedef struct {
    bool enabled;
    FailsafeActionType action;
    uint32_t delay_ms;
    uint32_t hold_time_ms;
    float param1;
    float param2;
    float param3;
} FailsafeConfig;

typedef struct {
    /* Battery failsafe */
    FailsafeConfig battery_low;
    FailsafeConfig battery_critical;
    float battery_voltage_min;
    float battery_voltage_critical;
    
    /* GPS failsafe */
    FailsafeConfig gps_loss;
    uint32_t gps_timeout_ms;
    float gps_hdop_max;
    
    /* RC failsafe */
    FailsafeConfig rc_loss;
    uint32_t rc_timeout_ms;
    
    /* Communication failsafe */
    FailsafeConfig comm_loss;
    uint32_t comm_timeout_ms;
    
    /* Sensor failsafe */
    FailsafeConfig sensor_failure;
    uint32_t sensor_timeout_ms;
    
    /* Altitude limit */
    FailsafeConfig altitude_limit;
    float altitude_max;
    float altitude_min;
    
    /* Speed limit */
    FailsafeConfig speed_limit;
    float speed_max;
    
    /* Attitude limit */
    FailsafeConfig attitude_limit;
    float attitude_max;
    
    /* Geo-fence */
    FailsafeConfig geofence;
    float fence_radius;
    float fence_altitude;
    
    /* System failsafe */
    FailsafeConfig watchdog;
    bool auto_arm;
    bool auto_disarm;
    uint32_t disarm_timeout_ms;
} FailsafeSystemConfig;

/* ============================================================================
 * Failsafe State
 * ============================================================================ */

typedef struct {
    bool active;
    FailsafeActionType current_action;
    FailsafeTriggerType trigger_type;
    uint32_t trigger_time;
    uint32_t action_start_time;
    uint32_t action_duration_ms;
    bool recovery_possible;
    bool executed;
    float trigger_value;
    char trigger_description[64];
} FailsafeState;

/* ============================================================================
 * Function Prototypes
 * ============================================================================ */

/* Failsafe Configuration */
void failsafe_config_init(FailsafeSystemConfig *config);
void failsafe_config_set_default(FailsafeSystemConfig *config);
void failsafe_config_print(FailsafeSystemConfig *config);

/* Failsafe Management */
void failsafe_actions_init(FailsafeSystemConfig *config);
void failsafe_actions_update(uint32_t current_time_ms);
bool failsafe_actions_trigger(FailsafeTriggerType trigger, float value, const char *description);
void failsafe_actions_reset(void);
bool failsafe_actions_is_active(void);
FailsafeActionType failsafe_actions_get_current_action(void);
FailsafeTriggerType failsafe_actions_get_trigger(void);

/* Failsafe Action Execution */
bool failsafe_execute_action(FailsafeActionType action, float param1, float param2, float param3);
void failsafe_action_land(void);
void failsafe_action_rtl(void);
void failsafe_action_hold(void);
void failsafe_action_disarm(void);
void failsafe_action_parachute(void);
void failsafe_action_circle(void);
void failsafe_action_loiter(void);
void failsafe_action_terminate(void);

/* Recovery */
bool failsafe_recover(void);
bool failsafe_can_recover(void);

/* Monitoring */
bool failsafe_check_battery(float voltage, float current);
bool failsafe_check_gps(bool has_fix, float hdop, uint32_t last_update_ms);
bool failsafe_check_rc(uint32_t last_update_ms);
bool failsafe_check_comm(uint32_t last_heartbeat_ms);
bool failsafe_check_sensors(uint32_t last_update_ms);
bool failsafe_check_altitude(float altitude);
bool failsafe_check_speed(float speed);
bool failsafe_check_attitude(float roll, float pitch, float yaw);
bool failsafe_check_geofence(float x, float y, float z, float radius, float altitude);

/* Statistics */
uint32_t failsafe_get_trigger_count(void);
uint32_t failsafe_get_execution_count(void);
uint32_t failsafe_get_recovery_count(void);

#ifdef __cplusplus
}
#endif

#endif /* FAILSAFE_ACTIONS_H */