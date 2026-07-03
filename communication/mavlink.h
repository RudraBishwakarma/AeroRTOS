#ifndef AERORTOS_MAVLINK_H
#define AERORTOS_MAVLINK_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * MAVLink Protocol Implementation
 * Based on MAVLink v1.0 / v2.0
 * ============================================================================ */

/* MAVLink Message Types */
#define MAVLINK_MSG_ID_HEARTBEAT          0
#define MAVLINK_MSG_ID_SYS_STATUS         1
#define MAVLINK_MSG_ID_BATTERY_STATUS     2
#define MAVLINK_MSG_ID_ATTITUDE           3
#define MAVLINK_MSG_ID_GLOBAL_POSITION    4
#define MAVLINK_MSG_ID_LOCAL_POSITION     5
#define MAVLINK_MSG_ID_VFR_HUD            6
#define MAVLINK_MSG_ID_COMMAND_LONG       7
#define MAVLINK_MSG_ID_COMMAND_ACK        8
#define MAVLINK_MSG_ID_MISSION_ITEM       9
#define MAVLINK_MSG_ID_MISSION_ACK        10
#define MAVLINK_MSG_ID_MISSION_COUNT      11
#define MAVLINK_MSG_ID_MISSION_REQUEST    12
#define MAVLINK_MSG_ID_SET_MODE           13
#define MAVLINK_MSG_ID_PARAM_REQUEST      14
#define MAVLINK_MSG_ID_PARAM_VALUE        15
#define MAVLINK_MSG_ID_RC_CHANNELS        16
#define MAVLINK_MSG_ID_RAW_IMU            17
#define MAVLINK_MSG_ID_SCALED_IMU         18
#define MAVLINK_MSG_ID_GPS_RAW            19
#define MAVLINK_MSG_ID_ATTITUDE_QUATERNION 20

/* MAVLink System States */
#define MAVLINK_STATE_UNINIT              0
#define MAVLINK_STATE_BOOT                1
#define MAVLINK_STATE_CALIBRATING         2
#define MAVLINK_STATE_STANDBY             3
#define MAVLINK_STATE_ACTIVE              4
#define MAVLINK_STATE_CRITICAL            5
#define MAVLINK_STATE_EMERGENCY           6
#define MAVLINK_STATE_POWEROFF            7
#define MAVLINK_STATE_FLIGHT_TERMINATION  8

/* MAVLink Modes */
#define MAVLINK_MODE_MANUAL               0
#define MAVLINK_MODE_STABILIZE            1
#define MAVLINK_MODE_ALT_HOLD             2
#define MAVLINK_MODE_LOITER               3
#define MAVLINK_MODE_AUTO                 4
#define MAVLINK_MODE_RTL                  5
#define MAVLINK_MODE_LAND                 6
#define MAVLINK_MODE_POSITION             7
#define MAVLINK_MODE_ACRO                 8
#define MAVLINK_MODE_DRIFT                9
#define MAVLINK_MODE_SPORT                10
#define MAVLINK_MODE_FLIP                 11
#define MAVLINK_MODE_AUTOTUNE             12
#define MAVLINK_MODE_POSHOLD              13

/* MAVLink Command Result */
#define MAVLINK_CMD_RESULT_ACCEPTED       0
#define MAVLINK_CMD_RESULT_TEMPORARILY_REJECTED 1
#define MAVLINK_CMD_RESULT_DENIED         2
#define MAVLINK_CMD_RESULT_UNSUPPORTED    3
#define MAVLINK_CMD_RESULT_FAILED         4
#define MAVLINK_CMD_RESULT_IN_PROGRESS    5
#define MAVLINK_CMD_RESULT_CANCELLED      6

/* MAVLink Command IDs */
#define MAVLINK_CMD_NAV_TAKEOFF           22
#define MAVLINK_CMD_NAV_LAND              21
#define MAVLINK_CMD_NAV_RETURN_TO_LAUNCH  20
#define MAVLINK_CMD_NAV_WAYPOINT          16
#define MAVLINK_CMD_COMPONENT_ARM_DISARM  400
#define MAVLINK_CMD_DO_SET_MODE           176
#define MAVLINK_CMD_DO_SET_ROI            201
#define MAVLINK_CMD_MISSION_START         300
#define MAVLINK_CMD_PREFLIGHT_CALIBRATION 241

/* ============================================================================
 * MAVLink Message Structures
 * ============================================================================ */

/* Heartbeat Message */
typedef struct {
    uint32_t custom_mode;
    uint8_t type;
    uint8_t autopilot;
    uint8_t base_mode;
    uint8_t system_status;
    uint8_t mavlink_version;
} MavlinkHeartbeat;

/* Attitude Message */
typedef struct {
    uint32_t time_boot_ms;
    float roll;
    float pitch;
    float yaw;
    float rollspeed;
    float pitchspeed;
    float yawspeed;
} MavlinkAttitude;

/* Global Position Message */
typedef struct {
    uint32_t time_boot_ms;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    int32_t relative_alt;
    int16_t vx;
    int16_t vy;
    int16_t vz;
    uint16_t hdg;
} MavlinkGlobalPosition;

/* Local Position Message */
typedef struct {
    uint32_t time_boot_ms;
    float x;
    float y;
    float z;
    float vx;
    float vy;
    float vz;
} MavlinkLocalPosition;

/* VFR HUD Message */
typedef struct {
    float airspeed;
    float groundspeed;
    int16_t heading;
    uint16_t throttle;
    float alt;
    float climb;
} MavlinkVfrHud;

/* Command Long Message */
typedef struct {
    uint8_t target_system;
    uint8_t target_component;
    uint16_t command;
    uint8_t confirmation;
    float param1;
    float param2;
    float param3;
    float param4;
    float param5;
    float param6;
    float param7;
} MavlinkCommandLong;

/* Command ACK Message */
typedef struct {
    uint16_t command;
    uint8_t result;
} MavlinkCommandAck;

/* Mission Item Message */
typedef struct {
    uint8_t target_system;
    uint8_t target_component;
    uint16_t seq;
    uint8_t frame;
    uint16_t command;
    uint8_t current;
    uint8_t autocontinue;
    float param1;
    float param2;
    float param3;
    float param4;
    float x;
    float y;
    float z;
} MavlinkMissionItem;

/* Mission Count Message */
typedef struct {
    uint8_t target_system;
    uint8_t target_component;
    uint16_t count;
} MavlinkMissionCount;

/* Mission Request Message */
typedef struct {
    uint8_t target_system;
    uint8_t target_component;
    uint16_t seq;
} MavlinkMissionRequest;

/* Mission ACK Message */
typedef struct {
    uint8_t target_system;
    uint8_t target_component;
    uint8_t type;
} MavlinkMissionAck;

/* System Status Message */
typedef struct {
    uint32_t onboard_control_sensors_present;
    uint32_t onboard_control_sensors_enabled;
    uint32_t onboard_control_sensors_health;
    uint16_t load;
    uint16_t voltage_battery;
    int16_t current_battery;
    int16_t battery_remaining;
    uint16_t drop_rate_comm;
    uint16_t errors_comm;
    uint16_t errors_count1;
    uint16_t errors_count2;
    uint16_t errors_count3;
    uint16_t errors_count4;
} MavlinkSysStatus;

/* ============================================================================
 * MAVLink Context
 * ============================================================================ */

typedef struct {
    uint8_t system_id;
    uint8_t component_id;
    uint8_t target_system_id;
    uint8_t target_component_id;
    uint32_t sequence;
    bool connected;
    uint32_t last_heartbeat;
    uint32_t heartbeat_interval_ms;
    uint8_t state;
    uint8_t mode;
    uint32_t custom_mode;
    uint8_t autopilot_type;
    uint8_t system_type;
    float mission_progress;
    
    /* Statistics */
    uint32_t messages_sent;
    uint32_t messages_received;
    uint32_t messages_dropped;
    uint32_t bytes_sent;
    uint32_t bytes_received;
    float bandwidth_usage;
} MavlinkContext;

/* ============================================================================
 * Function Prototypes
 * ============================================================================ */

/* MAVLink Context Management */
void mavlink_init(MavlinkContext *ctx, uint8_t system_id, uint8_t component_id);
void mavlink_set_target(MavlinkContext *ctx, uint8_t system_id, uint8_t component_id);
void mavlink_update(MavlinkContext *ctx, uint32_t current_time_ms);

/* MAVLink Message Construction */
uint32_t mavlink_heartbeat(MavlinkContext *ctx, uint8_t *buffer, uint32_t buffer_size);
uint32_t mavlink_attitude(MavlinkContext *ctx, uint8_t *buffer, uint32_t buffer_size,
                          float roll, float pitch, float yaw,
                          float rollspeed, float pitchspeed, float yawspeed);
uint32_t mavlink_global_position(MavlinkContext *ctx, uint8_t *buffer, uint32_t buffer_size,
                                 int32_t lat, int32_t lon, int32_t alt,
                                 int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz,
                                 uint16_t hdg);
uint32_t mavlink_local_position(MavlinkContext *ctx, uint8_t *buffer, uint32_t buffer_size,
                                float x, float y, float z,
                                float vx, float vy, float vz);
uint32_t mavlink_vfr_hud(MavlinkContext *ctx, uint8_t *buffer, uint32_t buffer_size,
                         float airspeed, float groundspeed, int16_t heading,
                         uint16_t throttle, float alt, float climb);
uint32_t mavlink_sys_status(MavlinkContext *ctx, uint8_t *buffer, uint32_t buffer_size,
                            uint32_t sensors_present, uint32_t sensors_enabled,
                            uint32_t sensors_health, uint16_t voltage_battery,
                            int16_t current_battery, int16_t battery_remaining);

/* MAVLink Message Parsing */
bool mavlink_parse_message(MavlinkContext *ctx, const uint8_t *buffer, uint32_t length);
bool mavlink_parse_command_long(MavlinkContext *ctx, const MavlinkCommandLong *cmd);
bool mavlink_parse_mission_item(MavlinkContext *ctx, const MavlinkMissionItem *item);

/* MAVLink Command Processing */
bool mavlink_process_command(MavlinkContext *ctx, uint16_t command, 
                             float param1, float param2, float param3,
                             float param4, float param5, float param6, float param7);
bool mavlink_send_command_ack(MavlinkContext *ctx, uint16_t command, uint8_t result);


/* Telemetry/GCS convenience API */
void mavlink_telemetry_init(uint8_t system_id, uint8_t component_id);
void mavlink_telemetry_update(uint32_t current_time_ms,
                              float roll, float pitch, float yaw,
                              float rollspeed, float pitchspeed, float yawspeed,
                              int32_t lat, int32_t lon, int32_t alt,
                              int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz,
                              uint16_t hdg);
void gcs_command_init(void);
bool gcs_process_command(uint16_t command, float *params, uint32_t param_count);
bool gcs_is_armed(void);
uint8_t gcs_get_mode(void);
float gcs_get_target_altitude(void);
bool gcs_is_mission_active(void);
void gcs_set_mission_progress(float progress);
float gcs_get_mission_progress(void);
/* MAVLink Helper Functions */
uint16_t mavlink_crc16(const uint8_t *data, uint32_t length);
float gcs_get_mission_progress(void);
/* MAVLink Helper Functions */
uint16_t mavlink_crc16(const uint8_t *data, uint32_t length);
void mavlink_encode_uint32(uint8_t *buffer, uint32_t value);
uint32_t mavlink_decode_uint32(const uint8_t *buffer);
void mavlink_encode_float(uint8_t *buffer, float value);
float mavlink_decode_float(const uint8_t *buffer);

/* MAVLink UDP Functions */
bool mavlink_udp_init(const char *ip, uint16_t port);
void mavlink_udp_send(uint8_t *buffer, uint32_t length);
void mavlink_udp_cleanup(void);
bool mavlink_udp_is_initialized(void);

typedef struct {
    MavlinkContext ctx;
    uint8_t tx_buffer[512];
    uint8_t rx_buffer[512];
    uint32_t last_telemetry_send;
    uint32_t telemetry_interval_ms;
    bool telemetry_enabled;
} MavlinkTelemetry;

extern MavlinkTelemetry g_telemetry;

#ifdef __cplusplus
}
#endif

#endif /* AERORTOS_MAVLINK_H */