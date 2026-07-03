#include "mavlink.h"
#include "mavlink_udp.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ============================================================================
 * MAVLink CRC Table
 * ============================================================================ */
static const uint16_t mavlink_crc_table[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

/* ============================================================================
 * MAVLink Helper Functions
 * ============================================================================ */

uint16_t mavlink_crc16(const uint8_t *data, uint32_t length) {
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < length; i++) {
        crc = (crc >> 8) ^ mavlink_crc_table[(crc ^ data[i]) & 0xFF];
    }
    return crc;
}

void mavlink_encode_uint32(uint8_t *buffer, uint32_t value) {
    buffer[0] = (uint8_t)(value & 0xFF);
    buffer[1] = (uint8_t)((value >> 8) & 0xFF);
    buffer[2] = (uint8_t)((value >> 16) & 0xFF);
    buffer[3] = (uint8_t)((value >> 24) & 0xFF);
}

uint32_t mavlink_decode_uint32(const uint8_t *buffer) {
    return (uint32_t)buffer[0] | ((uint32_t)buffer[1] << 8) |
           ((uint32_t)buffer[2] << 16) | ((uint32_t)buffer[3] << 24);
}

void mavlink_encode_float(uint8_t *buffer, float value) {
    uint32_t val;
    memcpy(&val, &value, sizeof(float));
    mavlink_encode_uint32(buffer, val);
}

float mavlink_decode_float(const uint8_t *buffer) {
    uint32_t val = mavlink_decode_uint32(buffer);
    float result;
    memcpy(&result, &val, sizeof(float));
    return result;
}

/* ============================================================================
 * MAVLink Context Management
 * ============================================================================ */

void mavlink_init(MavlinkContext *ctx, uint8_t system_id, uint8_t component_id) {
    if (!ctx) return;
    
    memset(ctx, 0, sizeof(MavlinkContext));
    ctx->system_id = system_id;
    ctx->component_id = component_id;
    ctx->target_system_id = 0;
    ctx->target_component_id = 0;
    ctx->heartbeat_interval_ms = 1000;
    ctx->state = MAVLINK_STATE_STANDBY;
    ctx->mode = MAVLINK_MODE_STABILIZE;
    ctx->autopilot_type = 0;
    ctx->system_type = 0;
    ctx->connected = false;
    ctx->sequence = 0;
    
    printf("[MAVLINK] Initialized (System: %d, Component: %d)\n", system_id, component_id);
}

void mavlink_set_target(MavlinkContext *ctx, uint8_t system_id, uint8_t component_id) {
    if (!ctx) return;
    ctx->target_system_id = system_id;
    ctx->target_component_id = component_id;
    printf("[MAVLINK] Target set (System: %d, Component: %d)\n", system_id, component_id);
}

void mavlink_update(MavlinkContext *ctx, uint32_t current_time_ms) {
    if (!ctx) return;
    ctx->last_heartbeat = current_time_ms;
    
    /* Check connection timeout */
    if (ctx->connected && (current_time_ms - ctx->last_heartbeat > 5000)) {
        ctx->connected = false;
        printf("[MAVLINK] Connection lost\n");
    }
}

/* ============================================================================
 * MAVLink Message Construction
 * ============================================================================ */

static uint32_t mavlink_build_message(MavlinkContext *ctx, uint8_t *buffer, 
                                       uint32_t msg_id, const uint8_t *payload,
                                       uint32_t payload_len, uint8_t crc_extra) {
    if (!ctx || !buffer || payload_len > 255) return 0;
    
    uint32_t offset = 0;
    
    /* Header (MAVLink v1) */
    buffer[offset++] = 0xFE;  /* Start of frame */
    buffer[offset++] = (uint8_t)(payload_len & 0xFF);  /* Payload length */
    buffer[offset++] = (uint8_t)(ctx->sequence & 0xFF);  /* Sequence */
    buffer[offset++] = ctx->system_id;  /* System ID */
    buffer[offset++] = ctx->component_id;  /* Component ID */
    buffer[offset++] = (uint8_t)(msg_id & 0xFF);  /* Message ID (low byte) */
    
    /* Payload */
    memcpy(&buffer[offset], payload, payload_len);
    offset += payload_len;
    
    /* CRC */
    uint16_t crc = mavlink_crc16(&buffer[1], offset - 1);
    crc = (crc >> 8) ^ mavlink_crc_table[(crc ^ crc_extra) & 0xFF];
    buffer[offset++] = (uint8_t)(crc & 0xFF);
    buffer[offset++] = (uint8_t)((crc >> 8) & 0xFF);
    
    ctx->sequence++;
    ctx->messages_sent++;
    ctx->bytes_sent += offset;
    
    return offset;
}

uint32_t mavlink_heartbeat(MavlinkContext *ctx, uint8_t *buffer, uint32_t buffer_size) {
    if (!ctx || !buffer || buffer_size < 32) return 0;
    
    uint8_t payload[9] = {0};
    mavlink_encode_uint32(&payload[0], ctx->custom_mode);
    payload[4] = ctx->system_type;
    payload[5] = ctx->autopilot_type;
    payload[6] = ctx->mode;
    payload[7] = ctx->state;
    payload[8] = 3; /* MAVLink version */
    
    return mavlink_build_message(ctx, buffer, MAVLINK_MSG_ID_HEARTBEAT, payload, 9, 50);
}

uint32_t mavlink_attitude(MavlinkContext *ctx, uint8_t *buffer, uint32_t buffer_size,
                          float roll, float pitch, float yaw,
                          float rollspeed, float pitchspeed, float yawspeed) {
    if (!ctx || !buffer || buffer_size < 60) return 0;
    
    uint8_t payload[28] = {0};
    mavlink_encode_uint32(&payload[0], ctx->last_heartbeat);
    mavlink_encode_float(&payload[4], roll);
    mavlink_encode_float(&payload[8], pitch);
    mavlink_encode_float(&payload[12], yaw);
    mavlink_encode_float(&payload[16], rollspeed);
    mavlink_encode_float(&payload[20], pitchspeed);
    mavlink_encode_float(&payload[24], yawspeed);
    
    return mavlink_build_message(ctx, buffer, MAVLINK_MSG_ID_ATTITUDE, payload, 28, 39);
}

uint32_t mavlink_global_position(MavlinkContext *ctx, uint8_t *buffer, uint32_t buffer_size,
                                 int32_t lat, int32_t lon, int32_t alt,
                                 int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz,
                                 uint16_t hdg) {
    if (!ctx || !buffer || buffer_size < 64) return 0;
    
    uint8_t payload[28] = {0};
    mavlink_encode_uint32(&payload[0], ctx->last_heartbeat);
    mavlink_encode_uint32(&payload[4], (uint32_t)lat);
    mavlink_encode_uint32(&payload[8], (uint32_t)lon);
    mavlink_encode_uint32(&payload[12], (uint32_t)alt);
    mavlink_encode_uint32(&payload[16], (uint32_t)relative_alt);
    payload[20] = vx & 0xFF;
    payload[21] = (vx >> 8) & 0xFF;
    payload[22] = vy & 0xFF;
    payload[23] = (vy >> 8) & 0xFF;
    payload[24] = vz & 0xFF;
    payload[25] = (vz >> 8) & 0xFF;
    payload[26] = hdg & 0xFF;
    payload[27] = (hdg >> 8) & 0xFF;
    
    return mavlink_build_message(ctx, buffer, MAVLINK_MSG_ID_GLOBAL_POSITION, payload, 28, 104);
}

uint32_t mavlink_sys_status(MavlinkContext *ctx, uint8_t *buffer, uint32_t buffer_size,
                            uint32_t sensors_present, uint32_t sensors_enabled,
                            uint32_t sensors_health, uint16_t voltage_battery,
                            int16_t current_battery, int16_t battery_remaining) {
    if (!ctx || !buffer || buffer_size < 60) return 0;
    
    uint8_t payload[31] = {0};
    mavlink_encode_uint32(&payload[0], sensors_present);
    mavlink_encode_uint32(&payload[4], sensors_enabled);
    mavlink_encode_uint32(&payload[8], sensors_health);
    payload[12] = 0;  /* load */
    payload[13] = 0;
    payload[14] = voltage_battery & 0xFF;
    payload[15] = (voltage_battery >> 8) & 0xFF;
    payload[16] = current_battery & 0xFF;
    payload[17] = (current_battery >> 8) & 0xFF;
    payload[30] = battery_remaining & 0xFF;
    
    return mavlink_build_message(ctx, buffer, MAVLINK_MSG_ID_SYS_STATUS, payload, 31, 124);
}

uint32_t mavlink_vfr_hud(MavlinkContext *ctx, uint8_t *buffer, uint32_t buffer_size,
                         float airspeed, float groundspeed, int16_t heading,
                         uint16_t throttle, float alt, float climb) {
    if (!ctx || !buffer || buffer_size < 40) return 0;
    
    uint8_t payload[20] = {0};
    mavlink_encode_float(&payload[0], airspeed);
    mavlink_encode_float(&payload[4], groundspeed);
    mavlink_encode_float(&payload[8], alt);
    mavlink_encode_float(&payload[12], climb);
    payload[16] = heading & 0xFF;
    payload[17] = (heading >> 8) & 0xFF;
    payload[18] = throttle & 0xFF;
    payload[19] = (throttle >> 8) & 0xFF;
    
    return mavlink_build_message(ctx, buffer, MAVLINK_MSG_ID_VFR_HUD, payload, 20, 20);
}

/* ============================================================================
 * MAVLink Message Parsing
 * ============================================================================ */

static bool mavlink_verify_crc(const uint8_t *buffer, uint32_t length) {
    if (length < 12) return false;
    uint16_t crc = mavlink_crc16(&buffer[1], length - 4);
    uint16_t received_crc = buffer[length - 2] | (buffer[length - 1] << 8);
    return crc == received_crc;
}

bool mavlink_parse_message(MavlinkContext *ctx, const uint8_t *buffer, uint32_t length) {
    if (!ctx || !buffer || length < 12) return false;
    
    /* Check start of frame */
    if (buffer[0] != 0xFE) return false;
    
    /* Verify CRC */
    if (!mavlink_verify_crc(buffer, length)) {
        ctx->messages_dropped++;
        return false;
    }
    
    ctx->messages_received++;
    ctx->bytes_received += length;
    
    uint32_t msg_id = buffer[7] | (buffer[8] << 8) | (buffer[9] << 16);
    const uint8_t *payload = &buffer[10];
    
    /* Process message based on ID */
    switch (msg_id) {
        case MAVLINK_MSG_ID_HEARTBEAT:
            ctx->connected = true;
            ctx->last_heartbeat = ctx->last_heartbeat;
            ctx->state = payload[7];
            ctx->mode = payload[2];
            ctx->custom_mode = payload[3] | (payload[4] << 8) | (payload[5] << 16) | (payload[6] << 24);
            return true;
            
        case MAVLINK_MSG_ID_COMMAND_LONG: {
            MavlinkCommandLong cmd;
            cmd.target_system = payload[0];
            cmd.target_component = payload[1];
            cmd.command = payload[2] | (payload[3] << 8);
            cmd.confirmation = payload[4];
            cmd.param1 = mavlink_decode_float(&payload[5]);
            cmd.param2 = mavlink_decode_float(&payload[9]);
            cmd.param3 = mavlink_decode_float(&payload[13]);
            cmd.param4 = mavlink_decode_float(&payload[17]);
            cmd.param5 = mavlink_decode_float(&payload[21]);
            cmd.param6 = mavlink_decode_float(&payload[25]);
            cmd.param7 = mavlink_decode_float(&payload[29]);
            return mavlink_parse_command_long(ctx, &cmd);
        }
        
        case MAVLINK_MSG_ID_MISSION_ITEM: {
            MavlinkMissionItem item;
            item.target_system = payload[0];
            item.target_component = payload[1];
            item.seq = payload[2] | (payload[3] << 8);
            item.frame = payload[4];
            item.command = payload[5] | (payload[6] << 8);
            item.current = payload[7];
            item.autocontinue = payload[8];
            item.param1 = mavlink_decode_float(&payload[9]);
            item.param2 = mavlink_decode_float(&payload[13]);
            item.param3 = mavlink_decode_float(&payload[17]);
            item.param4 = mavlink_decode_float(&payload[21]);
            item.x = mavlink_decode_float(&payload[25]);
            item.y = mavlink_decode_float(&payload[29]);
            item.z = mavlink_decode_float(&payload[33]);
            return mavlink_parse_mission_item(ctx, &item);
        }
        
        default:
            return true;
    }
}

bool mavlink_parse_command_long(MavlinkContext *ctx, const MavlinkCommandLong *cmd) {
    if (!ctx || !cmd) return false;
    
    /* Check if this command is for us */
    if (cmd->target_system != ctx->system_id && cmd->target_system != 0) {
        return true;
    }
    
    printf("[MAVLINK] Command received: %d\n", cmd->command);
    
    return mavlink_process_command(ctx, cmd->command,
                                   cmd->param1, cmd->param2, cmd->param3,
                                   cmd->param4, cmd->param5, cmd->param6, cmd->param7);
}

bool mavlink_parse_mission_item(MavlinkContext *ctx, const MavlinkMissionItem *item) {
    if (!ctx || !item) return false;
    
    printf("[MAVLINK] Mission Item %d: Command %d, Pos: %.2f, %.2f, %.2f\n",
           item->seq, item->command, (double)item->x, (double)item->y, (double)item->z);
    return true;
}

/* ============================================================================
 * MAVLink Command Processing
 * ============================================================================ */

bool mavlink_process_command(MavlinkContext *ctx, uint16_t command, 
                             float param1, float param2, float param3,
                             float param4, float param5, float param6, float param7) {
    if (!ctx) return false;
    (void)param2;
    (void)param3;
    (void)param4;
    (void)param5;
    (void)param6;
    
    switch (command) {
        case MAVLINK_CMD_NAV_TAKEOFF:
            printf("[MAVLINK] Command: Takeoff (Alt: %.2f)\n", (double)param7);
            mavlink_send_command_ack(ctx, command, MAVLINK_CMD_RESULT_ACCEPTED);
            return true;
            
        case MAVLINK_CMD_NAV_LAND:
            printf("[MAVLINK] Command: Land\n");
            mavlink_send_command_ack(ctx, command, MAVLINK_CMD_RESULT_ACCEPTED);
            return true;
            
        case MAVLINK_CMD_NAV_RETURN_TO_LAUNCH:
            printf("[MAVLINK] Command: Return to Launch\n");
            mavlink_send_command_ack(ctx, command, MAVLINK_CMD_RESULT_ACCEPTED);
            return true;
            
        case MAVLINK_CMD_COMPONENT_ARM_DISARM:
            printf("[MAVLINK] Command: %s\n", param1 > 0.5f ? "ARM" : "DISARM");
            mavlink_send_command_ack(ctx, command, MAVLINK_CMD_RESULT_ACCEPTED);
            return true;
            
        case MAVLINK_CMD_DO_SET_MODE:
            printf("[MAVLINK] Command: Set Mode %d\n", (int)param1);
            mavlink_send_command_ack(ctx, command, MAVLINK_CMD_RESULT_ACCEPTED);
            return true;
            
        case MAVLINK_CMD_MISSION_START:
            printf("[MAVLINK] Command: Start Mission\n");
            mavlink_send_command_ack(ctx, command, MAVLINK_CMD_RESULT_ACCEPTED);
            return true;
            
        default:
            printf("[MAVLINK] Unknown command: %d\n", command);
            mavlink_send_command_ack(ctx, command, MAVLINK_CMD_RESULT_UNSUPPORTED);
            return false;
    }
}

bool mavlink_send_command_ack(MavlinkContext *ctx, uint16_t command, uint8_t result) {
    if (!ctx) return false;
    
    (void)result;
    
    /* Build ACK message (not actually sending, just logging) */
    printf("[MAVLINK] Command ACK: %d -> Result: %d\n", command, result);
    return true;
}

/* ============================================================================
 * MAVLink Telemetry Integration
 * ============================================================================ */

MavlinkTelemetry g_telemetry;

void mavlink_telemetry_init(uint8_t system_id, uint8_t component_id) {
    mavlink_init(&g_telemetry.ctx, system_id, component_id);
    g_telemetry.telemetry_interval_ms = 100;
    g_telemetry.telemetry_enabled = true;
    g_telemetry.last_telemetry_send = 0;
    printf("[TELEMETRY] Initialized\n");
}

void mavlink_telemetry_update(uint32_t current_time_ms,
                              float roll, float pitch, float yaw,
                              float rollspeed, float pitchspeed, float yawspeed,
                              int32_t lat, int32_t lon, int32_t alt,
                              int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz,
                              uint16_t hdg) {
    if (!g_telemetry.telemetry_enabled) return;
    if (current_time_ms - g_telemetry.last_telemetry_send < g_telemetry.telemetry_interval_ms) {
        return;
    }
    
    g_telemetry.last_telemetry_send = current_time_ms;
    
    /* Build and send heartbeat (every 1 second) */
    if (current_time_ms % 1000 < g_telemetry.telemetry_interval_ms) {
        uint32_t len = mavlink_heartbeat(&g_telemetry.ctx, g_telemetry.tx_buffer, sizeof(g_telemetry.tx_buffer));
        mavlink_udp_send(g_telemetry.tx_buffer, len);
    }
    
    /* Build and send attitude */
    uint32_t len = mavlink_attitude(&g_telemetry.ctx, g_telemetry.tx_buffer, sizeof(g_telemetry.tx_buffer),
                                    roll, pitch, yaw, rollspeed, pitchspeed, yawspeed);
    mavlink_udp_send(g_telemetry.tx_buffer, len);
    
    /* Build and send global position */
    len = mavlink_global_position(&g_telemetry.ctx, g_telemetry.tx_buffer, sizeof(g_telemetry.tx_buffer),
                                  lat, lon, alt, relative_alt, vx, vy, vz, hdg);
    mavlink_udp_send(g_telemetry.tx_buffer, len);
    
    /* Build and send VFR HUD */
    float airspeed = sqrtf(vx*vx + vy*vy + vz*vz);
    float groundspeed = sqrtf(vx*vx + vy*vy);
    len = mavlink_vfr_hud(&g_telemetry.ctx, g_telemetry.tx_buffer, sizeof(g_telemetry.tx_buffer),
                          airspeed, groundspeed, hdg, 0, alt, vz);
    mavlink_udp_send(g_telemetry.tx_buffer, len);
    
    /* Build and send system status */
    uint32_t sensors_present = 0xFFFFFFFF;
    uint32_t sensors_enabled = 0xFFFFFFFF;
    uint32_t sensors_health = 0xFFFFFFFF;
    uint16_t voltage_battery = 12500; /* 12.5V */
    int16_t current_battery = 0;
    int16_t battery_remaining = 100;
    len = mavlink_sys_status(&g_telemetry.ctx, g_telemetry.tx_buffer, sizeof(g_telemetry.tx_buffer),
                             sensors_present, sensors_enabled, sensors_health,
                             voltage_battery, current_battery, battery_remaining);
    mavlink_udp_send(g_telemetry.tx_buffer, len);
}

/* ============================================================================
 * GCS Command Interface
 * ============================================================================ */

typedef struct {
    bool armed;
    uint8_t mode;
    float target_altitude;
    float target_position[2];
    bool mission_active;
    uint32_t mission_current_waypoint;
    uint32_t mission_total_waypoints;
    float mission_progress;
} GCSCommandState;

static GCSCommandState g_gcs_state;

void gcs_command_init(void) {
    memset(&g_gcs_state, 0, sizeof(GCSCommandState));
    g_gcs_state.mode = MAVLINK_MODE_STABILIZE;
    printf("[GCS] Command Interface Initialized\n");
}

bool gcs_process_command(uint16_t command, float *params, uint32_t param_count) {
    (void)param_count;
    switch (command) {
        case MAVLINK_CMD_COMPONENT_ARM_DISARM:
            g_gcs_state.armed = params[0] > 0.5f;
            printf("[GCS] Drone %s\n", g_gcs_state.armed ? "ARMED" : "DISARMED");
            return true;
            
        case MAVLINK_CMD_NAV_TAKEOFF:
            g_gcs_state.target_altitude = params[6]; /* param7 is altitude */
            printf("[GCS] Takeoff to %.2fm\n", (double)g_gcs_state.target_altitude);
            return true;
            
        case MAVLINK_CMD_NAV_LAND:
            printf("[GCS] Landing\n");
            return true;
            
        case MAVLINK_CMD_NAV_RETURN_TO_LAUNCH:
            printf("[GCS] Return to Launch\n");
            return true;
            
        case MAVLINK_CMD_DO_SET_MODE:
            g_gcs_state.mode = (uint8_t)params[0];
            printf("[GCS] Mode changed to %d\n", g_gcs_state.mode);
            return true;
            
        case MAVLINK_CMD_MISSION_START:
            g_gcs_state.mission_active = true;
            printf("[GCS] Mission started\n");
            return true;
            
        default:
            return false;
    }
}

bool gcs_is_armed(void) {
    return g_gcs_state.armed;
}

uint8_t gcs_get_mode(void) {
    return g_gcs_state.mode;
}

float gcs_get_target_altitude(void) {
    return g_gcs_state.target_altitude;
}

bool gcs_is_mission_active(void) {
    return g_gcs_state.mission_active;
}

void gcs_set_mission_progress(float progress) {
    g_gcs_state.mission_progress = progress;
}

float gcs_get_mission_progress(void) {
    return g_gcs_state.mission_progress;
}
