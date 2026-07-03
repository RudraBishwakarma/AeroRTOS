import re

with open("main.c", "r") as f:
    text = f.read()

# 1. Update include
text = text.replace('#include "communication/mavlink.h"', '#include "mavlink/common/mavlink.h"')

# 2. Update port
text = text.replace('mavlink_udp_init("127.0.0.1", 14551)', 'mavlink_udp_init("127.0.0.1", 14550)')
text = text.replace('UDP telemetry enabled (127.0.0.1:14551)', 'UDP telemetry enabled (127.0.0.1:14550)')

# 3. Rewrite telemetry_task_function
new_task = """void telemetry_task_function(void *params) {
    (void)params;
    printf("[TELEMETRY] Started (ID: %u)\\n", task_get_current_id());
    
    uint32_t counter = 0;
    uint32_t current_time = 0;
    uint32_t start_time, end_time, exec_time;
    
    gcs_command_init();
    
    uint8_t system_id = 1;
    uint8_t component_id = 1;
    uint8_t tx_buffer[2048];
    mavlink_message_t msg;
    uint16_t len;
    
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
        
        /* 1. Heartbeat (every 1 second) */
        if (counter % 10 == 0) {
            mavlink_msg_heartbeat_pack(system_id, component_id, &msg,
                                       MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC,
                                       MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0, MAV_STATE_ACTIVE);
            len = mavlink_msg_to_send_buffer(tx_buffer, &msg);
            mavlink_udp_send(tx_buffer, len);
        }
        
        /* 2. Attitude */
        mavlink_msg_attitude_pack(system_id, component_id, &msg,
                                  current_time, roll, pitch, yaw, 0.0f, 0.0f, 0.0f);
        len = mavlink_msg_to_send_buffer(tx_buffer, &msg);
        mavlink_udp_send(tx_buffer, len);
        
        /* 3. Global Position */
        mavlink_msg_global_position_int_pack(system_id, component_id, &msg,
                                             current_time, lat, lon, (int32_t)(alt * 1000),
                                             (int32_t)(alt * 1000),
                                             (int16_t)(vx * 100), (int16_t)(vy * 100), (int16_t)(vz * 100), 0);
        len = mavlink_msg_to_send_buffer(tx_buffer, &msg);
        mavlink_udp_send(tx_buffer, len);
        
        /* 4. VFR HUD */
        float airspeed = sqrtf(vx*vx + vy*vy + vz*vz);
        float groundspeed = sqrtf(vx*vx + vy*vy);
        mavlink_msg_vfr_hud_pack(system_id, component_id, &msg,
                                 airspeed, groundspeed, 0, 0, alt, vz);
        len = mavlink_msg_to_send_buffer(tx_buffer, &msg);
        mavlink_udp_send(tx_buffer, len);
        
        /* 5. System Status (every 5 seconds) */
        if (counter % 50 == 0) {
            uint16_t voltage_battery = (uint16_t)(g_battery_voltage * 1000);
            int16_t current_battery = (int16_t)(g_battery_current * 100);
            int8_t battery_remaining = (int8_t)g_battery_remaining;
            
            mavlink_msg_sys_status_pack(system_id, component_id, &msg,
                                        0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 500,
                                        voltage_battery, current_battery, battery_remaining,
                                        0, 0, 0, 0, 0, 0, 0, 0, 0);
            len = mavlink_msg_to_send_buffer(tx_buffer, &msg);
            mavlink_udp_send(tx_buffer, len);
        }
        
        if (counter % 100 == 0) {
            printf("[TELEMETRY] MAVLink heartbeat sent over UDP\\n");
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
}"""

pattern = re.compile(r'void telemetry_task_function\(void \*params\) \{.*?\n\}\n', re.DOTALL)
text = pattern.sub(new_task + "\n", text)

with open("main.c", "w") as f:
    f.write(text)
