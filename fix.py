with open('main.c', 'r') as f:
    lines = f.readlines()

new_lines = []
skip = False

for line in lines:
    if 'queue_create("IMUData"' in line:
        new_lines.append(line)
        new_lines.append('    queue_create("GPSData", sizeof(GPSData), 10, false, &gps_data_queue);\n')
        new_lines.append('    queue_create("BaroData", sizeof(BaroData), 10, false, &baro_data_queue);\n')
        new_lines.append('    queue_create("Telemetry", sizeof(float[8]), 30, false, &telemetry_queue);\n')
        new_lines.append('    queue_create("Command", sizeof(float[3]), 10, false, &command_queue);\n')
        new_lines.append('    queue_create("Health", sizeof(HealthData), 20, false, &health_queue);\n')
        new_lines.append('    printf("[OK] Message queues created\\n");\n')
        new_lines.append('\n')
        new_lines.append('    /* Initialize Flight Controller */\n')
        new_lines.append('    ekf_init(&g_ekf_state);\n')
        new_lines.append('    cascaded_pid_init(&g_pid);\n')
        new_lines.append('    motor_mixer_init(&g_mixer, MIXER_QUAD_X);\n')
        new_lines.append('    printf("[OK] Flight Controller initialized\\n");\n')
        new_lines.append('\n')
        new_lines.append('    /* Initialize Safety Systems */\n')
        new_lines.append('    watchdog_init();\n')
        new_lines.append('    watchdog_start();\n')
        new_lines.append('    printf("[OK] Watchdog initialized\\n");\n')
        new_lines.append('\n')
        new_lines.append('    FailsafeSystemConfig failsafe_config;\n')
        new_lines.append('    failsafe_config_init(&failsafe_config);\n')
        new_lines.append('    failsafe_actions_init(&failsafe_config);\n')
        new_lines.append('    printf("[OK] Failsafe system initialized\\n");\n')
        new_lines.append('\n')
        new_lines.append('    health_monitor_init();\n')
        new_lines.append('    printf("[OK] Health monitor initialized\\n");\n')
        new_lines.append('\n')
        new_lines.append('    if (!mavlink_udp_init("127.0.0.1", 14551)) {\n')
        new_lines.append('        printf("[WARNING] UDP telemetry initialization failed\\n");\n')
        new_lines.append('    } else {\n')
        new_lines.append('        printf("[OK] UDP telemetry enabled (127.0.0.1:14551)\\n");\n')
        new_lines.append('    }\n')
        new_lines.append('\n')
        new_lines.append('    /* Initialize Performance Profilers */\n')
        new_lines.append('    cpu_profiler_init();\n')
        new_lines.append('    cpu_profiler_start();\n')
        new_lines.append('    printf("[OK] CPU profiler initialized\\n");\n')
        new_lines.append('\n')
        new_lines.append('    deadline_analyzer_init();\n')
        new_lines.append('    deadline_analyzer_start();\n')
        new_lines.append('    printf("[OK] Deadline analyzer initialized\\n");\n')
        new_lines.append('\n')
        new_lines.append('    memory_profiler_init();\n')
        new_lines.append('    printf("[OK] Memory profiler initialized\\n");\n')
        new_lines.append('\n')
        new_lines.append('    csv_logger_init();\n')
        new_lines.append('    printf("[OK] CSV logging enabled\\n");\n')
        new_lines.append('\n')
        new_lines.append('    /* Create Tasks */\n')
        new_lines.append('    err = task_create("Sensor", sensor_task_function, NULL, \n')
        new_lines.append('                      TASK_PRIORITY_CRITICAL, 2048, &sensor_task_id);\n')
        new_lines.append('    if (err != KERNEL_SUCCESS) return -1;\n')
        new_lines.append('\n')
        new_lines.append('    err = task_create("Control", control_task_function, NULL, \n')
        new_lines.append('                      TASK_PRIORITY_REAL_TIME, 4096, &control_task_id);\n')
        new_lines.append('    if (err != KERNEL_SUCCESS) return -1;\n')
        new_lines.append('\n')
        skip = True
        continue
    
    if skip and 'err = task_create("Estimator"' in line:
        skip = False
        new_lines.append(line)
        continue
        
    if not skip:
        new_lines.append(line)

with open('main.c', 'w') as f:
    f.writelines(new_lines)
