#ifndef CSV_LOGGER_H
#define CSV_LOGGER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void csv_logger_init(void);
void csv_logger_log_flight(float roll, float pitch, float yaw, float altitude,
                           float pos_x, float pos_y, float pos_z,
                           float motor1, float motor2, float motor3, float motor4);
void csv_logger_log_health(float cpu_usage, float battery_voltage, float battery_remaining,
                           uint32_t deadline_misses, uint32_t task_count);
void csv_logger_close(void);

#ifdef __cplusplus
}
#endif

#endif /* CSV_LOGGER_H */