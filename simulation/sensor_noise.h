#ifndef SENSOR_NOISE_H
#define SENSOR_NOISE_H

#include "sensors/imu_sensor.h"
#include "sensors/gps_sensor.h"

typedef struct {
    float noise_level;
} SensorNoiseConfig;

void sensor_noise_init(SensorNoiseConfig *config);
void sensor_noise_apply_imu(IMUData *imu, const SensorNoiseConfig *config);
void sensor_noise_apply_gps(GPSData *gps, const SensorNoiseConfig *config);

#endif /* SENSOR_NOISE_H */
