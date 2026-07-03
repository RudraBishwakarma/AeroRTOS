#include "gps_sensor.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

static struct {
    uint32_t update_rate;
    double position_lat;
    double position_lon;
    float position_alt;
    uint32_t last_update;
    bool initialized;
} g_gps = {
    .position_lat = 37.7749,
    .position_lon = -122.4194,
    .position_alt = 0.0f,
    .initialized = false
};

void gps_sensor_init(uint32_t update_rate_hz) {
    g_gps.update_rate = update_rate_hz;
    g_gps.initialized = true;
    printf("GPS Sensor initialized at %u Hz\n", update_rate_hz);
}

void gps_sensor_read(GPSData *data) {
    if (!data || !g_gps.initialized) {
        return;
    }
    
    /* Simulate GPS data with slight movement */
    static float time = 0;
    time += 0.2f;
    
    /* Simulate circular flight pattern */
    float radius = 0.0001f; /* ~11 meters */
    g_gps.position_lat += sin(time * 0.1f) * radius * 0.01f;
    g_gps.position_lon += cos(time * 0.1f) * radius * 0.01f;
    g_gps.position_alt += sin(time * 0.05f) * 0.1f;
    
    data->latitude = g_gps.position_lat + (rand() % 100 - 50) * 1e-7;
    data->longitude = g_gps.position_lon + (rand() % 100 - 50) * 1e-7;
    data->altitude = g_gps.position_alt + (rand() % 100 - 50) * 0.01f;
    data->velocity_north = (rand() % 100 - 50) * 0.01f;
    data->velocity_east = (rand() % 100 - 50) * 0.01f;
    data->velocity_down = (rand() % 100 - 50) * 0.01f;
    data->hdop = 1.0f + (rand() % 100 - 50) * 0.01f;
    data->satellites = 8 + (rand() % 8);
    data->timestamp = (uint32_t)(time * 1000);
    data->valid = true;
    data->fix_type = true;
}

void gps_sensor_set_position(double lat, double lon, float alt) {
    g_gps.position_lat = lat;
    g_gps.position_lon = lon;
    g_gps.position_alt = alt;
}