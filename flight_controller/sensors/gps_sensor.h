#ifndef GPS_SENSOR_H
#define GPS_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double latitude;    /* degrees */
    double longitude;   /* degrees */
    float altitude;     /* meters */
    float velocity_north; /* m/s */
    float velocity_east;  /* m/s */
    float velocity_down;  /* m/s */
    float hdop;         /* Horizontal dilution of precision */
    uint32_t satellites;
    uint32_t timestamp;
    bool valid;
    bool fix_type;
} GPSData;

void gps_sensor_init(uint32_t update_rate_hz);
void gps_sensor_read(GPSData *data);
void gps_sensor_set_position(double lat, double lon, float alt);

#ifdef __cplusplus
}
#endif

#endif /* GPS_SENSOR_H */