#ifndef MAVLINK_UDP_H
#define MAVLINK_UDP_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool mavlink_udp_init(const char *ip, uint16_t port);
void mavlink_udp_send(uint8_t *buffer, uint32_t length);
void mavlink_udp_cleanup(void);
bool mavlink_udp_is_initialized(void);

#ifdef __cplusplus
}
#endif

#endif /* MAVLINK_UDP_H */