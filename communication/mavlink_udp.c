#include "mavlink_udp.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <winsock2.h>
#include <ws2tcpip.h>


static SOCKET g_udp_socket = INVALID_SOCKET;
static struct sockaddr_in g_udp_addr;
static bool g_udp_initialized = false;

bool mavlink_udp_init(const char *ip, uint16_t port) {
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        printf("[UDP] WSAStartup failed\n");
        return false;
    }
    
    g_udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_udp_socket == INVALID_SOCKET) {
        printf("[UDP] Socket creation failed\n");
        WSACleanup();
        return false;
    }
    
    memset(&g_udp_addr, 0, sizeof(g_udp_addr));
    g_udp_addr.sin_family = AF_INET;
    g_udp_addr.sin_port = htons(port);
    inet_pton(AF_INET, ip, &g_udp_addr.sin_addr);
    
    g_udp_initialized = true;
    printf("[UDP] MAVLink telemetry initialized: %s:%d\n", ip, port);
    return true;
}

void mavlink_udp_send(uint8_t *buffer, uint32_t length) {
    if (!g_udp_initialized || g_udp_socket == INVALID_SOCKET) {
        return;
    }
    
    int result = sendto(g_udp_socket, (const char*)buffer, length, 0,
                       (struct sockaddr*)&g_udp_addr, sizeof(g_udp_addr));
    if (result == SOCKET_ERROR) {
        // Silent fail to avoid spam
    }
}

void mavlink_udp_cleanup(void) {
    if (g_udp_socket != INVALID_SOCKET) {
        closesocket(g_udp_socket);
        g_udp_socket = INVALID_SOCKET;
    }
    WSACleanup();
    g_udp_initialized = false;
}

bool mavlink_udp_is_initialized(void) {
    return g_udp_initialized;
}