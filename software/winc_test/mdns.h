#ifndef WINC_MDNS_H_
#define WINC_MDNS_H_

#include <stdint.h>
#include "socket/include/socket.h"

/* Minimal mDNS responder: answers A-record queries for MDNS_HOSTNAME".local"
 * with our IP, so the host can reach the board as e.g. `nc icepi.local 5555`
 * without reading the DHCP'd address off the UART.
 *
 * Port of WiFi101's WiFiMDNSResponder (Adafruit/Tony DiCola, LGPL) to plain C
 * on the raw WINC socket API. Name queries only -- no services/SRV/PTR. */

#define MDNS_HOSTNAME "icepi"   /* responds to icepi.local */

/* Call once DHCP has completed. ip_be = our IPv4 in network byte order
 * (tstrM2MIPConfig.u32StaticIP as delivered by M2M_WIFI_REQ_DHCP_CONF). */
void mdns_start(uint32 ip_be);

/* Socket-event hook: call FIRST in the app's socket callback; returns 1 if
 * the event belonged to the mDNS socket (and was consumed), 0 otherwise. */
uint8 mdns_socket_cb(SOCKET sock, uint8 u8Msg, void *pvMsg);

#endif /* WINC_MDNS_H_ */
