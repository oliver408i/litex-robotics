/* loader_hook: lets a running WINC app hand the board to the WiFi
 * flash-loader on a host "WFLE" datagram (UDP :5558) -- the cable-free
 * entry path (docs/boot_chain.md). Usage mirrors mdns.c: loader_hook_start()
 * after DHCP, loader_hook_socket_cb() first in the app's socket callback. */
#ifndef LOADER_HOOK_H
#define LOADER_HOOK_H

#include "socket/include/socket.h"

/* Shared with winc_loader's boot triage and the gateware boot_ctl flag. */
#define LOADER_BOOT_MAGIC 0xB007F1A5u
#define LOADER_HOOK_PORT  5558

void  loader_hook_start(void);                                    /* after DHCP   */
uint8 loader_hook_socket_cb(SOCKET sock, uint8 u8Msg, void *pvMsg); /* 1 = consumed */
void  loader_hook_request(void);  /* set flag + reset; does not return */

#endif
