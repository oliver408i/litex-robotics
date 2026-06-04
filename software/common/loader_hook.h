/* loader_hook: lets a running WINC app hand the board to the WiFi
 * flash-loader on request from the host -- the cable-free entry path.
 *
 * Usage (mirrors mdns.c):
 *   - call loader_hook_start() once DHCP is up;
 *   - call loader_hook_socket_cb() FIRST in the app's socket callback and
 *     return if it claims the event.
 *
 * The host (flash.py) sends a "WFLE" datagram to UDP port 5558; the hook
 * acks, writes LOADER_BOOT_MAGIC into the sticky boot_ctl flag (gateware,
 * survives soft reset) and pulses ctrl_reset. The BIOS flash-boots the
 * winc_loader, which sees the magic and stays in WiFi-loader mode instead
 * of chain-booting the app.
 */
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
