#ifndef WIFI_SECRETS_H_
#define WIFI_SECRETS_H_

/* Local Wi-Fi credentials -- EDIT ME, never commit (gitignored).
 *
 * The board joins this network (2.4 GHz only -- the WINC1500 has no 5 GHz),
 * then listens on TCP port LISTEN_PORT (main.c); connect from the host with
 * `nc <board-ip> 5555` (IP is printed on the UART after DHCP). */

#define WIFI_SSID   "BJL-NEW-2G"
#define WIFI_PSK    "Belinda_li"

#endif /* WIFI_SECRETS_H_ */
