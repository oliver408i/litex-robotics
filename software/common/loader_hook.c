/* See loader_hook.h. Socket-callback pattern copied from winc_test/mdns.c. */
#include <string.h>

#include <generated/csr.h>
#include <libbase/uart.h>
#include <system.h>

#include "loader_hook.h"

#ifndef CSR_BOOT_CTL_FLAG_ADDR
#error "SoC has no boot_ctl flag -- build a top that calls add_boot_ctl() (icepi_zero_all/winc)."
#endif

static SOCKET hook_sock = -1;
static uint8  hook_rx[32];

void loader_hook_request(void)
{
	boot_ctl_flag_write(LOADER_BOOT_MAGIC);
	uart_sync();
	ctrl_reset_write(1);
	for(;;) ;
}

void loader_hook_start(void)
{
	struct sockaddr_in addr;
	hook_sock = socket(AF_INET, SOCK_DGRAM, 0);
	if(hook_sock < 0)
		return;
	addr.sin_family      = AF_INET;
	addr.sin_port        = _htons(LOADER_HOOK_PORT);
	addr.sin_addr.s_addr = 0;
	bind(hook_sock, (struct sockaddr *)&addr, sizeof(addr));
}

uint8 loader_hook_socket_cb(SOCKET sock, uint8 u8Msg, void *pvMsg)
{
	if(sock != hook_sock || hook_sock < 0)
		return 0;
	switch(u8Msg) {
	case SOCKET_MSG_BIND:
		recvfrom(hook_sock, hook_rx, sizeof(hook_rx), 0);
		break;
	case SOCKET_MSG_RECVFROM: {
		tstrSocketRecvMsg *m = (tstrSocketRecvMsg *)pvMsg;
		if(m->s16BufferSize >= 4 && memcmp(m->pu8Buffer, "WFLE", 4) == 0) {
			/* ack, give it a moment to drain, then hand over to the loader.
			 * A lost ack is fine: the host polls the loader port anyway. */
			sendto(hook_sock, (void *)"WFLE", 4, 0,
			       (struct sockaddr *)&m->strRemoteAddr,
			       sizeof(m->strRemoteAddr));
			busy_wait(50);
			loader_hook_request();   /* no return */
		}
		recvfrom(hook_sock, hook_rx, sizeof(hook_rx), 0);   /* re-arm */
		break;
	}
	default:
		break;
	}
	return 1;
}
