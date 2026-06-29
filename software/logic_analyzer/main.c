/* IcePi Zero logic-analyzer firmware.
 *
 * WINC1500 networking scaffold (forked from software/winc_test) + a small TCP
 * control server (port 5559) that drives the LogicAnalyzer CSRs and streams the
 * SDRAM capture ring back to the host. Protocol + design: docs/logic_analyzer.md.
 *
 * The host (software/logic_analyzer/la_host.py) speaks the protocol below;
 * flash.py still works over WiFi because the WINC aux bus + loader_hook are
 * present exactly as in every WINC build.
 */
#include <stdint.h>
#include <string.h>

#include <irq.h>
#include <libbase/uart.h>
#include <system.h>
#include <generated/csr.h>
#include <generated/soc.h>
#include <generated/mem.h>

#include "driver/include/m2m_wifi.h"
#include "driver/include/m2m_types.h"
#include "socket/include/socket.h"
#include "aux_spi.h"
#include "mdns.h"
#include "loader_hook.h"
#include "wifi_secrets.h"

#ifndef CSR_LA_BASE
#error "Build the SoC from icepi_zero_la.py (provides the `la` CSRs) before compiling software/logic_analyzer."
#endif

extern void winc_service_irq(void);   /* nm_bsp_icepi.c */

/* ---- capture ring in SDRAM ----------------------------------------------
 * .bss/.stack live in the 8 KiB sram; the program's code/rodata sit at the
 * bottom of main_ram (SDRAM). So we can't use a C array for a multi-MiB ring --
 * we point it at a fixed high SDRAM offset, well clear of the code, and hand
 * the LogicAnalyzer a byte offset (it addresses SDRAM as native words; offset 0
 * == MAIN_RAM_BASE). 8 MiB offset + 16 MiB ring fits the 32 MiB part with room. */
#define LA_RING_OFFSET   0x00800000u                 /* 8 MiB into SDRAM */
#define LA_RING_SAMPLES  (4u * 1024 * 1024)          /* 16 MiB / 4 B = 4 Mi samples */
#define LA_RING_PTR      ((volatile uint32_t *)(MAIN_RAM_BASE + LA_RING_OFFSET))

/* status field bit positions (must match the CSRStatus field order) */
#define ST_RUNNING     (1u << 0)
#define ST_TRIGGERED   (1u << 1)
#define ST_DONE        (1u << 2)
#define ST_OVERRUN     (1u << 3)
#define ST_WRAPPED     (1u << 4)
#define ST_LCD_PRESENT (1u << 5)

/* ---- tiny UART logging --------------------------------------------------- */
static void log_char(char c) { if(c == '\n') uart_write('\r'); uart_write(c); }
static void log_puts(const char *s) { while(*s) log_char(*s++); }
static void log_nl(void) { log_char('\n'); uart_sync(); }
static void log_uint(uint32_t v)
{
	char buf[10]; unsigned int i = 0;
	if(v == 0) { log_char('0'); return; }
	while(v && i < sizeof(buf)) { buf[i++] = '0' + v % 10; v /= 10; }
	while(i) log_char(buf[--i]);
}
static void log_int(int32_t v) { if(v < 0) { log_char('-'); v = -v; } log_uint((uint32_t)v); }
static void log_ip(uint32_t ip) {
	log_uint(ip & 0xff); log_char('.'); log_uint((ip >> 8) & 0xff); log_char('.');
	log_uint((ip >> 16) & 0xff); log_char('.'); log_uint((ip >> 24) & 0xff);
}

/* ---- LA TCP control server ---------------------------------------------- */
#define LISTEN_PORT 5559
#define DUMP_CHUNK_SAMPLES 350           /* 350*4 = 1400 B, one WINC TCP segment */

static SOCKET srv_sock    = -1;
static SOCKET client_sock = -1;
static uint8  cmd_rx[64];                /* commands are tiny request/response */

/* in-flight ring download (driven by SOCKET_MSG_SEND) */
static uint32_t dump_next, dump_end;
static uint8    dumping;

static uint32_t rd32(const uint8 *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
	     | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void dump_send_chunk(SOCKET s)
{
	uint32_t remain, n;
	if(!dumping || dump_next >= dump_end) { dumping = 0; return; }
	remain = dump_end - dump_next;
	n = remain > DUMP_CHUNK_SAMPLES ? DUMP_CHUNK_SAMPLES : remain;
	/* Stream straight out of SDRAM; caches were flushed at dump start. */
	send(s, (void *)&LA_RING_PTR[dump_next], (uint16)(n * 4), 0);
	dump_next += n;
	if(dump_next >= dump_end) dumping = 0;
}

static void handle_cmd(SOCKET s, const uint8 *p, uint32_t len)
{
	if(len < 4) return;

	if(memcmp(p, "LNFO", 4) == 0) {
		uint8 r[17];
		uint32_t v;
		memcpy(r, "LA01", 4);
		v = LA_N_CHANNELS;   memcpy(r + 4,  &v, 4);
		v = LA_RING_SAMPLES; memcpy(r + 8,  &v, 4);
		v = CONFIG_CLOCK_FREQUENCY; memcpy(r + 12, &v, 4);
		r[16] = (la_status_read() & ST_LCD_PRESENT) ? 1 : 0;
		send(s, r, sizeof(r), 0);

	} else if(memcmp(p, "LCFG", 4) == 0 && len >= 25) {
		uint32_t ring_size = rd32(p + 17);
		if(ring_size == 0 || ring_size > LA_RING_SAMPLES)
			ring_size = LA_RING_SAMPLES;
		la_sample_div_write(rd32(p + 4));
		la_trig_mask_write (rd32(p + 8));
		la_trig_value_write(rd32(p + 12));
		la_trig_edge_write (p[16] ? 1 : 0);
		la_ring_base_write (LA_RING_OFFSET);          /* firmware owns the buffer */
		la_ring_size_write (ring_size);
		la_post_trig_write (rd32(p + 21));
		send(s, (void *)"LOK", 4, 0);                 /* includes the NUL */

	} else if(memcmp(p, "LARM", 4) == 0) {
		if(la_status_read() & ST_LCD_PRESENT) {
			log_puts("LARM refused: LCD/touch module still attached"); log_nl();
			send(s, (void *)"LBSY", 4, 0);
		} else {
			la_arm_write(1);
			send(s, (void *)"LOK", 4, 0);
		}

	} else if(memcmp(p, "LSTA", 4) == 0) {
		uint8 r[13];
		uint32_t v;
		r[0] = (uint8)la_status_read();
		v = la_wr_count_read();  memcpy(r + 1, &v, 4);
		v = la_wptr_read();      memcpy(r + 5, &v, 4);
		v = la_trig_addr_read(); memcpy(r + 9, &v, 4);
		send(s, r, sizeof(r), 0);

	} else if(memcmp(p, "LDMP", 4) == 0 && len >= 12) {
		uint32_t start = rd32(p + 4);
		uint32_t n     = rd32(p + 8);
		if(start > LA_RING_SAMPLES) start = LA_RING_SAMPLES;
		if(n > LA_RING_SAMPLES - start) n = LA_RING_SAMPLES - start;
		/* The DMA wrote the ring straight to SDRAM, bypassing the CPU caches.
		 * Flush so the bytes we read (and ship) are the captured data, not stale
		 * cache lines. */
		flush_cpu_dcache();
		flush_l2_cache();
		dump_next = start;
		dump_end  = start + n;
		dumping   = 1;
		dump_send_chunk(s);

	} else if(memcmp(p, "LABT", 4) == 0) {
		la_abort_write(1);
		dumping = 0;
		send(s, (void *)"LOK", 4, 0);
	}
}

static void tcp_listen(void)
{
	struct sockaddr_in addr;
	srv_sock = socket(AF_INET, SOCK_STREAM, 0);
	if(srv_sock < 0) { log_puts("socket() failed: "); log_int(srv_sock); log_nl(); return; }
	addr.sin_family      = AF_INET;
	addr.sin_port        = _htons(LISTEN_PORT);
	addr.sin_addr.s_addr = 0;
	bind(srv_sock, (struct sockaddr *)&addr, sizeof(addr));
}

static void socket_cb(SOCKET sock, uint8 u8Msg, void *pvMsg)
{
	if(mdns_socket_cb(sock, u8Msg, pvMsg))         return;
	if(loader_hook_socket_cb(sock, u8Msg, pvMsg))  return;

	switch(u8Msg) {
	case SOCKET_MSG_BIND: {
		tstrSocketBindMsg *m = (tstrSocketBindMsg *)pvMsg;
		if(m->status == 0) listen(srv_sock, 1);
		else { log_puts("bind failed: "); log_int(m->status); log_nl(); }
		break;
	}
	case SOCKET_MSG_LISTEN:
		log_puts("LA server listening on port "); log_uint(LISTEN_PORT); log_nl();
		break;
	case SOCKET_MSG_ACCEPT: {
		tstrSocketAcceptMsg *m = (tstrSocketAcceptMsg *)pvMsg;
		client_sock = m->sock;
		dumping = 0;
		log_puts("LA client connected from "); log_ip(m->strAddr.sin_addr.s_addr); log_nl();
		recv(client_sock, cmd_rx, sizeof(cmd_rx), 0);
		break;
	}
	case SOCKET_MSG_RECV: {
		tstrSocketRecvMsg *m = (tstrSocketRecvMsg *)pvMsg;
		if(m->s16BufferSize > 0) {
			handle_cmd(sock, m->pu8Buffer, (uint32_t)m->s16BufferSize);
			recv(sock, cmd_rx, sizeof(cmd_rx), 0);   /* re-arm for next command */
		} else {
			close(sock);
			client_sock = -1;
			dumping = 0;
		}
		break;
	}
	case SOCKET_MSG_SEND:
		/* a queued buffer drained: push the next ring chunk if downloading */
		if(dumping) dump_send_chunk(sock);
		break;
	default:
		break;
	}
}

/* ---- Wi-Fi event callback ----------------------------------------------- */
static void wifi_connect(void)
{
	log_puts("connecting to \""); log_puts(WIFI_SSID); log_puts("\"..."); log_nl();
	m2m_wifi_connect((char *)WIFI_SSID, (uint8)strlen(WIFI_SSID),
	                 M2M_WIFI_SEC_WPA_PSK, (void *)WIFI_PSK, M2M_WIFI_CH_ALL);
}

static void wifi_cb(uint8 u8MsgType, void *pvMsg)
{
	switch(u8MsgType) {
	case M2M_WIFI_RESP_CON_STATE_CHANGED: {
		tstrM2mWifiStateChanged *st = (tstrM2mWifiStateChanged *)pvMsg;
		if(st->u8CurrState == M2M_WIFI_CONNECTED) {
			log_puts("wifi: link up, waiting for DHCP..."); log_nl();
		} else {
			log_puts("wifi: disconnected (err "); log_int(st->u8ErrCode);
			log_puts("), retrying..."); log_nl();
			if(client_sock >= 0) { close(client_sock); client_sock = -1; }
			if(srv_sock    >= 0) { close(srv_sock);    srv_sock    = -1; }
			dumping = 0;
			wifi_connect();
		}
		break;
	}
	case M2M_WIFI_REQ_DHCP_CONF: {
		tstrM2MIPConfig *ip = (tstrM2MIPConfig *)pvMsg;
		log_puts("wifi: DHCP done, IP "); log_ip(ip->u32StaticIP); log_nl();
		tcp_listen();
		mdns_start(ip->u32StaticIP);   /* reachable as icepi.local */
		loader_hook_start();           /* flash.py can request the loader */
		break;
	}
	default:
		break;
	}
}

int main(void)
{
	sint8 ret;
	tstrWifiInitParam param;

	uart_init();
#ifdef CONFIG_CPU_HAS_INTERRUPT
	irq_setie(1);
#endif

	log_puts("IcePi Zero logic analyzer"); log_nl();
	log_puts("channels="); log_uint(LA_N_CHANNELS);
	log_puts(" ring="); log_uint(LA_RING_SAMPLES); log_puts(" samples @ off 0x");
	log_uint(LA_RING_OFFSET); log_nl();

	if(la_status_read() & ST_LCD_PRESENT)
		log_puts("WARNING: LCD/touch module appears attached -- unplug it before capturing.");
	else
		log_puts("guard: no LCD/touch module detected -- OK to capture.");
	log_nl();

	nm_bsp_init();
	memset(&param, 0, sizeof(param));
	param.pfAppWifiCb = wifi_cb;

	log_puts("m2m_wifi_init()..."); log_nl();
	ret = m2m_wifi_init(&param);
	if(ret == M2M_SUCCESS) {
		log_puts("WINC init OK"); log_nl();
		m2m_wifi_set_sleep_mode(M2M_NO_PS, 0);   /* throughput >> power */
		socketInit();
		registerSocketCallback(socket_cb, NULL);
		wifi_connect();
	} else {
		log_puts("WINC init FAILED, ret="); log_int(ret); log_nl();
	}

	for(;;) {
		winc_service_irq();
		m2m_wifi_handle_events(NULL);
	}
	return 0;
}
