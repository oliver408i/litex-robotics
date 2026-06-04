#include <stdint.h>
#include <string.h>

#include <irq.h>
#include <libbase/uart.h>
#include <system.h>
#include <generated/csr.h>
#include <generated/soc.h>

#include "driver/include/m2m_wifi.h"
#include "driver/include/m2m_types.h"
#include "socket/include/socket.h"
#include "aux_spi.h"
#include "mdns.h"
#include "loader_hook.h"
#include "wifi_secrets.h"

#ifndef CSR_AUX_SPI_BASE
#error "Build the SoC from icepi_zero_winc.py (provides aux_spi) before compiling software/winc_test."
#endif

extern void winc_service_irq(void);   /* nm_bsp_icepi.c */

/* ---- tiny UART logging (self-contained) -------------------------------- */
static void log_char(char c) { if(c == '\n') uart_write('\r'); uart_write(c); }
static void log_puts(const char *s) { while(*s) log_char(*s++); }
static void log_hex4(uint8_t v) { v &= 0x0f; log_char(v < 10 ? '0' + v : 'a' + v - 10); }
static void log_hex8(uint8_t v) { log_hex4(v >> 4); log_hex4(v); }
static void log_hex16(uint16_t v) { log_hex8(v >> 8); log_hex8(v); }
static void log_hex32(uint32_t v) { log_hex16(v >> 16); log_hex16(v); }
static void log_nl(void) { log_char('\n'); uart_sync(); }

static void log_uint(uint32_t v)
{
	char buf[10];
	unsigned int i = 0;
	if(v == 0) { log_char('0'); return; }
	while(v && i < sizeof(buf)) { buf[i++] = '0' + v % 10; v /= 10; }
	while(i) log_char(buf[--i]);
}

static void log_int(int32_t v)
{
	if(v < 0) { log_char('-'); v = -v; }
	log_uint((uint32_t)v);
}

static void log_ip(uint32_t ip_be)   /* network byte order: first octet = LSB */
{
	log_uint(ip_be & 0xff);         log_char('.');
	log_uint((ip_be >> 8) & 0xff);  log_char('.');
	log_uint((ip_be >> 16) & 0xff); log_char('.');
	log_uint((ip_be >> 24) & 0xff);
}

/* ---- TCP echo server ----------------------------------------------------
 * After DHCP completes we bind+listen on LISTEN_PORT; connect from the host
 * with `nc <board-ip> 5555`. Each accepted client gets a hello banner and an
 * echo of everything it sends. Exercises the full socket TX + RX path the
 * loader will use (the loader will listen the same way). */
#define LISTEN_PORT 5555

static SOCKET srv_sock    = -1;
static SOCKET client_sock = -1;
static uint8  tcp_rx[1460];          /* one full TCP MSS per RECV event */

/* Throughput sink: a client chunk starting with "BLST" + u32le byte-count
 * switches the connection from echo to sink -- count bytes (no echo, no
 * per-chunk UART logging, both would dominate the measurement), reply "DONE"
 * when everything arrived. Host side: winc_throughput.py. */
static uint32_t sink_expected;       /* 0 = echo mode */
static uint32_t sink_got;
static uint32_t sink_events;         /* RECV events while sinking          */
static uint32_t sink_max_chunk;      /* largest single RECV                */
static uint32_t sink_spi_base;       /* aux_spi_xfer_count at sink start   */
static uint32_t sink_sleep_base;     /* winc_sleep_calls at sink start     */
static uint32_t sink_sleepms_base;

extern uint32_t winc_sleep_calls, winc_sleep_ms;   /* nm_bsp_icepi.c */

/* UDP sink on UDP_PORT: counts datagram bytes, replies with the totals when
 * a "USTP" datagram arrives. Bypasses the WINC's TCP stack entirely --
 * isolates raw radio -> WINC -> SPI delivery rate from TCP windowing. */
#define UDP_PORT 5556
static SOCKET   udp_sock = -1;
static uint8    udp_rx[1472];
static uint32_t udp_bytes, udp_dgrams;

static void udp_sink_start(void)
{
	struct sockaddr_in addr;
	udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
	if(udp_sock < 0) return;
	addr.sin_family      = AF_INET;
	addr.sin_port        = _htons(UDP_PORT);
	addr.sin_addr.s_addr = 0;
	bind(udp_sock, (struct sockaddr *)&addr, sizeof(addr));
}

/* Returns 1 if the event belonged to the UDP sink socket. */
static uint8 udp_sink_cb(SOCKET sock, uint8 u8Msg, void *pvMsg)
{
	if(sock != udp_sock || udp_sock < 0)
		return 0;
	switch(u8Msg) {
	case SOCKET_MSG_BIND:
		recvfrom(udp_sock, udp_rx, sizeof(udp_rx), 0);
		log_puts("UDP sink on port "); log_uint(UDP_PORT); log_nl();
		break;
	case SOCKET_MSG_RECVFROM: {
		tstrSocketRecvMsg *m = (tstrSocketRecvMsg *)pvMsg;
		if(m->s16BufferSize > 0) {
			if(m->s16BufferSize >= 4 && memcmp(m->pu8Buffer, "USTP", 4) == 0) {
				uint8 reply[8];
				memcpy(reply, &udp_bytes, 4);
				memcpy(reply + 4, &udp_dgrams, 4);
				sendto(udp_sock, reply, sizeof(reply), 0,
				       (struct sockaddr *)&m->strRemoteAddr,
				       sizeof(m->strRemoteAddr));
				log_puts("udp sink: "); log_uint(udp_bytes);
				log_puts(" B in "); log_uint(udp_dgrams);
				log_puts(" datagrams"); log_nl();
				udp_bytes = udp_dgrams = 0;
			} else {
				udp_bytes += (uint32_t)m->s16BufferSize;
				udp_dgrams++;
			}
		}
		recvfrom(udp_sock, udp_rx, sizeof(udp_rx), 0);
		break;
	}
	default:
		break;
	}
	return 1;
}

static void tcp_listen(void)
{
	struct sockaddr_in addr;

	srv_sock = socket(AF_INET, SOCK_STREAM, 0);
	if(srv_sock < 0) {
		log_puts("socket() failed: "); log_int(srv_sock); log_nl();
		return;
	}
	addr.sin_family      = AF_INET;
	addr.sin_port        = _htons(LISTEN_PORT);
	addr.sin_addr.s_addr = 0;                     /* any */
	bind(srv_sock, (struct sockaddr *)&addr, sizeof(addr));
}

static void socket_cb(SOCKET sock, uint8 u8Msg, void *pvMsg)
{
	if(mdns_socket_cb(sock, u8Msg, pvMsg))   /* mDNS socket's events */
		return;
	if(loader_hook_socket_cb(sock, u8Msg, pvMsg))  /* host "enter loader" */
		return;
	if(udp_sink_cb(sock, u8Msg, pvMsg))      /* UDP throughput sink  */
		return;

	switch(u8Msg) {
	case SOCKET_MSG_BIND: {
		tstrSocketBindMsg *m = (tstrSocketBindMsg *)pvMsg;
		if(m->status == 0) {
			listen(srv_sock, 1);
		} else {
			log_puts("bind failed: "); log_int(m->status); log_nl();
		}
		break;
	}
	case SOCKET_MSG_LISTEN: {
		tstrSocketListenMsg *m = (tstrSocketListenMsg *)pvMsg;
		if(m->status == 0) {
			log_puts("TCP server listening on port "); log_uint(LISTEN_PORT);
			log_puts(" -- connect with `nc <board-ip> 5555`"); log_nl();
		} else {
			log_puts("listen failed: "); log_int(m->status); log_nl();
		}
		break;
	}
	case SOCKET_MSG_ACCEPT: {
		tstrSocketAcceptMsg *m = (tstrSocketAcceptMsg *)pvMsg;
		static const char hello[] =
			"hello from IcePi Zero (ATWINC1500 over aux SPI)\n";
		client_sock = m->sock;
		log_puts("client connected from "); log_ip(m->strAddr.sin_addr.s_addr);
		log_puts(" -- echoing"); log_nl();
		send(client_sock, (void *)hello, sizeof(hello) - 1, 0);
		recv(client_sock, tcp_rx, sizeof(tcp_rx), 0);   /* arm reception */
		break;
	}
	case SOCKET_MSG_RECV: {
		tstrSocketRecvMsg *m = (tstrSocketRecvMsg *)pvMsg;
		if(m->s16BufferSize > 0) {
			uint32_t n = (uint32_t)m->s16BufferSize;
			if(sink_expected == 0 && n >= 8 &&
			   memcmp(m->pu8Buffer, "BLST", 4) == 0) {
				/* enter sink mode: u32le total after the magic */
				sink_expected = (uint32_t)m->pu8Buffer[4]
				              | ((uint32_t)m->pu8Buffer[5] << 8)
				              | ((uint32_t)m->pu8Buffer[6] << 16)
				              | ((uint32_t)m->pu8Buffer[7] << 24);
				sink_got        = n - 8;
				sink_events     = 1;
				sink_max_chunk  = n;
				sink_spi_base   = aux_spi_xfer_count;
				sink_sleep_base = winc_sleep_calls;
				sink_sleepms_base = winc_sleep_ms;
				log_puts("throughput sink: expecting ");
				log_uint(sink_expected); log_puts(" B"); log_nl();
			} else if(sink_expected != 0) {
				sink_got += n;            /* bulk path: count only */
				sink_events++;
				if(n > sink_max_chunk) sink_max_chunk = n;
			} else {
				log_puts("rx "); log_int(m->s16BufferSize); log_puts(" B: ");
				for(sint16 i = 0; i < m->s16BufferSize; i++)
					log_char((char)m->pu8Buffer[i]);
				log_nl();
				send(sock, m->pu8Buffer, (uint16)m->s16BufferSize, 0);  /* echo */
			}
			if(sink_expected != 0 && sink_got >= sink_expected) {
				log_puts("sink done: "); log_uint(sink_got);
				log_puts(" B in "); log_uint(sink_events);
				log_puts(" events (avg "); log_uint(sink_got / sink_events);
				log_puts(" B, max "); log_uint(sink_max_chunk);
				log_puts(")"); log_nl();
				log_puts("  spi bytes: ");
				log_uint(aux_spi_xfer_count - sink_spi_base);
				log_puts(", sleeps: ");
				log_uint(winc_sleep_calls - sink_sleep_base);
				log_puts(" ("); log_uint(winc_sleep_ms - sink_sleepms_base);
				log_puts(" ms total)"); log_nl();
				send(sock, (void *)"DONE", 4, 0);
				sink_expected = sink_got = 0;   /* back to echo mode */
			}
			recv(sock, tcp_rx, sizeof(tcp_rx), 0);   /* re-arm */
		} else {
			log_puts("client closed / error: "); log_int(m->s16BufferSize);
			log_puts(" -- still listening"); log_nl();
			close(sock);                  /* server socket keeps listening */
			client_sock = -1;
			sink_expected = sink_got = 0;
		}
		break;
	}
	case SOCKET_MSG_SEND:
		break;   /* sent-bytes ack; uninteresting for the echo test */
	default:
		log_puts("  [sock cb] msg="); log_uint(u8Msg); log_nl();
		break;
	}
}

/* ---- Wi-Fi event callback ----------------------------------------------- */
static uint8 scan_idx, scan_total;

static void wifi_connect(void)
{
	log_puts("connecting to \""); log_puts(WIFI_SSID); log_puts("\" (WPA2)..."); log_nl();
	m2m_wifi_connect((char *)WIFI_SSID, (uint8)strlen(WIFI_SSID),
	                 M2M_WIFI_SEC_WPA_PSK, (void *)WIFI_PSK, M2M_WIFI_CH_ALL);
}

static void wifi_cb(uint8 u8MsgType, void *pvMsg)
{
	switch(u8MsgType) {
	case M2M_WIFI_RESP_CON_STATE_CHANGED: {
		tstrM2mWifiStateChanged *s = (tstrM2mWifiStateChanged *)pvMsg;
		if(s->u8CurrState == M2M_WIFI_CONNECTED) {
			log_puts("wifi: link up, waiting for DHCP..."); log_nl();
		} else {
			log_puts("wifi: disconnected (err "); log_int(s->u8ErrCode);
			log_puts("), retrying..."); log_nl();
			if(client_sock >= 0) { close(client_sock); client_sock = -1; }
			if(srv_sock    >= 0) { close(srv_sock);    srv_sock    = -1; }
			wifi_connect();
		}
		break;
	}
	case M2M_WIFI_REQ_DHCP_CONF: {
		tstrM2MIPConfig *ip = (tstrM2MIPConfig *)pvMsg;
		log_puts("wifi: DHCP done, IP "); log_ip(ip->u32StaticIP); log_nl();
		tcp_listen();
		udp_sink_start();
		mdns_start(ip->u32StaticIP);   /* board reachable as icepi.local */
		loader_hook_start();           /* flash.py can request the loader */
		break;
	}
	case M2M_WIFI_RESP_SCAN_DONE:
		scan_total = (uint8)m2m_wifi_get_num_ap_found();
		scan_idx   = 0;
		log_puts("scan done, APs found: "); log_uint(scan_total); log_nl();
		if(scan_total > 0)
			m2m_wifi_req_scan_result(0);
		break;
	case M2M_WIFI_RESP_SCAN_RESULT: {
		tstrM2mWifiscanResult *r = (tstrM2mWifiscanResult *)pvMsg;
		log_puts("  ["); log_uint(r->u8index); log_puts("] ch=");
		log_uint(r->u8ch); log_puts(" rssi="); log_int(r->s8rssi);
		log_puts(" ssid="); log_puts((const char *)r->au8SSID); log_nl();
		if(++scan_idx < scan_total) {
			m2m_wifi_req_scan_result(scan_idx);
		} else {
			log_puts("scan list complete"); log_nl();
		}
		break;
	}
	default:
		log_puts("  [wifi cb] msg type = "); log_uint(u8MsgType); log_nl();
		break;
	}
}

int main(void)
{
	uart_init();
#ifdef CONFIG_CPU_HAS_INTERRUPT
	irq_setie(1);
#endif

	log_puts("ATWINC1500 Wi-Fi bring-up (shared aux SPI bus)"); log_nl();
	log_puts("CSR uart       base=0x"); log_hex32(CSR_UART_BASE);        log_nl();
	log_puts("CSR aux_spi    base=0x"); log_hex32(CSR_AUX_SPI_BASE);     log_nl();
	log_puts("sys clk="); log_uint(CONFIG_CLOCK_FREQUENCY); log_puts(" Hz"); log_nl();
	log_puts("winc spi (initial)="); log_uint(WINC_SPI_DEFAULT_FREQUENCY);
	log_puts(" Hz, div="); log_uint(AUX_SPI_DIV_FOR_HZ(WINC_SPI_DEFAULT_FREQUENCY)); log_nl();

	/* Bus-health reference: LSM6DS3 WHO_AM_I via the same AuxSPIMaster, just
	 * a different chip-select. Pass = sclk/mosi/miso + SPI core are good, so a
	 * WINC failure is on the WINC side (power, CS/RST/IRQ wiring, fw). */
	aux_spi_select(&AUX_IMU);
	(void)aux_spi_xfer8(0x80 | 0x0F);            /* READ | WHO_AM_I */
	uint8_t who = aux_spi_xfer8(0x00);
	aux_spi_deselect();
	log_puts("IMU WHO_AM_I = 0x"); log_hex8(who);
	if(who == 0x69 || who == 0x6a)
		log_puts("  -- OK, shared SPI bus is healthy");
	else
		log_puts("  -- BAD (expect 0x69/0x6a): shared bus problem, not WINC-specific");
	log_nl();

	/* SPI HAL speed self-test: clock 100k dummy bytes (all CS high, slaves
	 * ignore; WINC still held in reset) at the WINC divider and time them
	 * with timer0 (free at this point -- nothing has called busy_wait). This
	 * is the raw ceiling of the byte-banged HAL, independent of the WINC. */
	{
		const uint32_t N = 100000;
		aux_spi_clk_divider_write(AUX_WINC.div);
		timer0_en_write(0);
		timer0_reload_write(0);
		timer0_load_write(0xffffffff);
		timer0_en_write(1);
		timer0_update_value_write(1);
		uint32_t t0 = timer0_value_read();
		for(uint32_t i = 0; i < N; i++)
			(void)aux_spi_xfer8(0xff);
		timer0_update_value_write(1);
		uint32_t cycles = t0 - timer0_value_read();
		timer0_en_write(0);
		uint32_t bps = (uint32_t)(((uint64_t)N * CONFIG_CLOCK_FREQUENCY) / cycles);
		log_puts("SPI HAL self-test: "); log_uint(bps);
		log_puts(" B/s raw ("); log_uint(cycles / N);
		log_puts(" cycles/byte at div="); log_uint(AUX_WINC.div);
		log_puts(")"); log_nl();
	}

	/* Keypress gate: 's' runs an AP scan first (link-layer sanity), anything
	 * else goes straight to connect -> DHCP -> TCP echo. */
	log_puts("press a key to start ('s' = scan first)..."); log_nl();
	int key = uart_read();

	nm_bsp_init();

	tstrWifiInitParam param;
	memset(&param, 0, sizeof(param));
	param.pfAppWifiCb = wifi_cb;

	log_puts("m2m_wifi_init()..."); log_nl();
	sint8 ret = m2m_wifi_init(&param);
	if(ret == M2M_SUCCESS) {
		log_puts("WINC init OK"); log_nl();
		/* No 802.11 power save: with PS on, the chip sleeps between beacons
		 * and every TCP data/ACK exchange waits for a wake window -- measured
		 * ~38 ms/segment (0.04 MB/s!). Throughput >> power for this project. */
		m2m_wifi_set_sleep_mode(M2M_NO_PS, 0);
		socketInit();
		registerSocketCallback(socket_cb, NULL);
		if(key == 's')
			m2m_wifi_request_scan(M2M_WIFI_CH_ALL);   /* then connect manually */
		else
			wifi_connect();
	} else {
		log_puts("WINC init FAILED, ret="); log_int(ret); log_nl();
		log_puts("  check power, CS/RESET_N/EN/IRQ wiring, and WINC fw version"); log_nl();
	}

	/* Service the (polled) WINC IRQ + pump the host driver event loop. */
	for(;;) {
		winc_service_irq();
		m2m_wifi_handle_events(NULL);
	}

	return 0;
}
