/* WiFi flash-loader + boot manager. The BIOS flash-boots this image from
 * FLASH_BOOT (0x200000); it either chain-boots the app at FLASH_APP_OFFSET
 * or stays resident as the WiFi flasher (WFL protocol on UDP :5557, host
 * side: ./flash.py). Boot triage order, protocol tables, entry paths and
 * all design rationale: docs/boot_chain.md. */
#include <stdint.h>
#include <string.h>

#include <irq.h>
#include <libbase/uart.h>
#include <libbase/crc.h>
#include <system.h>
#include <generated/csr.h>
#include <generated/soc.h>
#include <generated/mem.h>

#include "driver/include/m2m_wifi.h"
#include "driver/include/m2m_types.h"
#include "socket/include/socket.h"
#include "aux_spi.h"
#include "mdns.h"
#include "loader_hook.h"   /* LOADER_BOOT_MAGIC (shared with apps' hook) */
#include "wifi_secrets.h"
#include "flash_w25q.h"

#ifndef CSR_AUX_SPI_BASE
#error "Build the SoC from icepi_zero_winc.py --flash-master before compiling."
#endif

extern void winc_service_irq(void);   /* nm_bsp_icepi.c */

#define LOADER_PORT 5557

#ifndef FLASH_APP_OFFSET   /* soc.h constant when the SoC provides it */
#define FLASH_APP_OFFSET 0x280000
#endif

/* SRAM-resident copy stub (chain_stub.S) -- never returns. */
extern void chain_stub(const void *src, void *dst, uint32_t len, uint32_t entry);

/* SDRAM layout: app text/data live at the bottom of main_ram (~50 KB); the
 * image buffer and chunk bitmap sit far above them. Not in .bss -- that lands
 * in the 8 KB sram next to the stack. */
#define IMG_BUF       ((uint8_t *)(MAIN_RAM_BASE + 0x00800000))
#define IMG_MAX_LEN   (MAIN_RAM_SIZE - 0x00800000 - 0x00400000) /* keep 4 MB headroom */
#define EXEC_MAX_LEN  0x00800000   /* WFLX copies IMG_BUF down to 0; the app
                                      must end below IMG_BUF or the copy would
                                      eat its own source */
#define CHUNK_MAP     ((uint8_t *)(MAIN_RAM_BASE + 0x007f0000)) /* 64 KB below buffer */
#define CHUNK_MIN     512
#define CHUNK_MAX     1408   /* fits a 1472 B datagram with the 8 B header */
#define MISS_MAX      300    /* missing indices per WFLT (1216 B < UDP send cap) */

/* status codes in WFLA/WFLZ */
#define ST_OK          0
#define ST_BAD_ARGS    1
#define ST_INCOMPLETE  2
#define ST_CRC_SDRAM   3
#define ST_VERIFY_FAIL 4
#define ST_NO_SESSION  5

/* ---- tiny UART logging (same helpers as winc_test) ---------------------- */
static void log_char(char c) { if(c == '\n') uart_write('\r'); uart_write(c); }
static void log_puts(const char *s) { while(*s) log_char(*s++); }
static void log_hex4(uint8_t v) { v &= 0x0f; log_char(v < 10 ? '0' + v : 'a' + v - 10); }
static void log_hex8(uint8_t v) { log_hex4(v >> 4); log_hex4(v); }
static void log_hex32(uint32_t v) { log_hex8(v >> 24); log_hex8(v >> 16); log_hex8(v >> 8); log_hex8(v); }
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

/* ---- timer0 stopwatch (free: nothing sleeps while we program) ----------- */
static void sw_start(void)
{
	timer0_en_write(0);
	timer0_reload_write(0);
	timer0_load_write(0xffffffff);
	timer0_en_write(1);
}

static uint32_t sw_elapsed_ms(void)
{
	timer0_update_value_write(1);
	uint32_t cycles = 0xffffffff - timer0_value_read();
	return (uint32_t)(((uint64_t)cycles * 1000) / CONFIG_CLOCK_FREQUENCY);
}

/* ---- boot-manager triage --------------------------------------------------
 * Stay in loader mode (flag / FTDI level / 'l' key) or chain-boot the app;
 * an invalid app image always falls back to loader mode. docs/boot_chain.md. */

static int boot_flag_requested(void)
{
#ifdef CSR_BOOT_CTL_FLAG_ADDR
	if(boot_ctl_flag_read() == LOADER_BOOT_MAGIC) {
		boot_ctl_flag_write(0);
		return 1;
	}
#endif
	return 0;
}

static int ftdi_stay_requested(void)
{
#ifdef CSR_FTDI_SENSE_BASE
	/* stay = DTR asserted (bit0 low) + RTS deasserted (bit1 high);
	 * both-asserted is just an open port and must not trap boots. */
	uint32_t v = ftdi_sense_in_read();
	return ((v & 1u) == 0) && ((v & 2u) != 0);
#else
	return 0;
#endif
}

static int grace_window_hit(uint32_t window_ms)
{
	/* 'l' = enter loader; avoids 'Q'/ESC (BIOS abort keys). */
	sw_start();
	while(sw_elapsed_ms() < window_ms) {
		if(uart_read_nonblock()) {
			char c = uart_read();
			if(c == 'l' || c == 'L')
				return 1;
		}
	}
	return 0;
}

/* Validate + chain-boot the app .fbi; returns only if there is no valid
 * image. The copy runs from the SRAM-resident chain_stub because the app
 * lands exactly where this loader executes. */
static void try_chain_boot(void)
{
	const uint8_t *img = (const uint8_t *)(SPIFLASH_BASE + FLASH_APP_OFFSET);

	flush_cpu_dcache();   /* the slot may have just been reflashed */
	uint32_t len = ((const volatile uint32_t *)img)[0];
	uint32_t crc = ((const volatile uint32_t *)img)[1];

	if(len == 0 || len == 0xffffffff ||
	   len > SPIFLASH_SIZE - FLASH_APP_OFFSET - 8) {
		log_puts("no app image at flash 0x"); log_hex32(FLASH_APP_OFFSET);
		log_puts(" -- staying in loader"); log_nl();
		return;
	}
	if(crc32(img + 8, len) != crc) {
		log_puts("app image CRC mismatch -- staying in loader"); log_nl();
		return;
	}

	log_puts("chain-booting app ("); log_uint(len);
	log_puts(" B @0x"); log_hex32(FLASH_APP_OFFSET); log_puts(")"); log_nl();
	/* visual separator (a la the BIOS "Liftoff!"): everything below is app */
	log_puts("--============== \e[1mapp\e[0m ===============--"); log_nl();
	uart_sync();
#ifdef CONFIG_CPU_HAS_INTERRUPT
	irq_setmask(0);
	irq_setie(0);
#endif
	flush_cpu_icache();
	flush_cpu_dcache();
	chain_stub(img + 8, (void *)MAIN_RAM_BASE, (len + 3) & ~3u, MAIN_RAM_BASE);
	__builtin_unreachable();
}

/* ---- transfer session ---------------------------------------------------- */
static SOCKET ldr_sock = -1;
static uint8  ldr_rx[1472];

static uint8_t  session;                /* WFLB accepted                     */
static uint32_t img_off, img_len, img_crc;
static uint32_t chunk_sz, total_chunks, got_chunks;

static uint8_t  prog_pending;           /* WFLP accepted, run from main loop */
static uint8_t  prog_done;              /* result cached in prog_* below     */
static uint8_t  prog_status;
static uint32_t prog_ms, prog_flash_crc;
static struct sockaddr_in prog_addr;    /* who to answer when programming ends */

static uint8_t  reboot_pending;

static uint8_t  exec_pending;           /* WFLX accepted, run from main loop */
static struct sockaddr_in exec_addr;    /* who to ack just before the jump   */

static uint32_t rd_u32(const uint8 *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
	     | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void wr_u32(uint8 *p, uint32_t v)
{
	p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24;
}

static void ldr_reply(const struct sockaddr_in *to, uint8 *buf, uint16 len)
{
	sendto(ldr_sock, buf, len, 0, (struct sockaddr *)to, sizeof(*to));
}

static void send_wfla(const struct sockaddr_in *to, uint8_t status)
{
	uint8 r[16];
	memcpy(r, "WFLA", 4);
	r[4] = status; r[5] = 1 /* proto version */; r[6] = r[7] = 0;
	wr_u32(r + 8,  SPIFLASH_SIZE);
	wr_u32(r + 12, IMG_MAX_LEN);
	ldr_reply(to, r, sizeof(r));
}

static void send_wflz(const struct sockaddr_in *to)
{
	uint8 r[16];
	memcpy(r, "WFLZ", 4);
	r[4] = prog_status; r[5] = r[6] = r[7] = 0;
	wr_u32(r + 8,  prog_ms);
	wr_u32(r + 12, prog_flash_crc);
	ldr_reply(to, r, sizeof(r));
}

static void handle_begin(const uint8 *p, sint16 n, const struct sockaddr_in *from)
{
	if(n < 18) return;
	uint32_t off = rd_u32(p + 4), len = rd_u32(p + 8), crc = rd_u32(p + 12);
	uint32_t chk = (uint32_t)p[16] | ((uint32_t)p[17] << 8);

	if(session && off == img_off && len == img_len && crc == img_crc &&
	   chk == chunk_sz) {
		send_wfla(from, ST_OK);   /* retry/resume of the same image */
		return;
	}
	if(len == 0 || len > IMG_MAX_LEN ||
	   off + len > SPIFLASH_SIZE || off + len < off ||
	   (off & (FLASH_SECTOR_SIZE - 1)) != 0 ||
	   chk < CHUNK_MIN || chk > CHUNK_MAX) {
		send_wfla(from, ST_BAD_ARGS);
		return;
	}

	img_off  = off;  img_len = len;  img_crc = crc;  chunk_sz = chk;
	total_chunks = (len + chk - 1) / chk;
	got_chunks   = 0;
	memset(CHUNK_MAP, 0, (total_chunks + 7) / 8);
	session      = 1;
	prog_pending = prog_done = 0;

	log_puts("begin: "); log_uint(img_len);
	log_puts(" B -> flash 0x"); log_hex32(img_off);
	log_puts(" ("); log_uint(total_chunks); log_puts(" chunks of ");
	log_uint(chunk_sz); log_puts(" B)"); log_nl();
	send_wfla(from, ST_OK);
}

static void handle_data(const uint8 *p, sint16 n)
{
	if(!session || prog_pending || n < 9)
		return;
	uint32_t idx = rd_u32(p + 4);
	if(idx >= total_chunks)
		return;
	uint32_t expect = (idx == total_chunks - 1)
	                ? img_len - (total_chunks - 1) * chunk_sz : chunk_sz;
	if((uint32_t)(n - 8) != expect)
		return;
	memcpy(IMG_BUF + idx * chunk_sz, p + 8, expect);
	if(!(CHUNK_MAP[idx >> 3] & (1u << (idx & 7)))) {
		CHUNK_MAP[idx >> 3] |= 1u << (idx & 7);
		got_chunks++;
	}
}

static void handle_stat(const struct sockaddr_in *from)
{
	static uint8 r[16 + 4 * MISS_MAX];
	uint32_t nmiss = 0;

	if(session)
		for(uint32_t i = 0; i < total_chunks && nmiss < MISS_MAX; i++)
			if(!(CHUNK_MAP[i >> 3] & (1u << (i & 7))))
				wr_u32(r + 16 + 4 * nmiss++, i);

	memcpy(r, "WFLT", 4);
	wr_u32(r + 4, got_chunks);
	wr_u32(r + 8, session ? total_chunks : 0);
	r[12] = nmiss; r[13] = nmiss >> 8; r[14] = r[15] = 0;
	ldr_reply(from, r, (uint16)(16 + 4 * nmiss));
}

static void handle_prog(const struct sockaddr_in *from)
{
	if(prog_done) {                 /* idempotent: re-answer host retries */
		send_wflz(from);
		return;
	}
	prog_status = ST_OK; prog_ms = 0; prog_flash_crc = 0;
	if(!session) {
		prog_status = ST_NO_SESSION;
	} else if(got_chunks < total_chunks) {
		prog_status = ST_INCOMPLETE;
	} else if(!prog_pending) {
		prog_addr    = *from;
		prog_pending = 1;           /* heavy lifting happens in main loop */
		return;
	} else {
		return;                     /* already queued */
	}
	send_wflz(from);                /* error path only */
}

static void send_wflx(const struct sockaddr_in *to, uint8_t status)
{
	uint8 r[8];
	memcpy(r, "WFLX", 4);
	r[4] = status; r[5] = r[6] = r[7] = 0;
	ldr_reply(to, r, sizeof(r));
}

static void handle_exec(const struct sockaddr_in *from)
{
	if(exec_pending || (prog_pending && !prog_done))
		return;                     /* already queued / flash op in flight */
	if(!session) {
		send_wflx(from, ST_NO_SESSION);
	} else if(got_chunks < total_chunks) {
		send_wflx(from, ST_INCOMPLETE);
	} else if(img_len > EXEC_MAX_LEN) {
		send_wflx(from, ST_BAD_ARGS);
	} else {
		exec_addr    = *from;
		exec_pending = 1;           /* CRC + chain happen in main loop */
	}
}

static void handle_reboot(const struct sockaddr_in *from)
{
	uint8 r[4];
	memcpy(r, "WFLR", 4);
	ldr_reply(from, r, sizeof(r));
	reboot_pending = 1;             /* reset after the reply drains */
	log_puts("reboot requested"); log_nl();
}

/* Erase + program + verify; main-loop context only (runs for seconds with
 * the WINC unserviced -- fine, see docs/boot_chain.md). */
static void run_program(void)
{
	log_puts("image CRC check..."); log_nl();
	uint32_t c = crc32(IMG_BUF, img_len);
	if(c != img_crc) {
		log_puts("SDRAM image CRC mismatch: got 0x"); log_hex32(c);
		log_puts(" want 0x"); log_hex32(img_crc); log_nl();
		prog_status = ST_CRC_SDRAM;
		prog_done   = 1;
		send_wflz(&prog_addr);
		return;
	}

	sw_start();
	log_puts("erase 0x"); log_hex32(img_off); log_puts(" len ");
	log_uint(img_len); log_puts("..."); log_nl();
	flash_erase_range(img_off, img_len);
	uint32_t t_erase = sw_elapsed_ms();

	log_puts("program..."); log_nl();
	flash_program(img_off, IMG_BUF, img_len);
	uint32_t t_prog = sw_elapsed_ms();

	/* Readback verify through the flash mmap (quad XIP path) -- stale lines
	 * from before the erase must go. */
	flush_cpu_dcache();
	prog_flash_crc = crc32((const unsigned char *)(SPIFLASH_BASE + img_off), img_len);
	prog_ms     = sw_elapsed_ms();
	prog_status = (prog_flash_crc == img_crc) ? ST_OK : ST_VERIFY_FAIL;
	prog_done   = 1;

	log_puts(prog_status == ST_OK ? "verify OK" : "VERIFY FAILED");
	log_puts(" (erase "); log_uint(t_erase);
	log_puts(" ms, program "); log_uint(t_prog - t_erase);
	log_puts(" ms, verify "); log_uint(prog_ms - t_prog);
	log_puts(" ms)"); log_nl();
	send_wflz(&prog_addr);
}

/* WFLX: run the staged SDRAM image directly -- nothing flashed, no reset.
 * Same exit sequence as try_chain_boot, but the source is IMG_BUF instead of
 * the flash mmap; returns only on a CRC mismatch. The app vanishes on the
 * next reset (the loader then chain-boots whatever is in the flash slot). */
static void run_exec(void)
{
	exec_pending = 0;
	log_puts("exec: image CRC check..."); log_nl();
	uint32_t c = crc32(IMG_BUF, img_len);
	if(c != img_crc) {
		log_puts("SDRAM image CRC mismatch: got 0x"); log_hex32(c);
		log_puts(" want 0x"); log_hex32(img_crc); log_nl();
		send_wflx(&exec_addr, ST_CRC_SDRAM);
		return;
	}
	send_wflx(&exec_addr, ST_OK);
	/* let the ack drain before the WINC goes unserviced for good */
	for(int i = 0; i < 30; i++) {
		winc_service_irq();
		m2m_wifi_handle_events(NULL);
		busy_wait(10);
	}
	log_puts("running app from SDRAM ("); log_uint(img_len);
	log_puts(" B, not flashed -- gone on next reset)"); log_nl();
	log_puts("--============== \e[1mapp\e[0m ===============--"); log_nl();
	uart_sync();
#ifdef CONFIG_CPU_HAS_INTERRUPT
	irq_setmask(0);
	irq_setie(0);
#endif
	flush_cpu_icache();
	flush_cpu_dcache();
	chain_stub(IMG_BUF, (void *)MAIN_RAM_BASE, (img_len + 3) & ~3u, MAIN_RAM_BASE);
	__builtin_unreachable();
}

/* ---- sockets -------------------------------------------------------------- */
static void ldr_start(void)
{
	struct sockaddr_in addr;
	ldr_sock = socket(AF_INET, SOCK_DGRAM, 0);
	if(ldr_sock < 0) {
		log_puts("loader socket() failed: "); log_int(ldr_sock); log_nl();
		return;
	}
	addr.sin_family      = AF_INET;
	addr.sin_port        = _htons(LOADER_PORT);
	addr.sin_addr.s_addr = 0;
	bind(ldr_sock, (struct sockaddr *)&addr, sizeof(addr));
}

static void socket_cb(SOCKET sock, uint8 u8Msg, void *pvMsg)
{
	if(mdns_socket_cb(sock, u8Msg, pvMsg))
		return;
	if(sock != ldr_sock || ldr_sock < 0)
		return;

	switch(u8Msg) {
	case SOCKET_MSG_BIND:
		recvfrom(ldr_sock, ldr_rx, sizeof(ldr_rx), 0);
		log_puts("loader listening on UDP port "); log_uint(LOADER_PORT); log_nl();
		break;
	case SOCKET_MSG_RECVFROM: {
		tstrSocketRecvMsg *m = (tstrSocketRecvMsg *)pvMsg;
		if(m->s16BufferSize >= 4) {
			const uint8 *p = m->pu8Buffer;
			if     (memcmp(p, "WFLD", 4) == 0) handle_data(p, m->s16BufferSize);
			else if(memcmp(p, "WFLS", 4) == 0) handle_stat(&m->strRemoteAddr);
			else if(memcmp(p, "WFLB", 4) == 0) handle_begin(p, m->s16BufferSize, &m->strRemoteAddr);
			else if(memcmp(p, "WFLP", 4) == 0) handle_prog(&m->strRemoteAddr);
			else if(memcmp(p, "WFLX", 4) == 0) handle_exec(&m->strRemoteAddr);
			else if(memcmp(p, "WFLR", 4) == 0) handle_reboot(&m->strRemoteAddr);
		}
		recvfrom(ldr_sock, ldr_rx, sizeof(ldr_rx), 0);   /* re-arm */
		break;
	}
	default:
		break;
	}
}

/* ---- WiFi ------------------------------------------------------------------ */
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
			if(ldr_sock >= 0) { close(ldr_sock); ldr_sock = -1; }
			wifi_connect();
		}
		break;
	}
	case M2M_WIFI_REQ_DHCP_CONF: {
		tstrM2MIPConfig *ip = (tstrM2MIPConfig *)pvMsg;
		log_puts("wifi: DHCP done, IP "); log_ip(ip->u32StaticIP); log_nl();
		ldr_start();
		mdns_start(ip->u32StaticIP);   /* board reachable as icepi.local */
		break;
	}
	default:
		break;
	}
}

int main(void)
{
	uart_init();
#ifdef CONFIG_CPU_HAS_INTERRUPT
	irq_setie(1);
#endif

	log_puts("WiFi flash-loader (ATWINC1500 -> W25Q128 via LiteSPI master)"); log_nl();

	/* Flash-master smoke test before touching WiFi: JEDEC ID over the master,
	 * first mmap bytes over the XIP read path. W25Q128JV = ef4018; mmap[0]
	 * should show the bitstream preamble if one is flashed. */
	uint32_t id = flash_jedec_id();
	log_puts("flash JEDEC ID = 0x"); log_hex32(id);
	log_puts(id == 0x00ef4018 ? "  -- OK (W25Q128JV)" : "  -- UNEXPECTED");
	log_nl();
	flush_cpu_dcache();
	log_puts("flash[0..7] via mmap = ");
	for(int i = 0; i < 8; i++) {
		log_hex8(((const uint8_t *)SPIFLASH_BASE)[i]); log_char(' ');
	}
	log_nl();

	/* ---- boot-manager triage: stay in loader mode or chain to the app -- */
	if(boot_flag_requested()) {
		log_puts("boot flag set -- staying in loader"); log_nl();
	} else if(ftdi_stay_requested()) {
		log_puts("FTDI stay level -- staying in loader"); log_nl();
	} else if(grace_window_hit(500)) {
		log_puts("'l' key -- staying in loader"); log_nl();
	} else {
		try_chain_boot();   /* returns only without a valid app image */
	}
	log_puts("loader mode ('b' = reboot to app)"); log_nl();

	nm_bsp_init();

	tstrWifiInitParam param;
	memset(&param, 0, sizeof(param));
	param.pfAppWifiCb = wifi_cb;

	log_puts("m2m_wifi_init()..."); log_nl();
	sint8 ret = m2m_wifi_init(&param);
	if(ret != M2M_SUCCESS) {
		log_puts("WINC init FAILED, ret="); log_int(ret); log_nl();
		for(;;) ;
	}
	m2m_wifi_set_sleep_mode(M2M_NO_PS, 0);   /* throughput >> power, as in winc_test */
	socketInit();
	registerSocketCallback(socket_cb, NULL);
	wifi_connect();

	for(;;) {
		winc_service_irq();
		m2m_wifi_handle_events(NULL);

		/* console escape hatch: 'b' reboots (flag is clear -> chains app) */
		if(uart_read_nonblock() && uart_read() == 'b') {
			log_puts("rebooting to app"); log_nl();
			uart_sync();
			ctrl_reset_write(1);
		}

		if(prog_pending && !prog_done)
			run_program();

		if(exec_pending)
			run_exec();             /* returns only on CRC mismatch */

		if(reboot_pending) {
			/* let the WFLR ack drain, then hand back to the BIOS */
			for(int i = 0; i < 30; i++) {
				winc_service_irq();
				m2m_wifi_handle_events(NULL);
				busy_wait(10);
			}
			log_puts("resetting SoC"); log_nl();
			ctrl_reset_write(1);
		}
	}

	return 0;
}
