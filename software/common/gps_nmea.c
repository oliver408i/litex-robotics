/* NMEA GPS reader -- see gps_nmea.h. Integer-only (no soft-float on this
 * VexRiscv): coordinates are parsed straight to microdegrees. */
#include "gps_nmea.h"

#include <string.h>
#include <generated/csr.h>

/* Bring-up: log failed-checksum sentences so we can tell corruption-in-transit
 * (garbled / truncated text -> RX FIFO overflow) from a logic issue (clean text
 * but mismatched checksum). Off by default; build with -DGPS_DEBUG. */
#ifdef GPS_DEBUG
#include "log.h"
#endif

/* LiteX UART event bits (liblitebase/uart.c). We poll rxempty and ack RX so
 * the status re-arms; the gps UART's IRQ stays disabled (ev_enable = 0). */
#define GPS_UART_EV_RX  0x2u

/* NMEA sentences are <= 82 chars incl. CRLF; give the assembly buffer slack. */
#define LINE_MAX   96
#define MAX_FIELDS 24

static gps_status_t st;

static char     line[LINE_MAX];
static unsigned line_len;
static bool     in_sentence;     /* seen '$', collecting body */
static bool     line_ovf;        /* current line overran -> drop at terminator */

/* ---- UART ---------------------------------------------------------------- */

#ifdef CSR_GPS_BASE
static inline int gps_uart_getc(void)
{
	if(gps_rxempty_read())
		return -1;
	int c = (int)(gps_rxtx_read() & 0xffu);
	gps_ev_pending_write(GPS_UART_EV_RX);   /* ack + re-arm rxempty status */
	return c;
}
#else
/* SoC built without the gps UART: the reader is inert (link never alive). */
static inline int gps_uart_getc(void) { return -1; }
#endif

/* ---- small integer parse helpers ----------------------------------------- */

static uint32_t parse_uint(const char *s)
{
	uint32_t v = 0;
	while(*s >= '0' && *s <= '9')
		v = v * 10u + (uint32_t)(*s++ - '0');
	return v;
}

/* Read the fractional digits after a '.', scaled to exactly `places` digits
 * (zero-padded / truncated). e.g. ".038" with places=6 -> 38000. */
static uint32_t parse_frac(const char *dot, unsigned places)
{
	uint32_t v = 0;
	if(*dot == '.') dot++;
	for(unsigned i = 0; i < places; i++) {
		v *= 10u;
		if(*dot >= '0' && *dot <= '9')
			v += (uint32_t)(*dot++ - '0');
	}
	return v;
}

static int hexval(char c)
{
	if(c >= '0' && c <= '9') return c - '0';
	if(c >= 'A' && c <= 'F') return c - 'A' + 10;
	if(c >= 'a' && c <= 'f') return c - 'a' + 10;
	return -1;
}

/* NMEA coordinate "ddmm.mmmm" + hemisphere -> signed microdegrees.
 * The minutes are always the two integer digits left of the decimal point;
 * everything before them is whole degrees. */
static int32_t parse_coord(const char *s, char hemi)
{
	const char *dot = s;
	while(*dot && *dot != '.') dot++;
	unsigned intlen = (unsigned)(dot - s);
	if(intlen < 2) return 0;                 /* malformed / empty field */

	/* Split integer part into degrees + the trailing 2 minute digits. */
	char degbuf[8];
	unsigned dlen = intlen - 2;
	if(dlen >= sizeof(degbuf)) return 0;
	memcpy(degbuf, s, dlen);
	degbuf[dlen] = '\0';

	uint32_t deg     = parse_uint(degbuf);
	uint32_t min_int = (uint32_t)((s[intlen - 2] - '0') * 10 + (s[intlen - 1] - '0'));
	uint32_t min_u   = min_int * 1000000u + parse_frac(dot, 6);   /* micro-minutes */

	/* microdeg = deg*1e6 + (micro-minutes / 60). */
	int32_t udeg = (int32_t)(deg * 1000000u + min_u / 60u);
	if(hemi == 'S' || hemi == 'W')
		udeg = -udeg;
	return udeg;
}

/* ---- sentence parsing ----------------------------------------------------- */

/* GGA: ...,time,lat,N/S,lon,E/W,fixQ,sats,HDOP,alt,M,... */
static void parse_gga(char *const *f, int n)
{
	if(n < 8) return;
	st.gga++;
	if(f[1][0]) st.time_hms    = parse_uint(f[1]);
	if(f[6][0]) st.fix_quality = (uint8_t)parse_uint(f[6]);
	if(f[7][0]) st.sats        = (uint8_t)parse_uint(f[7]);
	if(f[2][0] && f[4][0]) {
		st.lat_udeg = parse_coord(f[2], f[3][0]);
		st.lon_udeg = parse_coord(f[4], f[5][0]);
	}
	if(n > 9 && f[9][0]) {
		const char *a = f[9];
		bool neg = (*a == '-');
		if(neg) a++;
		const char *dot = a;
		while(*dot && *dot != '.') dot++;
		int32_t dm = (int32_t)(parse_uint(a) * 10u + parse_frac(dot, 1));   /* decimeters */
		st.alt_dm = neg ? -dm : dm;
	}
}

/* RMC: ...,time,status,lat,N/S,lon,E/W,speed,course,date,... */
static void parse_rmc(char *const *f, int n)
{
	if(n < 10) return;
	st.rmc++;
	if(f[1][0]) st.time_hms = parse_uint(f[1]);
	st.valid = (f[2][0] == 'A');
	if(f[3][0] && f[5][0]) {
		st.lat_udeg = parse_coord(f[3], f[4][0]);
		st.lon_udeg = parse_coord(f[5], f[6][0]);
	}
	if(f[7][0]) {
		const char *sp = f[7];
		const char *dot = sp;
		while(*dot && *dot != '.') dot++;
		st.speed_mknots = (uint16_t)(parse_uint(sp) * 1000u + parse_frac(dot, 3));
	}
	if(f[9][0]) st.date_dmy = parse_uint(f[9]);
}

/* Validate checksum, tokenize, dispatch. `s` is the body between '$' and the
 * terminator (CR/LF), NUL-terminated, including the "*NN" checksum tail. */
static void handle_sentence(char *s, uint32_t now_ms)
{
	/* Locate the checksum delimiter and verify XOR of the body. */
	char *star = strchr(s, '*');
	if(star) {
		if(hexval(star[1]) < 0 || hexval(star[2]) < 0) {
			st.csum_errors++;
#ifdef GPS_DEBUG
			log_puts("gps badcsum(nohex): ["); log_puts(s); log_puts("]"); log_nl();
#endif
			return;
		}
		uint8_t want = (uint8_t)((hexval(star[1]) << 4) | hexval(star[2]));
		uint8_t got  = 0;
		for(char *p = s; p < star; p++) got ^= (uint8_t)*p;
		if(got != want) {
			st.csum_errors++;
#ifdef GPS_DEBUG
			log_puts("gps badcsum: got=0x"); log_hex8(got);
			log_puts(" want=0x");            log_hex8(want);
			log_puts(" [");                  log_puts(s); log_puts("]"); log_nl();
#endif
			return;
		}
		*star = '\0';                 /* trim checksum before tokenizing */
	}

	st.sentences++;
	st.last_ms = now_ms;
	strncpy(st.last, s, sizeof(st.last) - 1);
	st.last[sizeof(st.last) - 1] = '\0';

	/* Tokenize on ',' in place. */
	char *fields[MAX_FIELDS];
	int n = 0;
	fields[n++] = s;
	for(char *p = s; *p && n < MAX_FIELDS; p++) {
		if(*p == ',') { *p = '\0'; fields[n++] = p + 1; }
	}

	/* fields[0] = talker+type, e.g. "GPGGA"/"GNRMC". Match on the 3-char type
	 * suffix so any talker id works. */
	const char *type = fields[0];
	if(strlen(type) < 5) return;
	const char *suf = type + 2;
	if(!memcmp(suf, "GGA", 3))      parse_gga(fields, n);
	else if(!memcmp(suf, "RMC", 3)) parse_rmc(fields, n);
}

/* ---- public API ----------------------------------------------------------- */

void gps_nmea_init(uint32_t now_ms)
{
	memset(&st, 0, sizeof(st));
	st.last_ms  = now_ms;
	line_len    = 0;
	in_sentence = false;
	line_ovf    = false;

	/* Flush any stale bytes the module emitted before we started reading. */
	while(gps_uart_getc() >= 0)
		;
}

void gps_nmea_poll(uint32_t now_ms)
{
	int c;
	while((c = gps_uart_getc()) >= 0) {
		st.bytes++;

		if(c == '$') {                 /* start of a new sentence */
			line_len    = 0;
			in_sentence = true;
			line_ovf    = false;
			continue;
		}
		if(!in_sentence)
			continue;

		if(c == '\r' || c == '\n') {   /* terminator: close the sentence */
			if(line_ovf)        st.overflows++;
			else if(line_len) { line[line_len] = '\0'; handle_sentence(line, now_ms); }
			line_len    = 0;
			in_sentence = false;
			line_ovf    = false;
			continue;
		}

		if(line_len < LINE_MAX - 1)
			line[line_len++] = (char)c;
		else
			line_ovf = true;           /* too long: drop at the terminator */
	}
}

const gps_status_t *gps_nmea_get_status(void)
{
	return &st;
}

bool gps_nmea_link_alive(uint32_t now_ms, uint32_t timeout_ms)
{
	return st.sentences > 0 && (now_ms - st.last_ms) <= timeout_ms;
}
