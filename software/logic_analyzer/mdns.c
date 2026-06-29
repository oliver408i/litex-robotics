/* Minimal mDNS (RFC 6762) name responder for the ATWINC1500 -- see mdns.h.
 *
 * One UDP socket bound to 5353 and joined to 224.0.0.251 via the WINC's
 * IP_ADD_MEMBERSHIP sockopt. Incoming queries are matched against
 * MDNS_HOSTNAME".local" (single-question A/ANY queries, which is what
 * avahi/macOS/Windows resolvers send for plain name lookups); matches get a
 * multicast response with our A record + an NSEC record saying "no AAAA". */
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "mdns.h"

#define MDNS_PORT  5353
#define MDNS_TTL   120      /* seconds; short so a re-flashed board updates fast */

/* 224.0.0.251 in network byte order (first octet = LSB, as the WINC wants). */
#define MDNS_MCAST_IP  ((uint32)0xFB0000E0)

#define NAME_LEN   (sizeof(MDNS_HOSTNAME) - 1)
#define DOMAIN_LEN 5                      /* "local" */
#define HEADER_SIZE 12

static SOCKET mdns_sock = -1;
static uint32 our_ip;                     /* network byte order */
static uint8  rx_buf[512];

static const uint8 domain[DOMAIN_LEN] = { 'l', 'o', 'c', 'a', 'l' };

/* Response template, filled with name/TTL/IP at mdns_start():
 *   header(12) | QNAME(1+NAME_LEN+1+DOMAIN_LEN+1) | A(14) | NSEC(20) */
#define A_OFF      (HEADER_SIZE + 1 + NAME_LEN + 1 + DOMAIN_LEN + 1)
#define A_TTL_OFF  (A_OFF + 4)
#define A_IP_OFF   (A_OFF + 10)
#define NSEC_OFF   (A_OFF + 14)
#define RESP_LEN   (NSEC_OFF + 20)

static uint8 response[RESP_LEN] = {
	/* header */
	0x00, 0x00,               /* ID = 0                                    */
	0x84, 0x00,               /* flags: response + authoritative           */
	0x00, 0x00,               /* questions   = 0                           */
	0x00, 0x01,               /* answers     = 1                           */
	0x00, 0x00,               /* NS records  = 0                           */
	0x00, 0x01,               /* additionals = 1 (the NSEC)                */
	/* QNAME + A record + NSEC filled in by mdns_start() */
};

static const uint8 a_record[14] = {
	0x00, 0x01,               /* type A                                    */
	0x80, 0x01,               /* class IN + cache-flush                    */
	0x00, 0x00, 0x00, 0x00,   /* TTL (patched)                             */
	0x00, 0x04,               /* rdlength                                  */
	0x00, 0x00, 0x00, 0x00,   /* IP (patched)                              */
};

static const uint8 nsec_record[20] = {
	0xC0, 0x0C,               /* name = pointer to QNAME at offset 12      */
	0x00, 0x2F,               /* type NSEC                                 */
	0x80, 0x01,               /* class IN + cache-flush                    */
	0x00, 0x00, 0x00, 0x00,   /* TTL (patched)                             */
	0x00, 0x08,               /* rdlength                                  */
	0xC0, 0x0C,               /* next domain = QNAME                       */
	0x00, 0x04,               /* bitmap block 0, length 4                  */
	0x40, 0x00, 0x00, 0x00,   /* only bit 1 (A) set -> "A exists, no AAAA" */
};

static void build_response(void)
{
	uint8 *r = response + HEADER_SIZE;
	uint32 ttl = _htonl(MDNS_TTL);

	*r++ = NAME_LEN;
	memcpy(r, MDNS_HOSTNAME, NAME_LEN);          r += NAME_LEN;
	*r++ = DOMAIN_LEN;
	memcpy(r, domain, DOMAIN_LEN);               r += DOMAIN_LEN;
	*r++ = 0x00;                                 /* root terminator */

	memcpy(response + A_OFF, a_record, sizeof(a_record));
	memcpy(response + A_TTL_OFF, &ttl, 4);
	memcpy(response + A_IP_OFF,  &our_ip, 4);

	memcpy(response + NSEC_OFF, nsec_record, sizeof(nsec_record));
	memcpy(response + NSEC_OFF + 6, &ttl, 4);    /* NSEC TTL too */
}

/* Case-insensitive ASCII compare (mDNS names are case-insensitive). */
static uint8 name_matches(const uint8 *p)
{
	for (unsigned int i = 0; i < NAME_LEN; i++) {
		uint8 a = p[i], b = (uint8)MDNS_HOSTNAME[i];
		if (a >= 'A' && a <= 'Z') a += 'a' - 'A';
		if (b >= 'A' && b <= 'Z') b += 'a' - 'A';
		if (a != b) return 0;
	}
	return 1;
}

/* True iff the packet is a query whose first question is
 * MDNS_HOSTNAME.local, type A (or ANY), class IN. */
static uint8 is_our_query(const uint8 *q, sint16 len)
{
	const sint16 min_len = HEADER_SIZE + 1 + NAME_LEN + 1 + DOMAIN_LEN + 1 + 4;
	uint16 qtype, qclass;

	if (len < min_len)                                    return 0;
	if (q[2] != 0x00 || q[3] != 0x00)                     return 0;  /* must be a query */
	if (q[HEADER_SIZE] != NAME_LEN)                       return 0;
	if (!name_matches(q + HEADER_SIZE + 1))               return 0;
	if (q[HEADER_SIZE + 1 + NAME_LEN] != DOMAIN_LEN)      return 0;
	if (memcmp(q + HEADER_SIZE + 2 + NAME_LEN, domain, DOMAIN_LEN) != 0) return 0;
	if (q[HEADER_SIZE + 2 + NAME_LEN + DOMAIN_LEN] != 0)  return 0;  /* terminator */

	qtype  = (uint16)((q[min_len - 4] << 8) | q[min_len - 3]);
	qclass = (uint16)((q[min_len - 2] << 8) | q[min_len - 1]);
	if (qtype != 0x0001 && qtype != 0x00FF)               return 0;  /* A or ANY */
	if ((qclass & 0x7FFF) != 0x0001)                      return 0;  /* IN (ignore QU bit) */
	return 1;
}

static void mdns_arm_recv(void)
{
	recvfrom(mdns_sock, rx_buf, sizeof(rx_buf), 0);
}

void mdns_start(uint32 ip_be)
{
	struct sockaddr_in addr;

	our_ip = ip_be;
	build_response();

	if (mdns_sock >= 0) {        /* DHCP renew / reconnect: refresh only */
		return;
	}
	mdns_sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (mdns_sock < 0) {
		printf("mdns: socket() failed %d\n", mdns_sock);
		return;
	}
	addr.sin_family      = AF_INET;
	addr.sin_port        = _htons(MDNS_PORT);
	addr.sin_addr.s_addr = 0;
	bind(mdns_sock, (struct sockaddr *)&addr, sizeof(addr));
	/* rest of the setup continues in mdns_socket_cb on SOCKET_MSG_BIND */
}

uint8 mdns_socket_cb(SOCKET sock, uint8 u8Msg, void *pvMsg)
{
	if (sock != mdns_sock || mdns_sock < 0)
		return 0;

	switch (u8Msg) {
	case SOCKET_MSG_BIND: {
		tstrSocketBindMsg *m = (tstrSocketBindMsg *)pvMsg;
		if (m->status == 0) {
			uint32 mcast = MDNS_MCAST_IP;
			setsockopt(mdns_sock, SOL_SOCKET, IP_ADD_MEMBERSHIP,
			           &mcast, sizeof(mcast));
			mdns_arm_recv();
			printf("mdns: responding to %s.local\n", MDNS_HOSTNAME);
		} else {
			printf("mdns: bind failed %d\n", m->status);
		}
		break;
	}
	case SOCKET_MSG_RECVFROM: {
		tstrSocketRecvMsg *m = (tstrSocketRecvMsg *)pvMsg;
		if (m->s16BufferSize > 0 && is_our_query(m->pu8Buffer, m->s16BufferSize)) {
			struct sockaddr_in dst;
			dst.sin_family      = AF_INET;
			dst.sin_port        = _htons(MDNS_PORT);
			dst.sin_addr.s_addr = MDNS_MCAST_IP;
			sendto(mdns_sock, response, RESP_LEN, 0,
			       (struct sockaddr *)&dst, sizeof(dst));
		}
		mdns_arm_recv();
		break;
	}
	default:
		break;   /* sendto acks etc. -- consumed, nothing to do */
	}
	return 1;
}
