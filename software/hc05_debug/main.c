#include <generated/csr.h>
#include <libbase/uart.h>

#ifndef CSR_HC05_UART_BASE
#error "This firmware requires hc05_uart to be present in the SoC."
#endif

#ifndef CSR_HC05_EN_BASE
#error "This firmware requires hc05_en to be present in the SoC."
#endif

#define SOF0 0xaa
#define SOF1 0x55

#define CMD_PING        0x01
#define CMD_SET_EN      0x10
#define CMD_GET_EN      0x11
#define CMD_UART_WRITE  0x20
#define CMD_UART_LINE   0x21
#define CMD_UART_READ   0x22
#define CMD_UART_FLUSH  0x23
#define CMD_UART_STATUS 0x24
#define CMD_LR_RESET    0x30
#define CMD_LR_STATUS   0x31
#define CMD_LR_SPI      0x32
#define CMD_LR_CS       0x33
#define CMD_MCP_READ    0x40

#define RSP_ERROR       0x7f

#define ERR_BAD_LEN      1
#define ERR_BAD_CHECKSUM 2
#define ERR_BAD_CMD      3

#define HC05_RX_BUF_SIZE 256

static unsigned char hc05_rx_buf[HC05_RX_BUF_SIZE];
static unsigned int hc05_rx_rd;
static unsigned int hc05_rx_wr;

/* --- Helpers --- */

static void console_write_byte(unsigned char value) {
    while (uart_txfull_read());
    uart_rxtx_write(value);
}

static int console_read_nonblock(void) {
    if (uart_rxempty_read()) return -1;
    int c = (int)(uart_rxtx_read() & 0xff);
    uart_ev_pending_write(UART_EV_RX);
    return c;
}

static void hc05_write_byte(unsigned char value) {
    while (hc05_uart_txfull_read());
    hc05_uart_rxtx_write(value);
}

static int hc05_read_nonblock(void) {
    if (hc05_uart_rxempty_read()) return -1;
    int c = (int)(hc05_uart_rxtx_read() & 0xff);
    hc05_uart_ev_pending_write(UART_EV_RX);
    return c;
}

static void wait_lr1121_ready(void) {
    // Bit 0 is busy per your Cat() definition
    while (lr1121_status_in_read() & 0x1);
}

static void shared_spi_select(unsigned char sel) {
    shared_spi_cs_write((unsigned int)sel | (1u << 16));
}

static void shared_spi_deselect(void) {
    shared_spi_cs_write(0);
}

static unsigned char shared_spi_xfer_byte(unsigned char value) {
    shared_spi_mosi_write(value);
    shared_spi_control_write(1u | (8u << 8));
    while ((shared_spi_status_read() & 0x1) == 0);
    return (unsigned char)(shared_spi_miso_read() & 0xff);
}

static unsigned char checksum_xor(unsigned char len, unsigned char cmd, const unsigned char *payload) {
    unsigned char chk = len ^ cmd;
    for (unsigned int i = 0; i < len - 1; i++) chk ^= payload[i];
    return chk;
}

static void send_frame(unsigned char cmd, const unsigned char *payload, unsigned char payload_len) {
    unsigned char len = (unsigned char)(payload_len + 1);
    unsigned char chk = checksum_xor(len, cmd, payload);
    console_write_byte(SOF0);
    console_write_byte(SOF1);
    console_write_byte(len);
    console_write_byte(cmd);
    for (unsigned int i = 0; i < payload_len; i++) console_write_byte(payload[i]);
    console_write_byte(chk);
}

static void send_error(unsigned char cmd, unsigned char err) {
    unsigned char payload[2] = {cmd, err};
    send_frame(RSP_ERROR, payload, 2);
}

/* --- Buffer Management --- */

static unsigned int hc05_rx_count(void) { return (hc05_rx_wr - hc05_rx_rd) & (HC05_RX_BUF_SIZE - 1); }

static void hc05_rx_push(unsigned char value) {
    unsigned int next = (hc05_rx_wr + 1) & (HC05_RX_BUF_SIZE - 1);
    if (next == hc05_rx_rd) hc05_rx_rd = (hc05_rx_rd + 1) & (HC05_RX_BUF_SIZE - 1);
    hc05_rx_buf[hc05_rx_wr] = value;
    hc05_rx_wr = next;
}

static unsigned char hc05_rx_pop(void) {
    unsigned char value = hc05_rx_buf[hc05_rx_rd];
    hc05_rx_rd = (hc05_rx_rd + 1) & (HC05_RX_BUF_SIZE - 1);
    return value;
}

static void pump_hc05_rx(void) {
    for (;;) {
        int c = hc05_read_nonblock();
        if (c < 0) break;
        hc05_rx_push((unsigned char)c);
    }
}

/* --- Commands --- */

static void handle_command(unsigned char cmd, const unsigned char *payload, unsigned char payload_len) {
    switch (cmd) {
    case CMD_PING: {
        static const unsigned char pong[] = {'P', 'O', 'N', 'G'};
        send_frame((unsigned char)(cmd | 0x80), pong, 4);
        break;
    }
    case CMD_SET_EN:
        if (payload_len != 1) { send_error(cmd, ERR_BAD_LEN); break; }
        hc05_en_out_write(payload[0] ? 1 : 0);
        send_frame((unsigned char)(cmd | 0x80), payload, 1);
        break;
    case CMD_GET_EN: {
        unsigned char rsp = (unsigned char)(hc05_en_out_read() & 0x1);
        send_frame((unsigned char)(cmd | 0x80), &rsp, 1);
        break;
    }
    case CMD_UART_WRITE:
        for (unsigned int i = 0; i < payload_len; i++) hc05_write_byte(payload[i]);
        send_frame((unsigned char)(cmd | 0x80), payload, payload_len);
        break;
    case CMD_UART_READ: {
        unsigned char requested = payload_len ? payload[0] : 64;
        unsigned char count = (requested < hc05_rx_count()) ? requested : hc05_rx_count();
        unsigned char rsp[255];
        for (unsigned int i = 0; i < count; i++) rsp[i] = hc05_rx_pop();
        send_frame((unsigned char)(cmd | 0x80), rsp, count);
        break;
    }
    case CMD_LR_RESET:
        if (payload_len != 1) { send_error(cmd, ERR_BAD_LEN); break; }
        lr1121_reset_out_write(payload[0] ? 1 : 0);
        send_frame((unsigned char)(cmd | 0x80), payload, 1);
        break;
    case CMD_LR_STATUS: {
        unsigned char rsp[2];
        rsp[0] = (unsigned char)(lr1121_reset_out_read() & 0x1);
        rsp[1] = (unsigned char)(lr1121_status_in_read() & 0xFF);
        send_frame((unsigned char)(cmd | 0x80), rsp, 2);
        break;
    }
    case CMD_LR_SPI: {
        unsigned char rsp[255];
        wait_lr1121_ready();
        shared_spi_select(0x2);
        for (unsigned int i = 0; i < payload_len; i++) rsp[i] = shared_spi_xfer_byte(payload[i]);
        shared_spi_deselect();
        wait_lr1121_ready();
        send_frame((unsigned char)(cmd | 0x80), rsp, payload_len);
        break;
    }
    case CMD_LR_CS: {
        if (payload_len != 1) { send_error(cmd, ERR_BAD_LEN); break; }
        if (payload[0]) { wait_lr1121_ready(); shared_spi_select(0x2); } 
        else { shared_spi_deselect(); wait_lr1121_ready(); }
        send_frame((unsigned char)(cmd | 0x80), payload, 1);
        break;
    }
    case CMD_MCP_READ: {
        if (payload_len != 1) { send_error(cmd, ERR_BAD_LEN); break; }
        shared_spi_select(0x1);
        shared_spi_xfer_byte(0x01);
        unsigned char rx1 = shared_spi_xfer_byte(0x80 | ((payload[0] & 0x07) << 4));
        unsigned char rx2 = shared_spi_xfer_byte(0x00);
        shared_spi_deselect();
        unsigned char rsp[2] = {(rx1 & 0x03), rx2};
        send_frame((unsigned char)(cmd | 0x80), rsp, 2);
        break;
    }
    default: send_error(cmd, ERR_BAD_CMD); break;
    }
}

int main(void) {
    enum { ST_SOF0, ST_SOF1, ST_LEN, ST_CMD, ST_PAYLOAD, ST_CHECKSUM } state = ST_SOF0;
    unsigned char len = 0, cmd = 0, payload[255], payload_pos = 0;

    hc05_en_out_write(1);
    lr1121_reset_out_write(1); // Set Reset High (Inactive)
    hc05_rx_rd = 0; hc05_rx_wr = 0;

    for (;;) {
        pump_hc05_rx();
        int c = console_read_nonblock();
        if (c < 0) continue;

        switch (state) {
            case ST_SOF0: if ((unsigned char)c == SOF0) state = ST_SOF1; break;
            case ST_SOF1: state = ((unsigned char)c == SOF1) ? ST_LEN : ST_SOF0; break;
            case ST_LEN: 
                len = (unsigned char)c; 
                if (len == 0) { send_error(0, ERR_BAD_LEN); state = ST_SOF0; }
                else state = ST_CMD; 
                break;
            case ST_CMD: 
                cmd = (unsigned char)c; payload_pos = 0;
                state = (len == 1) ? ST_CHECKSUM : ST_PAYLOAD; 
                break;
            case ST_PAYLOAD: 
                payload[payload_pos++] = (unsigned char)c;
                if (payload_pos == (unsigned char)(len - 1)) state = ST_CHECKSUM; 
                break;
            case ST_CHECKSUM:
                if ((unsigned char)c != checksum_xor(len, cmd, payload)) send_error(cmd, ERR_BAD_CHECKSUM);
                else handle_command(cmd, payload, (unsigned char)(len - 1));
                state = ST_SOF0;
                break;
        }
    }
}