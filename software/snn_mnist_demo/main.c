// Standalone firmware for the SNN-MLP MNIST classifier.
//
// Boots into a binary UART command protocol the host driver
// tools/stream_snn_mnist_uart.py speaks. All multi-byte values are little
// endian. Each command is replied to with a lowercase ack of its letter.
//
// Commands:
//   'P'                                           ping  -> 'p'
//   'C' + u32 weight_base + u32 beats + u32 cyc   configure loader CSRs -> 'c'
//   'B' + 74*int16 biases (HIDDEN then OUT_SIZE)  load biases via CSRs  -> 'b'
//   'W' + u32 length + length bytes               copy blob into SDRAM  -> 'w'
//   'I' + 784*int16 pixels                        run inference         -> 'i'
//                                                  + 1 byte class
//                                                  + 10 bytes spike counts
//
// Weight blob lives at MAIN_RAM_BASE + WEIGHT_BLOB_OFFSET. The host sets
// weight_base = MAIN_RAM_BASE + WEIGHT_BLOB_OFFSET via the 'C' command.
#include <stdint.h>

#include <generated/csr.h>
#include <generated/mem.h>
#include <generated/soc.h>
#include <irq.h>
#include <system.h>            // flush_cpu_dcache, flush_l2_cache
#include <libbase/uart.h>

#define IMG_SIZE            784
#define BIAS_COUNT          74      // HIDDEN(64) + OUT_SIZE(10)
#define OUT_SPIKE_COUNTERS  10
#define WEIGHT_BLOB_OFFSET  0x100000u    // 1 MiB into SDRAM; firmware lives below
#define WEIGHT_BLOB_ADDR    (MAIN_RAM_BASE + WEIGHT_BLOB_OFFSET)
#define WEIGHT_CHUNK_SIZE   256u         // host must ack-wait every chunk so the
                                         // UART RX FIFO never overflows on big
                                         // (~200 KB) blob uploads.

static void uart_write_str(const char *s)
{
    while (*s) uart_write(*s++);
}

static uint32_t uart_read_u32_le(void)
{
    uint32_t v = 0;
    for (int i = 0; i < 4; i++)
        v |= ((uint32_t)(uint8_t)uart_read()) << (i * 8);
    return v;
}

static int16_t uart_read_i16_le(void)
{
    uint8_t lo = (uint8_t)uart_read();
    uint8_t hi = (uint8_t)uart_read();
    return (int16_t)((uint16_t)hi << 8 | (uint16_t)lo);
}

static inline void snn_start_pulse(void)
{
    snn_control_write(1u);   // bit 0 = start (pulse)
}
static inline void snn_clear_pulse(void)
{
    snn_control_write(2u);   // bit 1 = clear_state (pulse)
}
static inline void snn_pixel_commit(uint16_t addr, int16_t data)
{
    snn_pixel_addr_write(addr);
    snn_pixel_data_write((uint16_t)data);
    snn_pixel_ctl_write(1u); // .we pulse
}
static inline void snn_bias_commit(uint16_t addr, int16_t data)
{
    snn_bias_addr_write(addr);
    snn_bias_data_write((uint16_t)data);
    snn_bias_ctl_write(1u);  // .we pulse
}
static inline int snn_done(void)
{
    return (snn_status_read() >> 1) & 1u;  // status bit 1 = done
}
static inline uint8_t snn_classification(void)
{
    return (uint8_t)((snn_status_read() >> 5) & 0xFu); // 4-bit class at bit 5
}

static uint8_t spike_count_read(int i)
{
    switch (i) {
    case 0:  return (uint8_t)snn_spike_count_0_read();
    case 1:  return (uint8_t)snn_spike_count_1_read();
    case 2:  return (uint8_t)snn_spike_count_2_read();
    case 3:  return (uint8_t)snn_spike_count_3_read();
    case 4:  return (uint8_t)snn_spike_count_4_read();
    case 5:  return (uint8_t)snn_spike_count_5_read();
    case 6:  return (uint8_t)snn_spike_count_6_read();
    case 7:  return (uint8_t)snn_spike_count_7_read();
    case 8:  return (uint8_t)snn_spike_count_8_read();
    case 9:  return (uint8_t)snn_spike_count_9_read();
    default: return 0;
    }
}

static void cmd_configure(void)
{
    uint32_t base = uart_read_u32_le();
    uint32_t bpc  = uart_read_u32_le();
    uint32_t nc   = uart_read_u32_le();
    snn_weight_base_write(base);
    snn_weight_beats_per_cycle_write(bpc);
    snn_weight_num_cycles_write(nc);
    uart_write('c');
}

static void cmd_load_biases(void)
{
    for (int i = 0; i < BIAS_COUNT; i++) {
        int16_t v = uart_read_i16_le();
        snn_bias_commit((uint16_t)i, v);
    }
    uart_write('b');
}

static void cmd_load_weights(void)
{
    uint32_t len = uart_read_u32_le();
    volatile uint8_t *dst = (volatile uint8_t *)WEIGHT_BLOB_ADDR;
    uint32_t i = 0;
    // Chunked receive: after every WEIGHT_CHUNK_SIZE bytes, send '.' so the
    // host knows it can release the next chunk. This bounds in-flight bytes
    // to chunk_size + ack_RTT, which keeps the UART RX FIFO from overflowing
    // when the cache evicts a line and the CPU stalls briefly.
    while (i < len) {
        uint32_t remaining = len - i;
        uint32_t chunk = (remaining < WEIGHT_CHUNK_SIZE) ? remaining : WEIGHT_CHUNK_SIZE;
        for (uint32_t j = 0; j < chunk; j++)
            dst[i + j] = (uint8_t)uart_read();
        i += chunk;
        uart_write('.');
    }
    // Make sure CPU-side writes are visible to the SNN's Wishbone master.
    flush_cpu_dcache();
#ifdef CONFIG_L2_SIZE
    flush_l2_cache();
#endif
    uart_write('w');
}

static void cmd_infer(void)
{
    snn_clear_pulse();

    // 784 little-endian int16 pixels, written into pixel_mem one at a time.
    for (int i = 0; i < IMG_SIZE; i++) {
        int16_t px = uart_read_i16_le();
        snn_pixel_commit((uint16_t)i, px);
    }

    snn_start_pulse();
    while (!snn_done()) { /* busy-wait */ }

    uart_write('i');
    uart_write(snn_classification());
    for (int i = 0; i < OUT_SPIKE_COUNTERS; i++)
        uart_write(spike_count_read(i));
}

int main(void)
{
    irq_setmask(0);
    irq_setie(1);
    uart_init();

    uart_write_str("# snn_mnist_demo ready\n");

    for (;;) {
        uint8_t cmd = (uint8_t)uart_read();
        switch (cmd) {
        case 'P': uart_write('p');           break;
        case 'C': cmd_configure();           break;
        case 'B': cmd_load_biases();         break;
        case 'W': cmd_load_weights();        break;
        case 'I': cmd_infer();               break;
        default:  uart_write('?');           break;
        }
    }
    return 0;
}
