# Display + Touch Architecture

This is a high-level walkthrough of how the LCD and touch subsystem
fits together, from LVGL down to the panel and touch IC. It's
intentionally light on specifics — no pinouts, no CSR bit layouts —
just how the pieces compose.

## Layers

```
   +-----------------------------------------------------------+
   |  LVGL  (widget tree, layout engine, software renderer)    |
   |                                                           |
   |    flush_cb()        indev read_cb()       tick cb        |
   +--------|------------------|---------------------|---------+
            |                  |                     |
            v                  v                     v
       LCD engine          HW I2C master          timer0
       (gateware)          (gateware)             (free-running)
            |                  |
            v                  v
       SPI + CS/DC          I2C SCL/SDA
            |                  |
            v                  v
       ST7796S panel        FT6336U touch IC
```

LVGL sits on top of three driver glue points. Below those, all three
peripherals are LiteX gateware modules. Below the gateware are real
pins to the panel and the capacitive touch controller.

## Render path

LVGL runs in **partial render mode with double buffering**. Two
buffers in SDRAM (one is being rendered into while the other is being
shifted out). Per dirty rectangle:

1. **Render.** The software renderer composites the rect into the
   current LVGL buffer as RGB565, pre-swapped so the wire byte order
   matches what the ST7796S expects.

2. **Stage.** `flush_cb()` writes the rect coordinates and the
   buffer's source pointer and length into the LCD engine, stashes the
   `lv_display_t *` for the completion ISR, and fires a `DMA_RECT` op.
   It does **not** wait for the transfer to complete — it returns
   immediately.

3. **Sequencer.** The LCD engine FSM owns CS and DC. A `DMA_RECT` op
   internally emits three sub-frames — set-column, set-row, write-RAM
   — each framed by its own CS pulse with hardware-driven DC. The CPU
   is uninvolved for the duration.

4. **Push.** The FSM produces one byte per system cycle into a small
   async FIFO; an SPI shifter in its own clock domain drains the FIFO
   at line rate (~92 MHz SCK). There is no per-byte CPU handshake.

5. **Fetch.** The engine reads pixel data from main memory via its
   own wishbone bus master, prefetching 32-bit words behind the FIFO
   so word-boundary fetches don't starve the shifter.

6. **Complete.** After the last byte clocks out, the engine raises an
   op-done event. A short ISR clears the pending bit and calls
   `lv_display_flush_ready()` for the buffer that was in flight. LVGL
   is then free to reuse that buffer.

Because step 2 returns immediately, LVGL begins rendering the next
dirty rect into the other buffer while the previous buffer is still
being shifted out. CPU rendering and SPI shifting overlap; for any
non-trivial frame the slower of the two dominates rather than the sum.

## Op queue

The engine has a single-slot queue ahead of the active op. Software
writes a new op's CSRs and fires `op.start`:

- If the engine is idle, the op runs immediately.
- If the engine is busy, the op latches into the queue and runs the
  moment the active op completes — no IDLE round-trip.

A status bit (`can_accept`) tells software when the queue slot is
free. LVGL's flush model only ever has one outstanding flush, so we
don't fully exploit queue depth from LVGL; but the mechanism is in
place for any future workload (multi-rect blits, descriptor lists)
that wants to keep the engine fed without per-op software involvement.

## Touch path

LVGL's input-device timer ticks roughly every 33 ms. On each tick:

1. `indev read_cb()` runs an I2C transaction against the FT6336U:
   write the register pointer, repeated-start, read the touch-count
   byte and the point-1 coordinates.
2. The I2C master is a hardware FSM (wishbone-mapped). The CPU writes
   one CSR per logical I2C step — start, write byte, read byte, stop
   — and the gateware clocks the actual bits and ACKs.
3. The callback returns either a press with coordinates or a release.
   LVGL routes the event to whichever widget owns that point.

The touch IC's interrupt line is also wired to an IRQ-capable GPIO,
though the firmware doesn't use it today — polling is cheap now that
the I2C transaction is hardware-driven. The IRQ is available if a
future workload wants to skip polls when nothing has changed.

## Tick source

LVGL needs a monotonic millisecond clock. The firmware gives it one
via `lv_tick_set_cb()`, derived from `timer0` running free as a
32-bit countdown. Each callback samples the timer, computes elapsed
cycles against a saved previous value, and accumulates into a ms
counter.

One practical consequence: `timer0` is exclusively owned by the LVGL
tick once initialized. The firmware must not call `busy_wait()` (which
reloads `timer0`) after that point — doing so silently corrupts the
time source and causes LVGL's timers to stop firing.

## Why the layering looks like this

- **CS / DC framing in hardware** keeps per-byte CSR traffic out of
  the hot path. A multi-byte command goes from ~5+3·N CSR writes to 3.
- **RECT ops in hardware** bake the panel's set-window + write-RAM
  pattern into the engine. A rect flush is a fixed handful of CSR
  writes regardless of size.
- **Pipelined push** lets the SPI shifter run at panel bandwidth, not
  at the FSM round-trip rate. The depth-4 FIFO and in-flight counter
  let the FSM stay ahead of the shifter without blocking.
- **Op queue** decouples the CPU's "set up next op" work from the
  engine's "finish current op" work.
- **Op-done IRQ** lets the CPU work on rendering while SPI is busy,
  rather than spinning on a `wait_idle` poll.
- **Hardware I2C** frees the CPU during touch polls, which matters
  more once LVGL's tick handler is doing real per-frame work.

The cumulative effect is that the CPU per LVGL frame spends its time
rendering pixels; the SPI bus stays loaded for as long as there are
pixels to push; and the two run in parallel.

## Known limitation: tearing

There is no synchronization between LVGL's flushes and the panel's
internal scanout. The ST7796S exposes a tearing-effect (TE) output for
this purpose, but the panel module we use does not break that pin out.
So flushes that happen mid-scanout will show a brief seam between the
old and new frame on the line that's currently being drawn. The
rendering pipeline itself is healthy — the artifact is at the physical
panel boundary, not in the gateware or driver.
