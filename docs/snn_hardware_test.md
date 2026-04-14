# SNN Hardware Test

This note describes the current hardware test flow for the tiny SNN tracking
estimator peripheral.

## What The Gateware Exposes

With `--with-snn-poc`, the SoC adds an `snn` CSR block with:

- `snn_control`
  Bit 0: `start`
  Bit 1: `clear_state`
- `snn_measurement`
  signed `Q4.12` input measurement
- `snn_status`
  busy/done plus a small phase/index debug view
- `snn_position`
  signed `Q4.12` position estimate
- `snn_velocity`
  signed `Q4.12` velocity estimate
- `snn_cycles`
  cycles spent on the last sample

The currently validated hardware datapath is:

- firmware-driven 4-bit spike encoder
- 8-neuron feedforward LIF bank in Verilog
- 2-output fixed-point readout in Verilog
- LiteX CSR wrapper in Python

When the board resources are available:

- LED0 shows `busy`
- LED1 shows `done`

## Build And Load

From the repo root:

```bash
python3 icepi_zero.py --with-snn-poc --build --load
```

If you want SDRAM as well:

```bash
python3 icepi_zero.py --with-snn-poc --with-sdram --build --load
```

The build should generate a `csr.csv` file in the build directory. That file is
the source of truth for the CSR addresses.

## Standalone Firmware

A small standalone firmware demo is available in
[software/snn_demo/main.c](/run/media/mp2/af99f329-f82e-4702-883f-eb45eeaf5a26/vscode-linux/fpga-mcu/software/snn_demo/main.c).

It:

- initializes UART
- clears the SNN state
- feeds a built-in measurement sequence
- prints `input`, `position`, `velocity`, and `cycles`
- repeats forever

Build it after the gateware build has generated the LiteX software headers:

```bash
make -C software/snn_demo
```

That should produce:

- `software/snn_demo/snn_demo.elf`
- `software/snn_demo/snn_demo.bin`

For the simplest first hardware test, rebuild the SoC with the firmware
embedded in integrated ROM:

```bash
python3 icepi_zero.py --with-snn-poc --integrated-rom-init software/snn_demo/snn_demo.bin --build --load
```

That replaces the BIOS image for the loaded bitstream and is usually the
cleanest way to do first bring-up.

You can also flash the firmware to SPI flash with the existing helper in
`icepi_zero.py`:

```bash
python3 icepi_zero.py --with-snn-poc --with-spi-flash --flash-firmware software/snn_demo/snn_demo.bin
```

If you want the system to autoboot that firmware from SPI flash, rebuild the
SoC with a boot offset and flash the firmware at the same offset:

```bash
python3 icepi_zero.py --with-snn-poc --with-spi-flash --flash-boot-offset 0x200000 --build --load
python3 icepi_zero.py --with-snn-poc --with-spi-flash --flash-firmware software/snn_demo/snn_demo.bin --firmware-offset 0x200000
```

## Serial Console

Open the LiteX BIOS UART at:

- baud: `1000000`
- format: `8N1`

Typical tools:

- `picocom`
- `minicom`
- `screen`

With the standalone firmware, the same UART becomes the firmware console rather
than the BIOS console.

## Quick Check

The current demo should print lines like:

```text
sample 0 in=0.483 pos=0.321 vel=0.293 cyc=13
sample 1 in=0.444 pos=0.513 vel=0.227 cyc=13
sample 2 in=0.417 pos=0.481 vel=0.185 cyc=13
```

The exact last digit may vary by a few thousandths depending on build and
rounding, but:

- `cyc` should be `13`
- `pos` and `vel` should stay close to the reference values above

You can also check a captured UART log against the expected demo sequence:

```bash
python3 tools/check_snn_demo_log.py /path/to/uart.log
```

## First Manual Test

If you use the standalone firmware, you do not need this manual CSR poking flow
for the first bring-up. It is still useful as a fallback if the firmware does
not boot as expected.

1. Reset the estimator state.
2. Write one fixed-point measurement.
3. Pulse the `start` bit.
4. Read back `snn_status`, `snn_position`, `snn_velocity`, and `snn_cycles`.

Use [tools/snn_fixed.py](/run/media/mp2/af99f329-f82e-4702-883f-eb45eeaf5a26/vscode-linux/fpga-mcu/tools/snn_fixed.py)
to convert values:

```bash
python3 tools/snn_fixed.py 0.5
python3 tools/snn_fixed.py --decode 0x0800
```

`0.5` should encode to about `0x0800` in `Q4.12`.

## BIOS Bring-Up Pattern

Use the addresses from `csr.csv` and the BIOS `mem_write` / `mem_read`
commands.

Example sequence:

1. Write `2` to `snn_control` to pulse `clear_state`.
2. Write the measurement value to `snn_measurement`.
3. Write `1` to `snn_control` to pulse `start`.
4. Poll `snn_status` until `done=1`.
5. Read `snn_position`, `snn_velocity`, and `snn_cycles`.

The current feedforward hardware implementation should report about `13` cycles
per sample.

## Expected Early Result

For the first bring-up, the most important checks are:

- `busy` rises after `start`
- `done` eventually rises
- `snn_cycles` is near `13`
- outputs change when you vary the measurement stream
- repeated samples preserve state unless `clear_state` is pulsed

## Notes

- The gateware in this repo has not been hardware-built in this session because
  the local environment here does not currently have LiteX/Migen installed.
- The currently validated hardware block is feedforward only. The Python sim
  still contains the larger recurrent reservoir model used during exploration.
- The peripheral coefficients are fixed in the current Verilog block and were
  derived from the same Python training flow used by the simulation.
