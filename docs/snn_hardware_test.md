# SNN Hardware Notes

This note summarizes what the current hardware PoC exposes and what behavior
has already been validated on board.

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
- 8-neuron LIF bank in Verilog
- previous-timestep spike recurrence
- 2-output fixed-point readout in Verilog
- LiteX CSR wrapper in Python

When the board resources are available:

- LED0 shows `busy`
- LED1 shows `done`

## Standalone Firmware

A small standalone firmware demo is available in
[software/snn_demo/main.c](/run/media/mp2/af99f329-f82e-4702-883f-eb45eeaf5a26/vscode-linux/fpga-mcu/software/snn_demo/main.c).

It:

- initializes UART
- clears the SNN state
- feeds a built-in measurement sequence
- prints tracking outputs for the normal demo sequence
- prints detailed internal state in verbose mode
- runs a short recurrent probe
- repeats forever

## Quick Check

The normal tracking demo should print CSV rows like:

```text
S,0,1980,1980,4,1305,1167,13,1980,1980,792,396,-866,-1716,742,-2102,379,0,-866,0,-866,2
```

The exact last digit may vary by a few thousandths depending on build and
rounding after Q4.12 conversion, but:

- `cyc` should be `13`
- decoded `position` and `velocity` should stay close to the reference values
  checked by the log checker

You can also check a captured UART log against the expected demo sequence:

```bash
python3 tools/check_snn_demo_log.py /path/to/uart.log
```

The fixed coefficient tables embedded in the Verilog core can be regenerated
and checked with:

```bash
python3 tools/export_snn_coeffs.py
```

## Machine-Friendly UART Stream

The standalone firmware now defaults to a line-oriented telemetry protocol for
host tools. It prints a comment header, then one CSV row per SNN sample:

```text
type,t,measurement,delta,input_spikes,position,velocity,cycles,raw,draw,feat,dfeat,m0,m1,m2,m3,m4,beta,isum,rsum,mclip,status
S,0,1980,1980,4,1305,1167,13,1980,1980,792,396,-866,-1716,742,-2102,379,0,-866,0,-866,2
```

All numeric signal fields are raw signed `Q4.12` integers unless noted by the
field name. `type` is `S` for the built-in sample sequence, `P` for the
recurrent probe, and `H` for host-driven samples.

To watch and decode the stream:

```bash
python3 tools/stream_snn_uart.py --port /dev/ttyUSB0 --baud 1000000
```

To record decoded rows with extra floating-point columns:

```bash
python3 tools/stream_snn_uart.py --port /dev/ttyUSB0 --baud 1000000 --output snn_stream.csv
```

If `matplotlib` is installed, add `--plot` for live measurement, output, and
membrane traces. The plot defaults to `S` rows, which are the normal demo
sequence. Use `--plot-types P` for the recurrent probe, or `--plot-types S,P`
to show both with line breaks between row groups.

The same UART link can also be used to send host-driven samples to the firmware:

```text
H
C
D
M <measurement_raw>
S <measurement_raw> <input_spikes>
?
```

`H` enters host-input mode without clearing state. `C` clears the SNN and
host-mode measurement history, then stays in host-input mode. `D` returns to
the built-in demo stream. `M` lets firmware compute the spike bits from the
measurement history. `S` supplies explicit spike bits. `?` prints the current
mode and command summary.

To replay a raw-measurement CSV through the FPGA and plot the returned `H`
rows:

```bash
python3 tools/stream_snn_uart.py --port /dev/ttyUSB0 --baud 1000000 \
    --replay my_samples.csv --plot --output host_replay.csv
```

The replay CSV should include a `measurement` or `measurement_raw` column.

For a quick live-generated host stream, use `--generate`. Values are configured
in Q4.12 floating-point units and converted to raw samples before being sent:

```bash
python3 tools/stream_snn_uart.py --port /dev/ttyUSB0 --baud 1000000 \
    --generate sine --generate-amplitude 0.5 --generate-noise 0.03 \
    --plot --output generated_replay.csv
```

Supported generated waveforms are `sine`, `square`, `ramp`, and `walk`. Use
`--generate-count 0` to stream until interrupted. On completion or `Ctrl-C`,
generated mode prints error metrics comparing the FPGA position output against
the clean generated signal, and velocity output against the clean signal's
step-to-step delta. It also prints the input noise level actually sent to the
FPGA.

## Verbose Field Guide

When `SNN_DEMO_VERBOSE` is enabled in
[software/snn_demo/main.c](/run/media/mp2/af99f329-f82e-4702-883f-eb45eeaf5a26/vscode-linux/fpga-mcu/software/snn_demo/main.c),
each sample line includes extra internal values. These are all signed `Q4.12`
unless noted otherwise.

- `in`: input measurement sent to the estimator
- `pos`: output position estimate from the readout
- `vel`: output velocity estimate from the readout
- `cyc`: cycles used for that sample; currently expected to be `13`
- `raw`: latched measurement value inside the hardware block
- `draw`: latched measurement delta, computed as current measurement minus previous measurement
- `feat`: normalized measurement feature used by the readout, approximately `raw / 2.5` with clipping
- `dfeat`: normalized delta feature used by the readout, approximately `draw / 5.0` with clipping
- `m0`..`m4`: post-update membrane values for selected neurons after leak, input sum, recurrent sum, clipping, and threshold/reset
- `spk`: 4-bit externally supplied encoder spike pattern; for example `0x0004` means the positive-delta spike is active
- `beta`: leak term for neuron 0, roughly the previous membrane multiplied by the leak factor
- `isum`: input-weight contribution for neuron 0 from the current 4-bit spike input
- `rsum`: recurrent-weight contribution for neuron 0 from the previous timestep's neuron spikes
- `mclip`: neuron 0 membrane after adding leak, input, and recurrence, then applying membrane clipping but before threshold subtraction
- `status`: raw `snn_status` register value; in normal completion prints it should indicate `done=1`

## Recurrent Probe

After the main measurement sequence, the firmware runs a short recurrent probe
that repeatedly injects the positive-delta spike bit pattern `0x0004`.

Representative output:

```text
probe 0 inj=0x0004 pos=-0.041 vel=0.126 m4=0.093 cyc=13
probe 1 inj=0x0004 pos=-0.080 vel=0.387 m4=0.174 cyc=13
probe 2 inj=0x0004 pos=-0.189 vel=0.489 m4=0.244 cyc=13
probe 3 inj=0x0004 pos=-0.209 vel=0.705 m4=0.508 cyc=13
```

The most important checks are:

- `snn_cycles` is near `13`
- the normal tracking outputs stay close to the reference sequence
- `m4` rises during the recurrent probe instead of simply decaying
- `rsum` becomes non-zero during recurrently active steps when verbose mode is enabled

## Notes

- The gateware in this repo has not been hardware-built in this session because
  the local environment here does not currently have LiteX/Migen installed.
- The current hardware PoC includes a basic recurrent path, but the full Python
  recurrent reservoir is still richer than the implemented hardware schedule.
- The peripheral coefficients are fixed in the current Verilog block and were
  derived from the same Python training flow used by the simulation.
