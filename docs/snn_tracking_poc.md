# SNN Tracking PoC

This PoC explores a very small leaky integrate-and-fire (LIF) spiking neural
network as a streaming estimator for 1D motion tracking.

## Goal

Estimate position and velocity from a noisy position measurement stream and
compare the result against a standard constant-velocity Kalman filter.

The point is not to beat a Kalman filter in general. The point is to create a
small temporal estimator that:

- is easy to simulate and verify
- has a direct fixed-point implementation path
- maps naturally to FPGA hardware

## Current Simulation Layout

- [sim/generate_tracking_data.py](/run/media/mp2/af99f329-f82e-4702-883f-eb45eeaf5a26/vscode-linux/fpga-mcu/sim/generate_tracking_data.py)
  creates synthetic 1D motion sequences with noisy measurements
- [sim/kalman_1d.py](/run/media/mp2/af99f329-f82e-4702-883f-eb45eeaf5a26/vscode-linux/fpga-mcu/sim/kalman_1d.py)
  provides the constant-velocity Kalman baseline
- [sim/lif_reservoir_1d.py](/run/media/mp2/af99f329-f82e-4702-883f-eb45eeaf5a26/vscode-linux/fpga-mcu/sim/lif_reservoir_1d.py)
  implements a small 8-neuron recurrent LIF reservoir and a trained linear
  readout, plus a hardware-schedule execution model
- [sim/run_tracking_poc.py](/run/media/mp2/af99f329-f82e-4702-883f-eb45eeaf5a26/vscode-linux/fpga-mcu/sim/run_tracking_poc.py)
  runs the end-to-end comparison

## Model Summary

Exploration pipeline:

```text
measurement z[t]
-> 4-channel spike encoder
-> 8-neuron recurrent LIF reservoir
-> linear readout
-> estimated position and velocity
```

Current encoder channels:

- positive position level
- negative position level
- positive measurement delta
- negative measurement delta

The readout currently uses:

- all 8 membrane states
- normalized measurement tap
- normalized measurement-delta tap
- bias term

Adding the two direct observation taps keeps the design small while making the
estimator much more realistic for hardware.

## Hardware Prototype

The currently validated FPGA datapath is a simplified but faithful hardware
prototype:

- firmware-side 4-bit threshold encoder
- 8-neuron feedforward LIF bank in Verilog
- normalized measurement and delta taps
- 2-output fixed-point readout in Verilog
- LiteX CSR wrapper for SoC integration

This prototype intentionally drops recurrence for now. The goal was to get a
small estimator running correctly on hardware before deciding whether the extra
state coupling is worth the timing and verification cost.

## Hardware-Schedule Model

The simulation includes a hardware-oriented execution path that models:

- sequential neuron updates
- sequential readout accumulation
- quantized arithmetic at each step

For the current feedforward hardware build, the realized schedule is:

- `1` cycle to latch input and update the membrane bank
- `11` readout MAC cycles
- `1` cycle to publish the final outputs

That gives `13` cycles per input sample on hardware.

## Chosen Numeric Format

For the current PoC, the SNN path uses one shared fractional scale with
explicit clip ranges per value class:

- fractional bits: `12`
- membrane state clip: `+/-3.999`
- weight clip: `+/-3.999`
- feature clip: `+/-1.999`
- readout accumulator clip: `+/-7.999`

This is intentionally simple for a first FPGA version:

- one common fractional alignment means straightforward shift-based scaling
- membrane and accumulator limits are explicit
- the hardware-schedule model now matches the high-level SNN path exactly

## Running The Sim

From the repo root:

```bash
python3 sim/run_tracking_poc.py
```

To emit a per-timestep trace for one generated test sequence:

```bash
python3 sim/run_tracking_poc.py --trace-out /tmp/snn_trace.csv
```

To emit a cycle-level hardware trace with neuron partial sums and readout
accumulator values:

```bash
python3 sim/run_tracking_poc.py --hw-trace-out /tmp/snn_hw_trace.csv
```

The trace CSV includes:

- raw measurement and measurement delta
- ground-truth position and velocity
- SNN estimates
- floating-point Kalman estimates
- quantized Kalman estimates
- hardware-schedule SNN estimates
- encoder spikes
- reservoir spikes
- per-neuron membrane state

The hardware trace CSV includes:

- per-neuron leak term
- per-neuron input and recurrent partial sums
- per-neuron post-threshold membrane value
- per-readout feature value
- per-readout product
- per-readout accumulator state

## Current Observations

- The tiny SNN estimator is worse than the floating-point Kalman baseline, as
  expected, but it is already tracking the state reasonably well.
- The naive quantized Kalman path degrades sharply in the current setup. That
  is a useful result because it highlights how sensitive recursive estimation
  can be to fixed-point choices.
- The SNN path is already largely quantized internally, which makes it a good
  candidate for the first FPGA estimator experiment.
- The current Verilog feedforward implementation now matches the Python
  feedforward reference on hardware for the demo sequence, including the
  normalized direct features and 2-output readout.
- The timing-friendly sequential MAC readout raises sample latency to `13`
  cycles, but it is a much more practical tradeoff than a single-cycle wide
  readout tree.

## Next Steps

- clean up the debug-facing hardware interface now that the datapath is stable
- add a host-side checker to compare UART output against the reference sequence
- measure resource use and timing after trimming bring-up-only debug logic
- decide whether recurrence is worth reintroducing for this PoC
