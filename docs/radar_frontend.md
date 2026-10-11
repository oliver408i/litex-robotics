# CDM324 analog front end — the gain stage ahead of the MCP3008

Recorded 2026-09-20. Breadboard, as built.

Nothing in this document is gateware, which is why it is not described anywhere
in `targets/colorlight_i9/adc.py` or `gateware/mcp3008.py` — those start at the
ADC's input pin. This file covers what happens before that pin, because without
it none of the numbers in `tools/adc_scope.py` mean anything.

The CDM324 emits a Doppler IF that is small and sits on its own DC offset. The
ADC is unipolar, 10-bit, and referenced to 3V3. The front end therefore has four
jobs: block the module's DC, re-centre the signal at mid-scale, amplify it
enough to occupy a useful number of ADC codes, and band-limit it before
sampling.

```
CDM324 IF -> 1 uF -> +IN A (100k to VBIAS)
                       |
                    LM358 A, non-inverting, Rf/Rg = 200k/10k -> G = 21
                       |
                  OUT A -> 1k -> MCP3008 CH1
                                   |
                                 22 nF
                                   |
                                  GND
```

## Bias: half-scale, buffered, and derived from the ADC rail

A 10k/10k divider off **3V3** gives the raw midpoint (~1.62 V measured). That
node is bypassed to ground and buffered by the **second half of the LM358**
wired as a unity-gain follower (+IN B = pin 5, OUT B = pin 7 tied to -IN B =
pin 6). Pin 7 is the `VBIAS` used by the rest of the stage.

**The divider must not be used directly as the analog ground.** Its Thevenin
impedance is 10k || 10k = 5k, and the gain network's 10k leg would load it and
shift the operating point. The follower is what makes `VBIAS` stiff.

**Taking the bias from 3V3 rather than the op-amp's 5 V rail is the right
choice and worth not "tidying up" later:** `VREF` is that same 3V3, so the bias
and the reference move together and mid-scale stays at code 512 under rail
drift. A bias derived from the 5 V rail would drift against `VREF`.

## AC coupling — high-pass at 1.59 Hz

1 uF in series, 100k to `VBIAS` on the amp side:

    f_hp = 1 / (2*pi * 100k * 1u) = 1.59 Hz

The capacitor blocks the module's DC; the 100k sets the input-side DC level to
`VBIAS`.

**This corner is deliberately below the Doppler band, which means it does not
reject drift.** Everything from 1.59 Hz up passes, including slow thermal and
flicker components. That is handled digitally instead — `tools/adc_log.py` and
`tools/adc_scope.py` subtract the window mean before any spectral work. If
low-frequency wander ever dominates a capture, raising this corner to ~10-20 Hz
is the analog fix; the Doppler band of interest starts at ~50 Hz (0.3 m/s).

## Gain — 21x, adjustable by one resistor

Amplifier A is non-inverting, referenced to `VBIAS` rather than ground:

    G = 1 + Rf/Rg,   Rg = 10k (-IN A to VBIAS),   Rf = 200k (two 100k in series)
    G = 21

so `V_out = VBIAS + 21 * V_if_ac`, with DC output sitting at `VBIAS`.

Referred to the input, one ADC count is:

    3.22 mV / 21 = 153 uV at the CDM324's IF pin

**21x is on the low side for a bare CDM324 and the stage is built to be changed.**
Series 100k resistors set the gain directly:

| Rf | G |
|---|---|
| 100k | 11 |
| 200k | 21 (current) |
| 300k | 31 |
| 500k | 51 |
| 1M | 101 |

Headroom check before raising it: the swing must stay inside 0 < code < 1023,
i.e. roughly +-1.6 V around `VBIAS` at the ADC, so +-77 mV of IF at G = 21 and
+-16 mV at G = 101. Clipping shows up as a flat-topped waveform and as spectral
splatter — harmonics of the true Doppler line — rather than as a missing signal.

**Gain does not improve SNR.** It only makes weak returns occupy more codes so
quantisation stops dominating. If the noise floor rises with the signal when Rf
goes up, the limit is upstream and more gain will not help.

## Anti-alias — first order, 7.2 kHz

    f_lp = 1 / (2*pi * 1k * 22n) = 7.23 kHz

**Call this a noise trim, not an anti-alias filter.** Sampling at 20 ksps puts
Nyquist at 10 kHz, and a first-order roll-off is only about -5 dB at 12.8 kHz —
the frequency that folds back onto 7.2 kHz. Out-of-band content between 10 and
20 kHz is attenuated, not removed, and what survives lands inside the band as a
mirror image. It is adequate because the CDM324's IF is itself band-limited and
nothing in the rig generates strong content up there; it would not be adequate
against a deliberate interferer or a switching supply near 15 kHz. If a
spectral line ever appears that mirrors around 10 kHz when the sample rate is
changed, this is why.

The 1k has a second job: the LM358 runs from **5 V** and can swing to roughly
3.5 V, which is above the MCP3008's 3V3 supply. If the amp saturates high, the
1k limits the input-clamp current to ~0.2 mA, which is safe. **Do not replace
it with a wire.**

## LM358P pinout

```
        LM358P
      +--------+
OUT A |1      8| V+ (5 V)
-IN A |2      7| OUT B = VBIAS
+IN A |3      6| -IN B
 GND  |4      5| +IN B
      +--------+
```

A: radar gain. B: virtual-ground buffer.

## Expected ADC behaviour, and how to tell the states apart

At rest the output sits at `VBIAS`, i.e. near code 512. Captures have shown a
centre somewhat above 512 from amplifier offset multiplied by the gain; this is
expected and is removed by the mean subtraction in the host tools. The
condition that matters is that the waveform stays inside 0 and 1023 without
clipping.

Four displays that look similar and are not:

| what you see | what it means |
|---|---|
| flat at ~512, nothing on motion | signal below the LSB, or the front end unpowered — check `VBIAS` is ~1.62 V at pin 7 first |
| flat at 1023 on every channel | nothing answering on the SPI bus; MISO is pulled up in the FPGA (see `gateware/mcp3008.py`) — this is a bus fault, not an analog one |
| flat at 0 | the part answers with D_OUT stuck low — also a bus fault |
| empty spectrum, correct DC centre | **correct** for a stationary scene: CW Doppler produces no AC at zero relative velocity |

The fourth row is the one to keep in mind, and the reason it is worth listing
next to the others: a working stage watching a still room and a dead stage
produce the same empty scope. The distinguishing test is motion — a hand wave
must put 40-200 Hz on the display (0.25-1.25 m/s at 160.94 Hz per m/s).

## 60 Hz is in-band and the gain applies to it

The capture chain was originally calibrated on mains hum picked up by a
floating input: 60.08 Hz at 63 dB over the noise floor, harmonics within
0.08 Hz of exact multiples (see `targets/colorlight_i9/README.md`). With the
front end attached that hum is no longer a convenient calibration source — it
is an interferer sitting at **0.37 m/s**, inside the hand-wave band, and it is
amplified 21x along with everything else.

Practical consequences: keep the CDM324's leads short, keep the breadboard away
from mains wiring and the PSU, and treat a persistent line at exactly 60 Hz (and
120, 180, 240) as hum rather than as a target. The scope's peak-hold makes this
easy to see — hum is stationary in the waterfall, a real target drifts.

## What has not been verified

- Component values here are as-built on breadboard and were recorded from the
  bench, not re-measured against the assembled circuit for this document.
- No capture against a real moving target has been recorded yet. Until one
  exists, the chain is verified only as far as "mains hum reproduces correctly
  through the digital path," which says nothing about the CDM324 itself.
- The front end has no supply decoupling documented. The LM358's 5 V rail and
  the CDM324's supply are both worth a local 100 nF if noise turns out to be
  rail-borne.
