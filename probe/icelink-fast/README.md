# icelink-fast

Replacement DAPLink *interface* firmware for the MuseLab APM32F103CB probe on
the Colorlight i9 baseboard. Goal: a lean JTAG streamer, no MSC/drag-and-drop.

## Layout on the probe

    0x08000000  48K  stock DAPLink bootloader (restored from the teardown dump)
    0x0800C000  80K  our app -- this project

## PB15 = USB_ENUM -- the thing that gates all USB

From the board schematic: **PB15 gates the USB D+ pull-up**, and PA9 is LED_RUN.
Nothing enumerates until PB15 is driven high as a push-pull output. A device
that never asserts it shows `USB_ISTR = 0` forever -- peripheral fully
initialised, host completely unaware anything is plugged in.

This is why the stm32duino DFU bootloader was a dead end here: it has no idea
PB15 exists, so it can never enumerate on this board without binary patching.
It was abandoned for that reason; `tools/stm32duino_boot20_pb0_8k.bin` and the
udev rule are kept only as references.

Confirmed by register diff between the two bootloaders:

    DAPLink (enumerates)  GPIOB_CRH 0x34444443  ODR 0x8011  -> PB15 out, high
    stm32duino (silent)   GPIOB_CRH 0x44444444  ODR 0x0011  -> PB15 floating

## Getting back into the bootloader without NRST

`src/boot.c`. The DAPLink bootloader launches whatever is at 0x0800C000 if its
vector table looks valid, and shows MAINTENANCE when it does not -- so
"reboot to bootloader" is: erase our own first flash page, then reset via
`AIRCR.SYSRESETREQ`. No reset pin, no DAPLink shared-RAM magic to reverse.

    request_bootloader()   sets _boot_magic (0x20004ff0), SYSRESETREQ
    boot_check()           runs early in Reset_Handler; on the magic, calls
    enter_bootloader()     erases page 0 of the app, then SYSRESETREQ

`enter_bootloader()` lives in `.ramfunc` and executes from SRAM: the whole image
is under 1 KB, so it sits *inside* the page being erased and would fault on the
next instruction fetch if it ran from flash. `_boot_magic` sits above `_estack`
in a linker-reserved 16-byte hole so neither the stack nor .bss can touch it;
it survives SYSRESETREQ but not a power cycle, which is the behaviour we want.

Verified on hardware: after triggering, 0x0800C000 reads 0xffffffff, VTOR is
0x08000000, FLASH_CR is re-locked, and MAINTENANCE mounts.

Once the USB stack exists, a vendor command calls `request_bootloader()` and
the whole loop is host-driven. Until then it is triggered over SWD by writing
0xB007B007 to 0x20004ff0 and resetting.

## Flashing over USB (no grabbers)

The DAPLink bootloader presents a MAINTENANCE mass-storage drive whenever it is
not running a valid app. Copy the image onto it:

    make usb-flash      # cp build/icelink-fast.bin to the MAINTENANCE drive

Then reset the probe. The bootloader writes the image but stays in MAINTENANCE
until it is reset -- do not read that as a rejection.

SWD stays available as the backstop: `make flash`.

## Why the JTAG pins are awkward

TCK=PB13, TMS=PB14, TDI=PB8, TDO=PB9 (corroborated against the stock image:
GPIOB 0/6/8/9/12/13/14 configured, 11 BSRR refs, unrolled bitbang sequences).
TCK lands on SPI2_SCK but TMS took MISO and TDI is not on MOSI, so hardware SPI
is unusable. All four are in GPIOB bits 0-15, so one BSRR write sets the whole
bus state. PB13 is TIM1_CH1N, which keeps a hardware-clocked TIM1+DMA path open
later (needs BDTR.MOE -- easy to forget, the pin stays silent without it).

USB full-speed caps throughput near 8 Mbit/s of TCK regardless, so the unrolled
bitbang loop is not the bottleneck.

## Bench procedure

Needs pyusb: `pip install pyusb` (or `.venv/bin/pip install pyusb`).

NRST is wired to the CH347's TRST pin = **GPIO5** (pin 9, DTR0/SCS1/TRST).
OpenOCD cannot drive it in SWD mode, so it is set out-of-band:

    python3 host/ch347_nrst.py --state       # read first, change nothing
    python3 host/ch347_nrst.py --high        # or --low, see below
    make verify-bootloader                   # diff against the teardown dump
    make flash                               # programs 0x0800C000 only
    python3 host/ch347_nrst.py --release

**Polarity: NRST must be driven HIGH** (verified 2026-09-01). Left alone, GPIO5
reads as an input pulled high and SWD returns `DPIDR 0xfe0c001b`; driven high as
an output it returns the correct `0x2ba01477`. OpenOCD also needs
`DBGMCU_CR = 0x307` right after halt, or DAPLink's unfed watchdog resets the
core and the link looks flaky. Both are baked into the configs.
grabbers on, try `--high` then `make verify-bootloader`; if OpenOCD cannot
attach, try `--low`. Whichever lets OpenOCD halt the core is the answer --
record it here once known.

Note that `program` runs a flash algorithm *on the core*, so the core must not
be held in reset during `make flash`. If the working state turns out to be
NRST low, it has to be released between attach and program.

## Hardware facts (measured, not assumed)

* **8 MHz crystal on OSC_IN/OSC_OUT, and it works.** SYSCLK 72 MHz, USB 48 MHz,
  verified: `RCC_CR 0x0303a083` (HSERDY+PLLRDY), `RCC_CFGR 0x001d040a`
  (SW=SWS=PLL, PLLSRC=HSE, x9, PPRE1=/2, USBPRE=/1.5).
* **A grabber on the crystal pins will stop the oscillator.** An earlier run
  hung forever in the HSERDY spin and looked exactly like "this board has no
  crystal" -- it was a test clip loading OSC_IN/OSC_OUT. If HSE ever fails to
  start, suspect the probe wiring before the silicon.
* **`clock_init` falls back to HSI** (HSI/2 * 12 = 48 MHz) if HSE does not come
  up within ~150 ms, rather than spinning forever. Both paths put USB on
  exactly 48 MHz. `g_clock_source` (0=HSI, 1=HSE, 2=no PLL) and `g_sysclk_hz`
  are globals -- read them over SWD to see which path ran. Measured: 1 / 72000000.
* **The bootloader takes a moment to hand off.** Sampling GPIO immediately
  after an NRST pulse catches the bootloader's own pin config
  (`CRL 0x43484447`), not ours (`0x43484443`). Wait, or sample repeatedly,
  before concluding an image was rejected.
* **PB0 is the LED.** Confirmed twice: our blink toggles it, and the
  stm32duino `pb0` bootloader build blinks it during the DFU window.
* **`daplink_info` at +0x20 was never required to launch** an image -- the old
  DAPLink bootloader ran ours with those words zeroed. Moot now; the field is
  still emitted but the stm32duino bootloader ignores it.
* **The bootloader hand-off is slow with no USB attached.** 6 s after reset it
  was still in the bootloader (PC 0x080006d2); by 20 s it had jumped to the app
  (VTOR 0x08002000, PC 0x08002274). Do not read an early sample as a failure.
* **`program` does not work with this adapter** ("Unable to reset target") and
  writes nothing while appearing to try. Use `flash write_image erase unlock`
  + `verify_image`, and reset by pulsing NRST externally.

## Status

- [x] Stage 0: boots in the interface slot, verified running on hardware
      (PB0 and PB6 both toggling, PC in our code, flash write verified)
- [x] Clock: HSE 8 MHz crystal OK -- SYSCLK 72 MHz, USB 48 MHz (HSI fallback kept)
- [x] NRST polarity recorded: drive GPIO5 HIGH; DBGMCU_CR=0x307 after halt
- [x] SWD link verified: CPUID 0x412fc231, bootloader SP/PC match the dump
- [x] USB CDC up (TinyUSB 0.18 stm32_fsdev + ST CMSIS headers in vendor/).
      Enumerates as cafe:4001 "ECP5 JTAG probe"; prints to
      /dev/serial/by-id/usb-icelink-fast_ECP5_JTAG_probe_000001-if00
- [x] **JTAG WORKS**: IDCODE = 0x41112043 (LFE5U-45F), stable across reads.
      Pinout CONFIRMED: TCK=PB13 TMS=PB14 TDI=PB8 TDO=PB9.
- [x] Dead-man timer: no USB enumeration within 20 s -> enter_bootloader().
      This is what makes every future image safe to flash blind.
- [x] USB flashing path WORKING end-to-end: copy to MAINTENANCE, reset, app
      runs (VTOR 0x0800c000, HSE 72 MHz, PB15 asserted by our code)
- [x] Escape hatch WORKING: self-erase + SYSRESETREQ from .ramfunc, verified
      end-to-end (app -> MAINTENANCE -> usb-flash -> app running)
- [ ] Move the trigger from "write magic over SWD" to a USB vendor command,
      which is what finally makes the grabbers optional
- [ ] DEAD-MAN TIMER (do this in the same build as USB): if USB has not
      enumerated ~20 s after boot, call enter_bootloader() on ourselves. Without
      it, any image that fails to enumerate strands the probe with SWD as the
      only way back -- which is exactly the hole today's build left open.
- [x] JTAG engine (src/jtag.c): TAP reset + IDCODE, unrolled BSRR bitbang,
      pins as four #defines. Verified against real silicon.

## Debugger gotcha

Halting the core kills USB: the host sees a disconnect and the device drops off
the bus, and `resume` does not bring it back -- it needs a reset. Several
"USB is broken" scares this session were just a config with no `resume` in it.
Always resume, or expect to reset afterwards.

## State as of 2026-09-01 (grabbers removed)

Probe is parked in the DAPLink bootloader with MAINTENANCE mounted, so the next
flash is `make usb-flash` + a USB replug. The app that was running erased its
own vector table to get there (the escape hatch, triggered over SWD).

Trigger the escape hatch (while SWD is still attached) with:

    openocd -f /tmp/setmagic.cfg      # writes 0xB007B007 to 0x20004ff0
    host/ch347_nrst.py --low; --high  # NRST reset -- NOT a debugger SYSRESETREQ,
                                      # which leaves the core halted and the app
                                      # never runs boot_check()

## SWD link notes (this bench)

The ch347 + IC-grabber link is unreliable: `DPIDR` reads garbage (0xfe0c001b)
or `dap init` fails on CSYSPWRUPACK, in long streaks. What actually fixes it,
in order: reseat SWDIO/SWCLK/GND, then unplug/replug the CH347 itself. Adapter
speed makes no difference; a software USB reset of the CH347 does not help.
`host/swd.sh` retries across speeds and is the normal way to run a config.

Also, once during this session a failed `flash write_image erase unlock`
left **readout protection enabled**, which presents as "stm32x device
protected" plus an unreadable 0x1ffff7e2. Fix: `stm32f1x unlock 0`, then reset.

## The SODIMM/SWD constraint

The i9 is a SODIMM module and the SWD test clips occupy the same space: **you
cannot have the FPGA seated and the grabbers attached at the same time.** Any
test involving the ECP5 must therefore be readable over USB, and any firmware
flashed before unclipping must be able to recover itself -- hence the dead-man
timer. A build without it that fails to enumerate is unrecoverable without
re-clipping, which has already cost one round trip.

## Verified JTAG result

    IDCODE = 0x41112043  LFE5U-45F  <-- MATCH   (stable, repeated)

TCK=PB13  TMS=PB14  TDI=PB8  TDO=PB9, all GPIOB, one BSRR write per edge.
TCK half-period is two nops at 72 MHz; raise it if longer wiring is used.

## CH347 as an external JTAG programmer -- investigated, parked 2026-09-01

Idea: send `z` to tri-state the F103's JTAG pins, clip a CH347 onto the same
nets, and use its hardware JTAG engine (USB high-speed, up to ~30 MHz) with
openFPGALoader instead of our bitbang. It does not work, and the reason is not
wiring.

Measured, not assumed:

* Both `openFPGALoader -c ch347_jtag` and OpenOCD's independent ch347 driver
  read **0xfc000003** instead of 0x41112043. Their agreement proves nothing --
  both hand byte streams to the same CH347 *hardware* engine, so a chip-level
  fault shows up identically in both.
* Our own capture (`c`) of the wire decodes to exactly 0xfc000003, so the
  adapter faithfully reports what is on the bus. The FPGA really returns that.
* The TMS sequence the CH347 emits is textbook-correct: 6x TLR, RTI,
  Select-DR, Capture-DR, Shift-DR, 32 shift clocks (42 clocks total at 469 kHz).
* TDI is driven on PB8 and the response arrives on PB9 -- measured -- so
  TDI/TDO orientation is correct, not merely believed to be.
* Failure is **identical at 469 kHz and 7.5 MHz**, and **identical with and
  without a ground clip**. Frequency- and ground-independent rules out both a
  timing budget problem and ringing/reflections.
* The returned bits are: correct for the first two, then zeros, then TDI-fill
  ones -- the shape of a shift register clocked past its contents.
* Our F103 reads 0x41112043 perfectly from the same nets, so the ECP5, the
  board routing and the clips are all fine.

Remaining hypothesis: the CH347's JTAG engine drives TMS/TDI on the same edge
the target samples them (setup/hold violation), leaving the TAP one clock out
of step. Chip version 5.44, firmware 0x45. A WCH firmware update is the only
obvious avenue, and it is Windows-tooling territory.

Not worth more time: our probe works, and the dev loop is dominated by nextpnr,
not by a 5 s bitstream load.

## Where to pick up

Working today:
  * probe reads IDCODE/USERCODE/STATUS over USB CDC, 2.16 MHz TCK, 270 kB/s
  * grabber-free loop: `b` -> MAINTENANCE -> `make usb-flash` -> replug USB
  * dead-man timer (20 s without USB -> back to bootloader) makes any image safe
  * `m` (net monitor) and `c` (TCK-edge capture) turn the probe into a JTAG
    logic analyser -- this is what diagnosed the CH347

Next, in order of value:
  1. Bitstream loader: ECP5 SRAM config over JTAG, then flash via the chip's
     JTAG-to-SPI background mode. This is the actual goal.
  2. Two cheap speed wins first (~1 hour, no new hardware): merge the TCK-clear
     into the next bit's TDI store (3 APB stores/bit -> 2), and skip the TDO
     read on write-only shifts. Expect ~20 cycles/bit read, ~13 write,
     i.e. ~3.6 MHz / ~5.5 MHz -> roughly 3 s per bitstream.
  3. Make hi-Z the power-on default with `a` to acquire, so replugging the
     probe while something else is clipped on cannot cause contention.
  4. `probe/` is still untracked on branch experiment/vexiiriscv-turbo. Give it
     its own branch and commit.

Do not lose: the factory DAPLink dump on mpc2
(/run/media/mp2/workspace/vscode-linux/daplink-apm32-dump). Its bootloader is
restored on the probe and is the only thing making USB flashing possible; there
is no second copy.
