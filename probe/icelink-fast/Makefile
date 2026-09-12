# icelink-fast -- replacement DAPLink interface firmware for the MuseLab
# APM32F103CB probe on the Colorlight i9 baseboard.

CROSS   ?= arm-none-eabi-
CC       = $(CROSS)gcc
OBJCOPY  = $(CROSS)objcopy
SIZE     = $(CROSS)size

BUILD   ?= build
TARGET   = icelink-fast

TUSB ?= /home/yin/pico-sdk/lib/tinyusb

SRCS  = src/startup.c src/system.c src/main.c src/boot.c src/jtag.c \
        src/usb_descriptors.c
TUSB_SRCS = $(TUSB)/src/tusb.c \
            $(TUSB)/src/common/tusb_fifo.c \
            $(TUSB)/src/device/usbd.c \
            $(TUSB)/src/device/usbd_control.c \
            $(TUSB)/src/class/cdc/cdc_device.c \
            $(TUSB)/src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c

OBJS  = $(SRCS:%.c=$(BUILD)/%.o) $(TUSB_SRCS:$(TUSB)/%.c=$(BUILD)/tusb/%.o)

CFLAGS  = -mcpu=cortex-m3 -mthumb -Os -g3 -std=c11
CFLAGS += -ffunction-sections -fdata-sections -fno-common
CFLAGS += -Wall -Wextra -Wundef -Werror=implicit-function-declaration
CFLAGS += -Isrc -Ivendor/cmsis/Include -I$(TUSB)/src
CFLAGS += -DSTM32F103xB
# TinyUSB is third-party: do not fail its build on our warning settings.
TUSB_CFLAGS = $(filter-out -Wall -Wextra -Wundef -std=c11,$(CFLAGS)) -std=gnu11 -w

# newlib-nano supplies memcpy/memset/strlen, which TinyUSB (and gcc's own
# codegen) call. -nostartfiles keeps our Reset_Handler as the entry point.
LDFLAGS  = -T ld/apm32f103cb_if.ld -nostartfiles --specs=nano.specs
LDFLAGS += -Wl,--gc-sections -Wl,-Map=$(BUILD)/$(TARGET).map

all: $(BUILD)/$(TARGET).bin check

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/tusb/%.o: $(TUSB)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(TUSB_CFLAGS) -c $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJS)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@
	@$(SIZE) $@

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

# Refuse to ship an image the bootloader would reject or that overruns the slot.
check: $(BUILD)/$(TARGET).bin
	@python3 tools/check_image.py $(BUILD)/$(TARGET).bin

clean:
	rm -rf $(BUILD)

.PHONY: all clean check

# Flash the interface slot over SWD. Bootloader is never erased.
flash: $(BUILD)/$(TARGET).bin check
	openocd -f tools/openocd-flash.cfg

# Re-verify the bootloader half against the teardown dump.
verify-bootloader:
	openocd -f tools/openocd-verify-bl.cfg

# Flash over USB via the stm32duino DFU bootloader -- no SWD, no grabbers.
# Reset the probe (or replug USB) first, then run this inside the DFU window.
dfu: $(BUILD)/$(TARGET).bin check
	dfu-util -a 2 -d 1eaf:0003 -D $(BUILD)/$(TARGET).bin

dfu-list:
	dfu-util -l

# Flash over USB via the DAPLink bootloader's MAINTENANCE drive (no grabbers).
# Requires the probe to be sitting in the bootloader; reset after copying.
MAINT ?= /run/media/$(USER)/MAINTENANCE
usb-flash: $(BUILD)/$(TARGET).bin check
	@test -d "$(MAINT)" || { echo "MAINTENANCE not mounted at $(MAINT)"; \
	  echo "the probe is running an app -- reset into the bootloader first"; exit 1; }
	cp $(BUILD)/$(TARGET).bin "$(MAINT)/" && sync
	@echo "copied -- now reset the probe (host/ch347_nrst.py --low/--high, or replug USB)"
