BUILD_DIR?=../../build/icepi_zero
LITEX_ROOT?=../../litex-setup
TRIPLE?=riscv64-linux-gnu

include $(BUILD_DIR)/software/include/generated/variables.mak

SOC_DIRECTORY := $(LITEX_ROOT)/litex/litex/soc
CPU_DIRECTORY := $(SOC_DIRECTORY)/cores/cpu/vexriscv
PICOLIBC_DIRECTORY := $(LITEX_ROOT)/pythondata-software-picolibc/pythondata_software_picolibc/data
COMPILER_RT_DIRECTORY := $(LITEX_ROOT)/pythondata-software-compiler_rt/pythondata_software_compiler_rt/data
LIBC_DIRECTORY := $(SOC_DIRECTORY)/software/libc
LIBCOMPILER_RT_DIRECTORY := $(SOC_DIRECTORY)/software/libcompiler_rt
LIBBASE_DIRECTORY := $(SOC_DIRECTORY)/software/libbase
LIBFATFS_DIRECTORY := $(SOC_DIRECTORY)/software/libfatfs
LIBLITESPI_DIRECTORY := $(SOC_DIRECTORY)/software/liblitespi
LIBLITEDRAM_DIRECTORY := $(SOC_DIRECTORY)/software/liblitedram
LIBLITEETH_DIRECTORY := $(SOC_DIRECTORY)/software/libliteeth
LIBLITESDCARD_DIRECTORY := $(SOC_DIRECTORY)/software/liblitesdcard
LIBLITESATA_DIRECTORY := $(SOC_DIRECTORY)/software/liblitesata
BIOS_DIRECTORY := $(SOC_DIRECTORY)/software/bios
BUILDINC_DIRECTORY := $(BUILD_DIR)/software/include

include $(SOC_DIRECTORY)/software/common.mak

# ---- LVGL build -------------------------------------------------------------
LVGL_DIR   := ../lvgl
LVGL_BUILD := lvgl_build

# Pull in every C file under lvgl/src/ except:
#  - drivers/   : HW-specific display/input drivers (SDL, FBDEV, ...); we
#                 write our own glue in this demo.
#  - vg_lite_*  : Vivante GPU bindings - headers not vendored.
LVGL_SRCS := $(shell find $(LVGL_DIR)/src -name '*.c' \
              -not -path '*/drivers/*' \
              -not -path '*/vg_lite_tvg/*' \
              -not -path '*/vg_lite_driver/*')
LVGL_OBJS := $(patsubst $(LVGL_DIR)/src/%.c,$(LVGL_BUILD)/%.o,$(LVGL_SRCS))

CFLAGS += -I$(LVGL_DIR) -I$(LVGL_DIR)/src -I. -DLV_CONF_INCLUDE_SIMPLE

# Shared firmware modules for the MNIST touch demo: LCD bring-up, HW I2C /
# FT6336U touch, UART log, and the MCP23S17 expander driver (lcd.c drives
# LCD/CTP reset through it on this variant -- see the CSR_IOX_RESET_BASE path).
# mcp23s17.c #includes "aux_spi.h", which resolves to winc_test/aux_spi.h below.
COMMON_DIR := ../common
CFLAGS    += -I$(COMMON_DIR)
COMMON_OBJS = lcd.o touch.o log.o mcp23s17.o

# Shared aux SPI bus HAL lives in winc_test/. This demo uses it ONLY for the
# MCP23S17 expander (common/mcp23s17.c) that drives LCD/CTP reset -- there is no
# IMU/ADC/SD logging in this MNIST-only build. Requires the aux bus + expander
# CS, i.e. icepi_zero_mnist_lcd.py (add_aux_imu with_iox=True). aux_spi.o is
# built via the explicit rule below (not VPATH -- see the note there).
WINC_DIR := ../winc_test
CFLAGS   += -I$(WINC_DIR)
AUX_OBJS  = aux_spi.o
# LVGL doesn't satisfy LiteX's stricter warning set; relax those without
# silencing real bugs.
CFLAGS += -Wno-strict-prototypes -Wno-old-style-definition -Wno-missing-prototypes
# Help the linker drop the large slice of LVGL we don't reference.
CFLAGS += -ffunction-sections -fdata-sections

# Debug HUD: the on-screen FPS box (bottom-right) + memory box (bottom-left).
# Off by default so shipping builds are clean. Re-enable for bring-up with:
#   make DEBUG_HUD=1
ifdef DEBUG_HUD
CFLAGS += -DMNIST_DEBUG_HUD
endif

OBJECTS = main.o crt0.o $(COMMON_OBJS) $(AUX_OBJS)

all: mnist_lcd_demo.bin

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@
ifneq ($(OS),Windows_NT)
	chmod -x $@
endif

vpath %.a $(PACKAGES:%=$(BUILD_DIR)/software/%)

mnist_lcd_demo.elf: $(OBJECTS) $(LVGL_OBJS)
	$(CC) $(LDFLAGS) -T linker.ld -N -o $@ \
		$(OBJECTS) $(LVGL_OBJS) \
		$(PACKAGES:%=-L$(BUILD_DIR)/software/%) \
		-Wl,--whole-archive \
		-Wl,--gc-sections \
		-Wl,-Map,$@.map \
		$(LIBS:lib%=-l%)

ifneq ($(OS),Windows_NT)
	chmod -x $@
endif

-include $(OBJECTS:.o=.d)
-include $(LVGL_OBJS:.o=.d)

VPATH = $(BIOS_DIRECTORY):$(CPU_DIRECTORY):$(COMMON_DIR)

%.o: %.c
	$(compile)

# aux_spi.c lives in winc_test/. Build it into a LOCAL object from source --
# don't put winc_test on VPATH, or make would pick up winc_test's prebuilt
# aux_spi.o (compiled against a different SoC's csr.h, and unresolvable by the
# linker under its bare name). Explicit rule => compiled fresh against our
# generated headers.
# aux_spi.c calls printf() on its timeout path but leaves the declaration to
# its includer (winc_test force-includes a prelude that provides it). Force
# <stdio.h> for just this object so the shared source stays untouched.
aux_spi.o: CFLAGS += -include stdio.h
aux_spi.o: $(WINC_DIR)/aux_spi.c
	$(compile)

%.o: %.S
	$(assemble)

$(LVGL_BUILD)/%.o: $(LVGL_DIR)/src/%.c
	@mkdir -p $(@D)
	$(CC) -c $(CFLAGS) -o $@ $<

clean:
	$(RM) $(OBJECTS) mnist_lcd_demo.elf mnist_lcd_demo.bin mnist_lcd_demo.elf.map .*~ *~
	$(RM) -r $(LVGL_BUILD)

.PHONY: all clean
