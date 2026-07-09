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

# Shared firmware modules: LCD bring-up, HW I2C / FT6336U, UART log.
COMMON_DIR := ../common
CFLAGS    += -I$(COMMON_DIR)
COMMON_OBJS = lcd.o touch.o log.o

# On an expander build (icepi_zero_mnist_lcd: CSR_IOX_RESET_BASE), lcd.c drives
# LCD/CTP reset through the MCP23S17 over the aux SPI bus, so pull in those
# drivers. On the plain LCD top (icepi_zero_lcd) the direct-pin reset path is
# used and neither is needed. Detect from the generated CSR header.
AUX_OBJS =
HAS_IOX := $(shell grep -q IOX_RESET_BASE $(BUILDINC_DIRECTORY)/generated/csr.h 2>/dev/null && echo 1)
ifeq ($(HAS_IOX),1)
COMMON_OBJS += mcp23s17.o
WINC_DIR := ../winc_test
CFLAGS   += -I$(WINC_DIR)
AUX_OBJS  = aux_spi.o
endif

OBJECTS = main.o crt0.o $(COMMON_OBJS) $(AUX_OBJS)

all: tetris.bin

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@
ifneq ($(OS),Windows_NT)
	chmod -x $@
endif

vpath %.a $(PACKAGES:%=$(BUILD_DIR)/software/%)

tetris.elf: $(OBJECTS)
	$(CC) $(LDFLAGS) -T linker.ld -N -o $@ \
		$(OBJECTS) \
		$(PACKAGES:%=-L$(BUILD_DIR)/software/%) \
		-Wl,--whole-archive \
		-Wl,--gc-sections \
		-Wl,-Map,$@.map \
		$(LIBS:lib%=-l%)

ifneq ($(OS),Windows_NT)
	chmod -x $@
endif

-include $(OBJECTS:.o=.d)

VPATH = $(BIOS_DIRECTORY):$(CPU_DIRECTORY):$(COMMON_DIR)

%.o: %.c
	$(compile)

# aux_spi.c lives in winc_test/; build a LOCAL object from source against our
# generated headers (don't VPATH it -- winc_test ships a prebuilt aux_spi.o for
# a different SoC). It printf()s on its timeout path but leaves the declaration
# to its includer, so force <stdio.h> for just this object.
ifeq ($(HAS_IOX),1)
aux_spi.o: CFLAGS += -include stdio.h
aux_spi.o: $(WINC_DIR)/aux_spi.c
	$(compile)
endif

%.o: %.S
	$(assemble)

clean:
	$(RM) $(OBJECTS) tetris.elf tetris.bin tetris.elf.map .*~ *~

.PHONY: all clean
