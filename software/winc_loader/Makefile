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

# ---- ATWINC1500 host driver (Microchip) --------------------------------------
HOST_DRV := ../host_drv
WINC_TEST := ../winc_test

# Same driver-port flags as winc_test (see its Makefile for the full why):
# force-included conf_winc.h/winc_prelude.h, section GC, -fcommon. The shared
# port files (conf_winc.h, aux_spi.h, mdns.h, wifi_secrets.h, ...) resolve
# from ../winc_test -- single copy, no fork.
CFLAGS += -I. -I$(WINC_TEST) -I../common -I$(HOST_DRV) -include conf_winc.h -include winc_prelude.h
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -fcommon

# Lean keep-list: WINC core + sockets only (spi_flash.o = WINC's own serial
# flash, a live nm_drv_init dep -- NOT the board flash this app programs).
WINC_OBJECTS = \
	m2m_wifi.o m2m_hif.o \
	nmasic.o nmdrv.o nmbus.o nmspi.o \
	nm_common.o socket.o spi_flash.o

# Board glue + aux-bus SPI HAL + mDNS responder, sourced from ../winc_test.
PORT_OBJECTS = nm_bsp_icepi.o nm_bus_wrapper_icepi.o aux_spi.o mdns.o

OBJECTS = main.o crt0.o flash_w25q.o chain_stub.o $(PORT_OBJECTS) $(WINC_OBJECTS)

all: winc_loader.bin

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@
ifneq ($(OS),Windows_NT)
	chmod -x $@
endif

vpath %.a $(PACKAGES:%=$(BUILD_DIR)/software/%)

winc_loader.elf: $(OBJECTS)
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

# WINC driver sources + the port files shared with winc_test. Type-scoped
# vpath (not flat VPATH): ../winc_test also holds that app's compiled .o
# files, which a flat VPATH would satisfy .o prerequisites with -- they'd be
# stale (old csr.h) and never land in this directory for the link.
vpath %.c $(BIOS_DIRECTORY):$(CPU_DIRECTORY):$(WINC_TEST):$(HOST_DRV)/driver/source:$(HOST_DRV)/common/source:$(HOST_DRV)/socket/source:$(HOST_DRV)/spi_flash/source
vpath %.S $(BIOS_DIRECTORY):$(CPU_DIRECTORY)

%.o: %.c
	$(compile)

%.o: %.S
	$(assemble)

clean:
	$(RM) $(OBJECTS) winc_loader.elf winc_loader.bin winc_loader.elf.map .*~ *~

.PHONY: all clean
