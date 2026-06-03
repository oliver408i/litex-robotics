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

# Force CONF_WINC_USE_SPI visible to every TU (the driver headers gate on it
# before any conf_winc.h include) and let "driver/include/..."-style includes
# resolve from the host_drv root. -I. picks up conf_winc.h + aux_spi.h.
# winc_prelude.h force-includes <stdlib.h> for C TUs (the WINC driver uses
# malloc/free without including it; GCC 14+/15 errors on implicit decls). It is
# __ASSEMBLER__-guarded so the crt0.S assembly pass stays clean.
CFLAGS += -I. -I$(HOST_DRV) -include conf_winc.h -include winc_prelude.h
# Function/data-level GC so --gc-sections can drop unused WINC APIs (e.g. the
# connect/enterprise paths in m2m_wifi.c that pull in malloc/free -> no heap
# needed) and keep the binary lean.
CFLAGS += -ffunction-sections -fdata-sections
# -fcommon: nm_common.h defines an uninitialized global (at_sb_printf) in the
# header; GCC 10+ defaults to -fno-common, which makes each TU's tentative
# definition a hard multiple-definition error. -fcommon merges them.
CFLAGS += -fcommon

# Lean keep-list: WINC core + sockets only. OTA / SSL / crypto / ATE / periph /
# spi_flash / i2c+uart bus drivers / *_crt variants are intentionally NOT built;
# --gc-sections (below) also drops any unreferenced code that slips in.
# spi_flash.o is required: nm_drv_init() calls spi_flash_enable() in the live
# init path, so it can't be dropped despite us not reflashing WINC firmware.
WINC_OBJECTS = \
	m2m_wifi.o m2m_hif.o \
	nmasic.o nmdrv.o nmbus.o nmspi.o \
	nm_common.o socket.o spi_flash.o

# Board glue (this project) + the shared aux-bus SPI HAL.
PORT_OBJECTS = nm_bsp_icepi.o nm_bus_wrapper_icepi.o aux_spi.o

OBJECTS = main.o crt0.o $(PORT_OBJECTS) $(WINC_OBJECTS)

all: winc_test.bin

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@
ifneq ($(OS),Windows_NT)
	chmod -x $@
endif

vpath %.a $(PACKAGES:%=$(BUILD_DIR)/software/%)

winc_test.elf: $(OBJECTS)
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

# Where to find the WINC driver sources (board glue + HAL live in this dir).
VPATH = $(BIOS_DIRECTORY):$(CPU_DIRECTORY):$(HOST_DRV)/driver/source:$(HOST_DRV)/common/source:$(HOST_DRV)/socket/source:$(HOST_DRV)/spi_flash/source

%.o: %.c
	$(compile)

%.o: %.S
	$(assemble)

clean:
	$(RM) $(OBJECTS) winc_test.elf winc_test.bin winc_test.elf.map .*~ *~

.PHONY: all clean
