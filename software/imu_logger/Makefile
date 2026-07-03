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

# Pull in every C file under lvgl/src/ except HW drivers (we glue our own) and
# the Vivante GPU bindings (headers not vendored).
LVGL_SRCS := $(shell find $(LVGL_DIR)/src -name '*.c' \
              -not -path '*/drivers/*' \
              -not -path '*/vg_lite_tvg/*' \
              -not -path '*/vg_lite_driver/*')
LVGL_OBJS := $(patsubst $(LVGL_DIR)/src/%.c,$(LVGL_BUILD)/%.o,$(LVGL_SRCS))

CFLAGS += -I$(LVGL_DIR) -I$(LVGL_DIR)/src -I. -DLV_CONF_INCLUDE_SIMPLE

# Shared firmware modules: LCD bring-up, HW I2C / FT6336U, UART log, aux ADC
# (MCP3008 power), LSM6DS3 IMU + FatFs logger, NMEA GPS reader.
COMMON_DIR := ../common
CFLAGS    += -I$(COMMON_DIR)
COMMON_OBJS = lcd.o touch.o log.o adc.o power.o lsm6ds3.o imu_log.o gps_nmea.o

# FatFs (libfatfs) + SD card driver (liblitesdcard) header dirs. The libraries
# link automatically via PACKAGES once the SoC is built with the SD core.
CFLAGS += -I$(LIBFATFS_DIRECTORY) -I$(LIBLITESDCARD_DIRECTORY)

# Shared aux SPI bus HAL lives in winc_test/ (reused for the MCP3008 power ADC +
# LSM6DS3 IMU). Pull in its header dir; aux_spi.o is built from source below.
WINC_DIR := ../winc_test
CFLAGS   += -I$(WINC_DIR)
AUX_OBJS  = aux_spi.o

# LVGL doesn't satisfy LiteX's stricter warning set; relax those without
# silencing real bugs.
CFLAGS += -Wno-strict-prototypes -Wno-old-style-definition -Wno-missing-prototypes
# Help the linker drop the large slice of LVGL we don't reference.
CFLAGS += -ffunction-sections -fdata-sections

# Touch bring-up diagnostic: log what the FT6336U reports to the UART console.
# Off by default. Enable with: make TOUCH_DEBUG=1
ifdef TOUCH_DEBUG
CFLAGS += -DTOUCH_DEBUG
endif

# IMU/SD logger bring-up breadcrumbs (localizes a blocking SD init). make IMU_LOG_DEBUG=1
ifdef IMU_LOG_DEBUG
CFLAGS += -DIMU_LOG_DEBUG
endif

# GPS: log failed-checksum sentences (corruption vs logic). make GPS_DEBUG=1
ifdef GPS_DEBUG
CFLAGS += -DGPS_DEBUG
endif

OBJECTS = main.o crt0.o $(COMMON_OBJS) $(AUX_OBJS)

all: imu_logger.bin

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@
ifneq ($(OS),Windows_NT)
	chmod -x $@
endif

vpath %.a $(PACKAGES:%=$(BUILD_DIR)/software/%)

imu_logger.elf: $(OBJECTS) $(LVGL_OBJS)
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
# don't put winc_test on VPATH, or make would pick up its prebuilt aux_spi.o
# (compiled against a different SoC's csr.h). aux_spi.c calls printf() on its
# timeout path but leaves the declaration to its includer; force <stdio.h> for
# just this object so the shared source stays untouched.
aux_spi.o: CFLAGS += -include stdio.h
aux_spi.o: $(WINC_DIR)/aux_spi.c
	$(compile)

%.o: %.S
	$(assemble)

$(LVGL_BUILD)/%.o: $(LVGL_DIR)/src/%.c
	@mkdir -p $(@D)
	$(CC) -c $(CFLAGS) -o $@ $<

clean:
	$(RM) $(OBJECTS) imu_logger.elf imu_logger.bin imu_logger.elf.map .*~ *~
	$(RM) -r $(LVGL_BUILD)

.PHONY: all clean
