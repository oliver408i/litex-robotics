BUILD_DIR?=../../build/sipeed_tang_nano_20k
LITEX_ROOT?=../../litex-setup
TRIPLE?=riscv64-linux-gnu

include $(BUILD_DIR)/software/include/generated/variables.mak

# Override generated absolute paths with workspace-relative ones.
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

OBJECTS = main.o crt0.o

all: blinky.bin

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@
ifneq ($(OS),Windows_NT)
	chmod -x $@
endif

vpath %.a $(PACKAGES:%=../%)

blinky.elf: $(OBJECTS)
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

VPATH = $(BIOS_DIRECTORY):$(CPU_DIRECTORY)

%.o: %.c
	$(compile)

%.o: %.S
	$(assemble)

clean:
	$(RM) $(OBJECTS) blinky.elf blinky.bin .*~ *~

.PHONY: all clean
