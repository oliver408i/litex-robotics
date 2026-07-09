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
BIOS_DIRECTORY := $(SOC_DIRECTORY)/software/bios
BUILDINC_DIRECTORY := $(BUILD_DIR)/software/include

include $(SOC_DIRECTORY)/software/common.mak

COMMON_DIR := ../common
SRC_DIR    := src

# DOOM likes old-C tentative definitions (multiple globals) and is warning-noisy;
# -fcommon restores the pre-gcc10 behavior, -O2 because it's a game, and the
# resolution is fixed to DOOM's native 320x200 (no scaling in the blit).
CFLAGS += -I$(COMMON_DIR) -I$(SRC_DIR) \
          -DDOOMGENERIC_RESX=320 -DDOOMGENERIC_RESY=200 \
          -fcommon -O2 \
          -Wno-implicit-function-declaration -Wno-unused -Wno-format

# --- DOOM engine object list (doomgeneric canonical set, minus SDL/xlib/stdc) ---
DOOM_SRC = am_map doomdef doomstat dstrings d_event d_items d_iwad d_loop d_main \
           d_mode d_net f_finale f_wipe g_game hu_lib hu_stuff info i_cdmus \
           i_endoom i_joystick i_scale i_sound i_system i_timer memio m_argv \
           m_bbox m_cheat m_config m_controls m_fixed m_menu m_misc m_random \
           p_ceilng p_doors p_enemy p_floor p_inter p_lights p_map p_maputl \
           p_mobj p_plats p_pspr p_saveg p_setup p_sight p_spec p_switch \
           p_telept p_tick p_user r_bsp r_data r_draw r_main r_plane r_segs \
           r_sky r_things sha1 sounds statdump st_lib st_stuff s_sound tables \
           v_video wi_stuff w_checksum w_file w_main w_wad z_zone i_input \
           i_video doomgeneric dummy \
           doomgeneric_icepi w_file_icepi syscalls

DOOM_OBJS = $(addsuffix .o,$(DOOM_SRC))

# Shared firmware modules: LCD engine, FT6336U touch, UART log.
COMMON_OBJS = lcd.o touch.o log.o

# On an expander (IOX) build, lcd.c drives LCD/CTP reset over the aux SPI bus,
# so pull in the MCP23S17 + aux_spi drivers. Detect from the generated CSRs.
AUX_OBJS =
HAS_IOX := $(shell grep -q IOX_RESET_BASE $(BUILDINC_DIRECTORY)/generated/csr.h 2>/dev/null && echo 1)
ifeq ($(HAS_IOX),1)
COMMON_OBJS += mcp23s17.o
WINC_DIR := ../winc_test
CFLAGS   += -I$(WINC_DIR)
AUX_OBJS  = aux_spi.o
endif

OBJECTS = crt0.o $(DOOM_OBJS) $(COMMON_OBJS) $(AUX_OBJS)

all: doom.bin

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@
ifneq ($(OS),Windows_NT)
	chmod -x $@
endif

vpath %.a $(PACKAGES:%=$(BUILD_DIR)/software/%)

doom.elf: $(OBJECTS)
	$(CC) $(LDFLAGS) -T linker.ld -N -o $@ \
		$(OBJECTS) \
		$(PACKAGES:%=-L$(BUILD_DIR)/software/%) \
		-Wl,--whole-archive \
		-Wl,--gc-sections \
		-Wl,-Map,$@.map \
		$(LIBS:lib%=-l%) \
		-Wl,--no-whole-archive \
		-lgcc
ifneq ($(OS),Windows_NT)
	chmod -x $@
endif

-include $(OBJECTS:.o=.d)

VPATH = $(SRC_DIR):$(COMMON_DIR):$(BIOS_DIRECTORY):$(CPU_DIRECTORY)

%.o: %.c
	$(compile)

ifeq ($(HAS_IOX),1)
aux_spi.o: CFLAGS += -include stdio.h
aux_spi.o: $(WINC_DIR)/aux_spi.c
	$(compile)
endif

%.o: %.S
	$(assemble)

clean:
	$(RM) $(OBJECTS) $(OBJECTS:.o=.d) doom.elf doom.bin doom.elf.map .*~ *~

.PHONY: all clean
