# Production firmware for Ubuntu / WSL; test/gcc remains the Renode build.
.DEFAULT_GOAL := all
.DELETE_ON_ERROR:

CHIP ?= F401C
CONFIG ?= Release
TOOLCHAIN_PREFIX ?= arm-none-eabi-
PYTHON ?= python3
EXTRA ?=

ifeq ($(CHIP),F401C)
DEVICE := STM32F401xC
STARTUP := gcc/startup_stm32f401xc.s
FLASH_SIZE := 256K
RAM_SIZE := 64K
else ifeq ($(CHIP),F401E)
DEVICE := STM32F401xE
STARTUP := gcc/startup_stm32f401xc.s
FLASH_SIZE := 512K
RAM_SIZE := 96K
else ifeq ($(CHIP),F411E)
DEVICE := STM32F411xE
STARTUP := gcc/startup_stm32f411xe.s
FLASH_SIZE := 512K
RAM_SIZE := 128K
else
$(error Unsupported CHIP '$(CHIP)'; use F401C, F401E or F411E)
endif

ifeq ($(CONFIG),Release)
OPT := -Os
OPT_FAST := -O2
CONFIG_DEFS := -DNDEBUG
else ifeq ($(CONFIG),Debug)
OPT := -Og
OPT_FAST := -Og
else
$(error Unsupported CONFIG '$(CONFIG)'; use Release or Debug)
endif

CC := $(TOOLCHAIN_PREFIX)gcc
OBJCOPY := $(TOOLCHAIN_PREFIX)objcopy
SIZE := $(TOOLCHAIN_PREFIX)size
BUILD := build/gcc/$(CHIP)/$(CONFIG)
TARGET := $(BUILD)/teacup
# Explicit production C list shared by the two IAR projects (including app/main).
include gcc/sources.mk
OBJECTS := $(addprefix $(BUILD)/,$(SOURCES:.c=.o)) $(BUILD)/gcc/startup.o
FAST_SRC := dda.c dda_queue.c dda_maths.c dda_lookahead.c dda_kinematics.c \
            linear_advance.c input_shaping.c timer.c serial.c uart.c usb_cdc.c \
            endstops.c clock.c
CPUFLAGS := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CPPFLAGS := -D$(DEVICE) $(CONFIG_DEFS) $(EXTRA) \
            $(addprefix -I, . cmsis/core cmsis/device config hal core display motion pff src)
# IAR projects select signed plain char. Keep debug symbols in both variants.
CFLAGS := $(CPUFLAGS) -std=c11 -fsigned-char $(OPT) -g3 \
          -ffunction-sections -fdata-sections -Wall -Wextra -Wno-unused-parameter \
          -Wno-missing-field-initializers -Werror=implicit-function-declaration -MMD -MP
LDFLAGS := $(CPUFLAGS) -T gcc/stm32f4.ld \
           -Wl,--defsym=__flash_size=$(FLASH_SIZE),--defsym=__ram_size=$(RAM_SIZE) \
           -Wl,--gc-sections,--print-memory-usage,-Map=$(TARGET).map \
           --specs=nano.specs --specs=nosys.specs

.PHONY: all check size all-chips all-variants clean help FORCE
all: $(TARGET).elf $(TARGET).hex $(TARGET).bin check size

# Detect changed options/toolchain without forcing unchanged incremental builds.
$(BUILD)/build-options.txt: FORCE
	@mkdir -p $(BUILD)
	@{ printf '%s\n' '$(CC)' '$(CPPFLAGS)' '$(CFLAGS)' '$(OPT_FAST)' '$(LDFLAGS)' '$(SOURCES)' '$(STARTUP)'; $(CC) --version; } > $@.tmp
	@cmp -s $@.tmp $@ && rm -f $@.tmp || mv -f $@.tmp $@

$(BUILD)/%.o: %.c $(BUILD)/build-options.txt Makefile gcc/sources.mk
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(if $(filter $(notdir $<),$(FAST_SRC)),$(OPT_FAST)) -c $< -o $@

$(BUILD)/gcc/startup.o: $(STARTUP) $(BUILD)/build-options.txt Makefile
	@mkdir -p $(dir $@)
	$(CC) $(CPUFLAGS) -g3 -c $< -o $@

$(TARGET).elf: $(OBJECTS) gcc/stm32f4.ld Makefile
	$(CC) $(LDFLAGS) $(OBJECTS) -lm -o $@

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) --gap-fill 0xff -O binary $< $@

check: $(TARGET).elf $(TARGET).hex $(TARGET).bin
	$(PYTHON) gcc/check_project.py
	$(PYTHON) gcc/check_firmware.py --chip $(CHIP) --elf $(TARGET).elf --hex $(TARGET).hex --bin $(TARGET).bin

size: $(TARGET).elf
	$(SIZE) $<

all-chips:
	@set -e; for chip in F401C F401E F411E; do $(MAKE) CHIP=$$chip CONFIG=$(CONFIG) all; done

all-variants:
	@set -e; for config in Release Debug; do $(MAKE) CONFIG=$$config all-chips; done

clean:
	rm -rf build/gcc

help:
	@printf '%s\n' 'make [-j] CHIP=F401C|F401E|F411E CONFIG=Release|Debug' \
	  'make all-variants  Build and validate all six production variants' \
	  'make clean         Remove production GCC outputs only' \
	  'Outputs: build/gcc/<CHIP>/<CONFIG>/teacup.{elf,hex,bin,map}' \
	  'Uses config/config.h (P3Steel); never defines TEACUP_RENODE_TEST.'

-include $(OBJECTS:.o=.d)
