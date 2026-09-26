# Cross-check build with arm-none-eabi-gcc. Not needed for IAR, it only
# verifies that the sources are clean C and builds the image for the
# Renode tests.
#   make CHIP=F401                  default board/printer config
#   make CHIP=F411
#   make CHIP=F401 TEST=1           Renode test configuration
#   make CHIP=F401 EXTRA="-DFOO"    additional defines

CHIP  ?= F401
TEST  ?= 0
# Optimization, mirroring the recommended IAR setup (README, "IAR:
# optimization"): OPT for everything, OPT_FAST for the time critical files
# (step path, interrupts, move planning).
OPT      ?= -Os
OPT_FAST ?= -O2
FAST_SRC := dda.c dda_queue.c dda_maths.c dda_lookahead.c dda_kinematics.c \
            timer.c serial.c uart.c usb_cdc.c endstops.c clock.c
# LTO=-flto: cross-file inlining, like IAR's multi-file compilation.
LTO      ?=
EXTRA ?=
ROOT  := ../..
BUILD := build_$(CHIP)_$(TEST)$(BUILD_SUFFIX)

CC      := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
SIZE    := arm-none-eabi-size

ifeq ($(CHIP),F411)
  DEV   := STM32F411xE
  START := startup_stm32f411xe.s
  MEM   := -Wl,--defsym=__flash_size=512K -Wl,--defsym=__ram_size=128K
else
  DEV   := STM32F401xC
  START := startup_stm32f401xc.s
  MEM   := -Wl,--defsym=__flash_size=256K -Wl,--defsym=__ram_size=64K
endif

DEFS := -D$(DEV) $(EXTRA)
ifeq ($(TEST),1)
  DEFS += -DTEACUP_RENODE_TEST
endif

INC := $(addprefix -I$(ROOT)/,cmsis/core cmsis/device config hal core display motion pff src)
SRC := $(wildcard $(ROOT)/*.c $(ROOT)/cmsis/*.c $(ROOT)/hal/*.c $(ROOT)/core/*.c \
                  $(ROOT)/display/*.c $(ROOT)/motion/*.c $(ROOT)/pff/*.c $(ROOT)/src/*.c)
OBJ := $(addprefix $(BUILD)/,$(notdir $(SRC:.c=.o))) $(BUILD)/startup.o

CFLAGS := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
          -std=c11 $(OPT) $(LTO) -g -ffunction-sections -fdata-sections \
          -Wall -Wextra -Wno-unused-parameter -Wstrict-prototypes \
          -Wmissing-prototypes -Werror=implicit-function-declaration \
          -Wno-missing-field-initializers -MMD -MP $(DEFS) $(INC)

vpath %.c $(sort $(dir $(SRC)))

all: $(BUILD)/teacup.elf
	@$(SIZE) $<

$(BUILD)/%.o: %.c | $(BUILD)
	$(CC) $(CFLAGS) $(if $(filter $(notdir $<),$(FAST_SRC)),$(OPT_FAST)) -c $< -o $@

$(BUILD)/startup.o: $(START) | $(BUILD)
	$(CC) -mcpu=cortex-m4 -mthumb -c $< -o $@

$(BUILD)/teacup.elf: $(OBJ)
	$(CC) -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard $(LTO) $(OPT) \
	  -T stm32f4.ld $(MEM) -Wl,--gc-sections -specs=nano.specs -specs=nosys.specs \
	  -Wl,-Map=$(BUILD)/teacup.map $^ -lm -o $@

$(BUILD):
	mkdir -p $@

clean:
	rm -rf build_*

.PHONY: all clean

-include $(OBJ:.o=.d)
