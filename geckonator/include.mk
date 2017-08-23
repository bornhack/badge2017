# This file is part of geckonator.
# Copyright 2017 Emil Renner Berthing <esmil@esmil.dk>
#
# geckonator is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# geckonator is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with geckonator. If not, see <http://www.gnu.org/licenses/>.

# Try uncommenting this if the build fails
#OLD = 1

NAME       = code
OUTDIR     = out
DESTDIR    = .
CHIP       = EFM32HG309F64
DFU_DEVICE = -d ,0483:df11

ifeq ($(patsubst %F64,,$(CHIP)),)
FLASH      = 64K
else ifeq ($(patsubst %F32,,$(CHIP)),)
FLASH      = 32K
else
$(error Unknown chip $(CHIP))
endif
BOOTLOADER = 4K
STACK      = 1024

MAKEFLAGS  = -rR
TOPDIR    := $(dir $(lastword $(MAKEFILE_LIST)))
VPATH      = .:$(TOPDIR)
OS        ?= $(shell uname -s)

CROSS_COMPILE = arm-none-eabi-

CC         = $(CROSS_COMPILE)gcc -std=c99
AS         = $(CROSS_COMPILE)gcc
OBJCOPY    = $(CROSS_COMPILE)objcopy
OBJDUMP    = $(CROSS_COMPILE)objdump
HEX        = $(OBJCOPY) -O ihex
BIN        = $(OBJCOPY) -O binary -S
DIS        = $(OBJDUMP) -d -M force-thumb
OPENOCD    = openocd
DFU_UTIL   = dfu-util
INSTALL    = install
SED        = sed
DOS2UNIX   = $(SED) 's/\x0D$$//'
PAGER     ?= less

ifeq ($(OS:Win%=),)
COMMENT    = rem
MKDIR_P    = mkdir
RM_RF      = rmdir /q /s
FILESIZE   = rem
else
COMMENT    = \#
MKDIR_P    = mkdir -p
RM_RF      = rm -rf
ifeq ($(OS),Linux)
FILESIZE   = stat --printf '    %s bytes\n'
else
FILESIZE   = stat -f '    %z bytes'
endif
endif

GECKO_SDK  = ../Gecko_SDK

OPT        = -Os
ARCHFLAGS  = -mthumb -mcpu=cortex-m0plus
CFLAGS     = $(ARCHFLAGS) $(OPT) -ggdb -pipe -Wall -Wextra -Wno-main -Wno-unused-parameter -fdata-sections -ffunction-sections -fstack-usage
ASFLAGS    = $(ARCHFLAGS)
CPPFLAGS   = -iquote '$(TOPDIR)SiliconLabs' -iquote '$(TOPDIR:%/=%)' -D$(CHIP) -D__STACK_SIZE=$(STACK)
LIBS       = -lc -lm -lnosys
LDFLAGS    = $(ARCHFLAGS) -nostartfiles -specs=nano.specs -Wl,-O1,--gc-sections -L '$(TOPDIR)lib' -T $(LDSCRIPT)

ifndef OLD
ARCHFLAGS += -masm-syntax-unified
CPPFLAGS  += -DGPIO_TYPE_STRUCT
LDFLAGS   += -Wl,--defsym,__flash_size=$(FLASH),--defsym,__bootloader_size=$(BOOTLOADER)
LDSCRIPT   = dynamic.ld
else
LDFLAGS   += -Wl,--no-wchar-size-warning
LDSCRIPT   = static$(FLASH)-$(BOOTLOADER).ld
endif

ifdef V
E=@$(COMMENT)
Q=
else
E=@echo
Q=@
endif

sources = init.S main.c
objects = $(patsubst %,$(OUTDIR)/%.o,$(basename $(sources)))

.PHONY: all release dis clean install flash dfu sdk

all: $(OUTDIR)/$(NAME).bin

release: CPPFLAGS += -DNDEBUG
release: $(OUTDIR)/$(NAME).bin

$(OUTDIR)/%.o: %.S $(MAKEFILE_LIST) | $(OUTDIR)
	$E '  AS      $@'
	$Q$(AS) -o $@ -c $(ASFLAGS) $(CPPFLAGS) $<

$(OUTDIR)/%.o: %.c $(MAKEFILE_LIST) | $(OUTDIR)
	$E '  CC      $@'
	$Q$(CC) -o $@ -c $(CFLAGS) $(CPPFLAGS) $<

$(OUTDIR)/$(NAME).elf: $(objects) $(MAKEFILE_LIST)
	$E '  LD      $@'
	$Q$(CC) -o $@ $(LDFLAGS) $(objects) $(LIBS)

$(OUTDIR)/%.hex: $(OUTDIR)/%.elf $(MAKEFILE_LIST)
	$E '  OBJCOPY $@'
	$Q$(HEX) $< $@

$(OUTDIR)/%.bin: $(OUTDIR)/%.elf $(MAKEFILE_LIST)
	$E '  OBJCOPY $@'
	$Q$(BIN) $< $@
	-$Q$(FILESIZE) $@ 2>/dev/null

$(OUTDIR)/%.lss: $(OUTDIR)/%.elf $(MAKEFILE_LIST)
	$E '  OBJDUMP $@'
	$Q$(DIS) $< > $@

$(OUTDIR):
	$E '  MKDIR   $@'
	$Q$(MKDIR_P) $@

$(DESTDIR)/$(NAME).bin: $(OUTDIR)/$(NAME).bin | $(DESTDIR)
	$E '  INSTALL $@'
	$Q$(INSTALL) -m644 '$<' '$@'

$(DESTDIR):
	$E '  INSTALL $@'
	$Q$(INSTALL) -dm755 '$(DESTDIR)'

dis: $(OUTDIR)/$(NAME).lss
	$(PAGER) $<

clean:
	$E '  RM      $(OUTDIR)'
	$Q$(RM_RF) $(OUTDIR)

install: $(DESTDIR)/$(NAME).bin

flash: $(OUTDIR)/$(NAME).elf
	$(OPENOCD) -c 'set CPUTAPID 0x0bc11477' -f interface/stlink-v2.cfg -f target/efm32.cfg -c 'init; program $< verify reset exit;'

dfu: $(OUTDIR)/$(NAME).bin
	$(DFU_UTIL) $(DFU_DEVICE) -R -D '$<'

sdk:
	$(MKDIR_P) SiliconLabs
	for i in '$(GECKO_SDK)/platform/Device/SiliconLabs/EFM32HG/Include'/*.h; do \
	  file="$${i##*/}"; \
	  [ "$$file" = 'system_efm32hg.h' ] || $(DOS2UNIX) "$$i" > "SiliconLabs/$$file"; \
	done
	$(SED) -i '/^#include.*"system_efm32hg.h"/d' SiliconLabs/*.h
	for i in arm_math.h cmsis_gcc.h core_cm0plus.h core_cmFunc.h core_cmInstr.h; do \
	  $(DOS2UNIX) "$(GECKO_SDK)/platform/CMSIS/Include/$$i" > "SiliconLabs/$$i"; \
	done
