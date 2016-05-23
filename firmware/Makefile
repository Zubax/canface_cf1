#
# Copyright (C) 2015  Zubax Robotics  <info@zubax.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
# Author: Pavel Kirienko <pavel.kirienko@zubax.com>
#

PROJECT = com.zubax.babel

HW_VERSION = 1
FW_VERSION_MAJOR = 1
FW_VERSION_MINOR = 0

#
# Application
#

CPPSRC = $(shell find src -name '*.cpp')

UINCDIR = src                \
          src/sys

UDEFS = -DFW_VERSION_MAJOR=$(FW_VERSION_MAJOR)           \
        -DFW_VERSION_MINOR=$(FW_VERSION_MINOR)           \
        -DHW_VERSION=$(HW_VERSION)                       \
        -DPRODUCT_ID_STRING=\"$(PROJECT)\"               \
        -DPRODUCT_NAME_STRING=\"Zubax\ Babel\"

#
# Git commit hash
#

GIT_HASH := $(shell git rev-parse --short HEAD)
UDEFS += -DGIT_HASH=0x$(GIT_HASH)

#
# ChibiOS
#

BUILD_CONFIG = 1
BUILD_CHIBIOS_SHELL = 1

BOOTLOADER_SIZE = 32768

DDEFS += -DCORTEX_VTOR_INIT=$(BOOTLOADER_SIZE)           \
         -DCRT1_AREAS_NUMBER=0

UDEFS += -DCONFIG_PARAMS_MAX=10

USE_LTO := yes

SERIAL_CLI_PORT_NUMBER = 1

RELEASE ?= 0
RELEASE_OPT = -O3 -fomit-frame-pointer # -g3    # -g3 is needed for profiling
DEBUG_OPT = -O2 -g3 -DDISABLE_WATCHDOG=1

LDSCRIPT = ld.ld

CWARN += -Wshadow -Wpointer-arith -Wno-packed -Wno-attributes -Wno-error=undef -Wno-error=shadow
CPPWARN += $(CWARN)

#
# Bootloader integration
#

HW_VERSION_MAJOR_MINOR := $(HW_VERSION).0
FW_VERSION_MAJOR_MINOR_VCS_HASH := $(FW_VERSION_MAJOR).$(FW_VERSION_MINOR).$(GIT_HASH)
COMPOUND_IMAGE_FILE := $(PROJECT)-$(HW_VERSION_MAJOR_MINOR)-$(FW_VERSION_MAJOR_MINOR_VCS_HASH).compound.bin
BOOTLOADER_DIR := $(abspath ../bootloader)
BOOTLOADER_IMAGE := $(BOOTLOADER_DIR)/build/com.zubax.babel.bin

# This rule hook is defined in the ChibiOS build system
POST_MAKE_ALL_RULE_HOOK: build/$(PROJECT).bin build/$(PROJECT).elf
	# Building the bootloader
	+cd $(BOOTLOADER_DIR) && make RELEASE=$(RELEASE)
	
	# Removing previous build outputs that could use a different git hash
	rm -rf build/*.application.bin build/*.compound.bin
	
	# Generating compound image with embedded bootloader
	cd build && dd if=/dev/zero bs=$(BOOTLOADER_SIZE) count=1 | tr "\000" "\377" >padded_bootloader.tmp.bin
	cd build && dd if=$(BOOTLOADER_IMAGE) of=padded_bootloader.tmp.bin conv=notrunc
	cd build && cat padded_bootloader.tmp.bin $(PROJECT).bin >$(COMPOUND_IMAGE_FILE)
	
	# Generating the signed image for the bootloader
	cd build && ../zubax_chibios/tools/make_boot_descriptor.py $(PROJECT).bin $(PROJECT) $(HW_VERSION_MAJOR_MINOR) \
	                                                           --also-patch-descriptor-in=$(PROJECT).elf           \
	                                                           --also-patch-descriptor-in=$(COMPOUND_IMAGE_FILE)
	
	# Injecting the bootloader into the final ELF; note that we're using unpadded bootloader to preserve signature
	cd build && $(TOOLCHAIN_PREFIX)-objcopy --add-section bootloader=$(BOOTLOADER_IMAGE)   \
	                                        --set-section-flags bootloader=load,alloc      \
	                                        --change-section-address bootloader=0x08000000 \
	                                        $(PROJECT).elf compound.elf
	
	# Removing temporary files
	cd build && rm -f $(PROJECT).bin $(PROJECT).elf *.hex *.tmp.bin

include zubax_chibios/rules_stm32f37x.mk
