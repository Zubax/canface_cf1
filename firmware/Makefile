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

PROJECT = com.zubax.can_adapter

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
        -DPRODUCT_NAME_STRING=\"Zubax\ CAN\ Adapter\"

#
# Git commit hash
#

GIT_HASH := $(shell git rev-parse --short HEAD)
UDEFS += -DGIT_HASH=0x$(GIT_HASH)

#
# ChibiOS
#

#BOOTLOADER_SIZE = 4096

#DDEFS += -DCORTEX_VTOR_INIT=$(BOOTLOADER_SIZE)

UDEFS += -DCONFIG_PARAMS_MAX=10

RELEASE ?= 0
ifeq ($(RELEASE),0)
    UDEFS += -DDISABLE_WATCHDOG=1
endif

USE_LTO := yes

SERIAL_CLI_PORT_NUMBER = 1

RELEASE_OPT = -O2 -fomit-frame-pointer
DEBUG_OPT = -Os -g3

LDSCRIPT = ld.ld

include zubax_chibios/rules_stm32f042.mk
