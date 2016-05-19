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
BL_VERSION_MAJOR = 1
BL_VERSION_MINOR = 0

APPLICATION_OFFSET = 32768  # TODO: Will be reduced to 24k when the bootloader is finished

#
# Application
#

CPPSRC = $(shell find src -name '*.cpp')

UINCDIR = src                \
          src/sys

UDEFS = -DBL_VERSION_MAJOR=$(BL_VERSION_MAJOR)           \
        -DBL_VERSION_MINOR=$(BL_VERSION_MINOR)           \
        -DHW_VERSION=$(HW_VERSION)                       \
        -DAPPLICATION_OFFSET=$(APPLICATION_OFFSET)       \
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

USE_LTO := yes

UDEFS += -DWATCHDOG_RETAIN_RESET_CAUSE_CODE=1

SERIAL_CLI_PORT_NUMBER = 1

RELEASE_OPT = -Os -fomit-frame-pointer
DEBUG_OPT = -Os -g3 -DDISABLE_WATCHDOG=1

LDSCRIPT = ld.ld

CWARN += -Wshadow -Wpointer-arith -Wno-packed -Wno-attributes -Wno-error=undef -Wno-error=shadow
CPPWARN += $(CWARN)

include zubax_chibios/rules_stm32f37x.mk
