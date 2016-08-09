#!/usr/bin/env python3
#
# Copyright (C) 2016 Zubax Robotics <info@zubax.com>
#
# This program is free software: you can redistribute it and/or modify it under the terms of the
# GNU General Public License as published by the Free Software Foundation, either version 3 of the License,
# or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
# without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along with this program.
# If not, see <http://www.gnu.org/licenses/>.
#
# Author: Pavel Kirienko <pavel.kirienko@zubax.com>
#

import os
import sys
sys.path.insert(1, os.path.join(sys.path[0], 'pyuavcan'))

from drwatson import init, run, make_api_context_with_user_provided_credentials, execute_shell_command,\
    info, error, input, CLIWaitCursor, download, abort, glob_one, download_newest, open_serial_port,\
    enforce, SerialCLI, BackgroundSpinner, fatal, BackgroundDelay, imperative, \
    load_firmware_via_gdb, convert_units_from_to, BackgroundCLIListener
import logging
import time
import yaml
import binascii
import uavcan
import glob
import re
from base64 import b64decode, b64encode
from contextlib import closing, contextmanager
from functools import partial


PRODUCT_NAME = 'com.zubax.babel'
DEFAULT_FIRMWARE_GLOB = 'https://files.zubax.com/products/%s/*.compound.bin' % PRODUCT_NAME
CAN_BITRATE = 1000000
FLASH_OFFSET = 0x08000000
TOOLCHAIN_PREFIX = 'arm-none-eabi-'
DEBUGGER_PORT_GDB_GLOB = '/dev/serial/by-id/*Black_Magic_Probe*-if00'
DEBUGGER_PORT_CLI_GLOB = '/dev/serial/by-id/*Black_Magic_Probe*-if02'
BOOT_TIMEOUT = 10
END_OF_BOOT_LOG_TIMEOUT = 3

BUS_VOLTAGE_RANGE_ON = 4, 6
BUS_VOLTAGE_RANGE_OFF = 0, 1


logger = logging.getLogger('main')
cli_logger = logging.getLogger('cli')


args = init('''Production testing application for Zubax Babel CAN adapters.
If you're a licensed manufacturer, you should have received usage
instructions with the manufacturing doc pack.''',
            lambda p: p.add_argument('iface', help='CAN interface or device path, e.g. "can0", "/dev/ttyACM0", etc.'),
            lambda p: p.add_argument('--firmware', '-f', help='location of the firmware file (if not provided, ' +
                                     'the firmware will be downloaded from Zubax Robotics file server)'),
            require_root=True)

info('''
Usage instructions:
1. Connect a CAN adapter to this computer. Supported adapters are:
1.1. SLCAN-compliant adapters. If you're using an SLCAN adapter,
     use its serial port name as CAN interface name (e.g. "/dev/ttyACM0").
1.2. SocketCAN-compatible adapters. In this case it is recommended to use
     8devices USB2CAN. Correct interface name would be "can0".
2. Connect exactly one DroneCode Probe to this computer.
   For more info refer to https://docs.zubax.com/dronecode_probe.
3. Follow the instructions printed in green. If you have any questions,
   don't hesitate to reach licensing@zubax.com, or use the emergency
   contacts provided to you earlier.
''')


# First we need to find out if we're using another Babel as a CAN adapter
def resolve_adapter_uid():
    if '/' not in args.iface:
        return

    iface_real = os.path.normpath(os.path.realpath(args.iface))

    for p in glob.glob('/dev/serial/by-id/*'):
        p_real = os.path.normpath(os.path.realpath(p))
        if p_real == iface_real:
            try:
                uid = re.findall(r'[-_]([0-9A-Fa-f]{8,32})', p)[0]
            except IndexError:
                uid = None
            logger.debug('SLCAN adapter port: %r --> %r, UID: %r', p, p_real, uid)
            return uid

    fatal('Could not determine UID of the SLCAN adapter')

SLCAN_ADAPTER_UID_LOWERCASE = resolve_adapter_uid().lower()


def get_target_serial_port_symlink():
    for p in glob.glob('/dev/serial/by-id/*'):
        if ('zubax' in p.lower()) and ('babel' in p.lower()) and (SLCAN_ADAPTER_UID_LOWERCASE not in p.lower()):
            logger.debug('Detected target serial port symlink: %r', p)
            return p


def wait_for_boot():
    boot_deadline = time.monotonic() + BOOT_TIMEOUT

    bootloader_detected = False

    while boot_deadline > time.monotonic():
        target_serial_symlink = get_target_serial_port_symlink()
        if not target_serial_symlink:
            continue

        if 'bootloader' in target_serial_symlink.lower():
            if not bootloader_detected:
                bootloader_detected = True
                info('Bootloader port detected, waiting for application to boot...')
        else:
            info('Boot successful')
            return

    abort('The serial port of the device did not appear in the system.'
          'This may indicate that the device could not boot or that the USB connection to it is not working properly.'
          "Please double check the cabling. If it still doesn't work, the device must be broken.")


def init_can_iface():
    if '/' not in args.iface:
        logger.debug('Using iface %r as SocketCAN', args.iface)
        execute_shell_command('ifconfig %s down && ip link set %s up type can bitrate %d sample-point 0.875',
                              args.iface, args.iface, CAN_BITRATE)
        return args.iface
    else:
        logger.debug('Using iface %r as SLCAN', args.iface)

        # We don't want the SLCAN daemon to interfere...
        execute_shell_command('killall -INT slcand &> /dev/null', ignore_failure=True)
        time.sleep(1)

        # Making sure the interface can be open
        with open(args.iface, 'bw') as _f:
            pass

        return args.iface


def check_interfaces():
    ok = True

    def test_serial_port(glob, name):
        try:
            with open_serial_port(glob):
                info('%s port is OK', name)
                return True
        except Exception:
            error('%s port is not working', name)
            return False

    info('Checking interfaces...')
    ok = test_serial_port(DEBUGGER_PORT_GDB_GLOB, 'GDB') and ok
    ok = test_serial_port(DEBUGGER_PORT_CLI_GLOB, 'CLI') and ok
    try:
        init_can_iface()
        info('CAN interface is OK')
    except Exception:
        logging.debug('CAN check error', exc_info=True)
        error('CAN interface is not working')
        ok = False

    if not ok:
        fatal('Required interfaces are not available. Please check your hardware configuration. '
              'If this application is running on a virtual machine, make sure that hardware '
              'sharing is configured correctly.')

check_interfaces()

licensing_api = make_api_context_with_user_provided_credentials()

with CLIWaitCursor():
    print('Please wait...')
    if args.firmware:
        firmware_data = download(args.firmware)
    else:
        firmware_data = download_newest(DEFAULT_FIRMWARE_GLOB)
    assert 30 < (len(firmware_data) / 1024) <= 240, 'Invalid firmware size'


def process_one_device(set_device_info):
    out = input('1. Connect DroneCode Probe to the debug connector.\n'
                '2. Connect CAN to the first CAN connector on the Babel; leve the other CAN connector empty.\n'
                '3. Connect USB to the Micro USB port on the Babel.\n'
                '4. If you want to skip firmware upload, type F now.\n'
                '5. Press ENTER.')

    skip_fw_upload = 'f' in out.lower()
    if not skip_fw_upload:
        info('Loading the firmware')
        with CLIWaitCursor():
            load_firmware_via_gdb(firmware_data,
                                  toolchain_prefix=TOOLCHAIN_PREFIX,
                                  load_offset=FLASH_OFFSET,
                                  gdb_port=glob_one(DEBUGGER_PORT_GDB_GLOB),
                                  gdb_monitor_scan_command='swdp_scan')
    else:
        info('Firmware upload skipped')

    info('Waiting for the device to boot...')
    wait_for_boot()

    # TODO: Test CAN interface (enable the terminator first!)
    # TODO: Test power supply
    # TODO: Test UART (requires an external power supply via CAN)


run(licensing_api, process_one_device)
