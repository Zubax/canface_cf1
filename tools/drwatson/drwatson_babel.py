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
    enforce, fatal, load_firmware_via_gdb, BackgroundSpinner
import drwatson.can
import logging
import time
import yaml
import glob
import re
from base64 import b64decode, b64encode
from contextlib import closing
import random
import binascii
import uavcan
import uavcan.driver


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
BUS_VOLTAGE_RANGE_OFF = 0, 2

NUM_TEST_FRAMES = 1000


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
            return uid.lower()

    fatal('Could not determine UID of the SLCAN adapter')

SLCAN_ADAPTER_UID_LOWERCASE = resolve_adapter_uid()


def get_target_serial_port_symlink():
    out = None
    for p in glob.glob('/dev/serial/by-id/*'):
        if SLCAN_ADAPTER_UID_LOWERCASE and (SLCAN_ADAPTER_UID_LOWERCASE in p.lower()):
            continue
        if ('zubax' in p.lower()) and ('babel' in p.lower()):
            enforce(not out,
                    'More than one target detected; make sure no extra hardware is attached to this computer\n'
                    '%r, %r', out, p)
            out = p

    logger.debug('Detected target serial port symlink: %r', out)
    return out


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

        time.sleep(1)

    abort('The serial port of the device did not appear in the system.'
          'This may indicate that the device could not boot or that the USB connection to it is not working properly.'
          "Please double check the cabling. If it still doesn't work, the device must be broken.")


def prepare_can_iface():
    if '/' not in args.iface:
        logger.debug('Using iface %r as SocketCAN', args.iface)
        execute_shell_command('ifconfig %s down && ip link set %s up type can bitrate %d sample-point 0.875',
                              args.iface, args.iface, CAN_BITRATE)
    else:
        logger.debug('Using iface %r as SLCAN', args.iface)
        # We don't want the SLCAN daemon to interfere...
        execute_shell_command('killall -INT slcand &> /dev/null', ignore_failure=True)
        time.sleep(1)
        # Making sure the interface can be open
        with open(args.iface, 'bw') as _f:
            pass


def check_and_prepare_interfaces():
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
        prepare_can_iface()
        info('CAN interface is OK')
    except Exception:
        logging.debug('CAN check error', exc_info=True)
        error('CAN interface is not working')
        ok = False

    if not ok:
        fatal('Required interfaces are not available. Please check your hardware configuration. '
              'If this application is running on a virtual machine, make sure that hardware '
              'sharing is configured correctly.')

check_and_prepare_interfaces()

licensing_api = make_api_context_with_user_provided_credentials()

with CLIWaitCursor():
    print('Please wait...')
    if args.firmware:
        firmware_data = download(args.firmware)
    else:
        firmware_data = download_newest(DEFAULT_FIRMWARE_GLOB)
    assert 30 < (len(firmware_data) / 1024) <= 240, 'Invalid firmware size'


def read_zubax_id(drv: drwatson.can.SLCAN):
    zubax_id_lines = drv.execute_cli_command('zubax_id')
    try:
        zubax_id = yaml.load(zubax_id_lines)
    except Exception:
        logger.info('Could not parse YAML: %r', zubax_id_lines)
        raise
    logger.info('Zubax ID: %r', zubax_id)
    return zubax_id


def make_random_can_frame():
    extended = random.choice([True, False])
    can_id = random.randrange(0, 2**(29 if extended else 11))
    data = bytes(random.randint(0, 255) for _ in range(random.randint(0, 8)))
    return uavcan.driver.CANFrame(can_id=can_id, data=data, extended=extended)


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
            try:
                load_firmware_via_gdb(firmware_data,
                                      toolchain_prefix=TOOLCHAIN_PREFIX,
                                      load_offset=FLASH_OFFSET,
                                      gdb_port=glob_one(DEBUGGER_PORT_GDB_GLOB),
                                      gdb_monitor_scan_command='swdp_scan')
            except Exception as ex:
                logging.info('Firmware load error', exc_info=True)
                fatal('Could not load firmware; check the debug connector; error: %r', ex)
    else:
        info('Firmware upload skipped')

    info('Waiting for the device to boot...')
    wait_for_boot()

    with closing(drwatson.can.SLCAN(get_target_serial_port_symlink(),
                                    bitrate=CAN_BITRATE,
                                    default_timeout=1)) as drv_target:
        info('Reading Zubax ID...')
        zubax_id = read_zubax_id(drv_target)
        unique_id = b64decode(zubax_id['hw_unique_id'])
        product_id = zubax_id['product_id']
        assert PRODUCT_NAME == product_id
        set_device_info(product_id, unique_id)

        info('Configuring the adapter...')

        drv_target.execute_cli_command('cfg set can.terminator_on 1')  # Terminator ON

        logger.info('Adapter state:\n%s', drv_target.execute_cli_command('stat'))

        with closing(uavcan.driver.make_driver(args.iface, bitrate=CAN_BITRATE)) as drv_test:
            random_frames = [make_random_can_frame() for _ in range(NUM_TEST_FRAMES)]

            info('Testing CAN bus exchange: target --> test')
            for idx, rf in enumerate(random_frames):
                drv_target.send(rf.id, rf.data, rf.extended)
                received = drv_test.receive(1)
                enforce(received is not None, 'Timeout when trying to receive frame %d', idx + 1)
                enforce(received.extended == rf.extended and
                        received.data == rf.data and
                        received.id == rf.id,
                        'Received frame %d [%r] does not match the reference [%r]', idx + 1, received, rf)

            info('Testing CAN bus exchange: test --> target')
            for idx, rf in enumerate(random_frames):
                drv_test.send(rf.id, rf.data, rf.extended)
                try:
                    received = drv_target.receive(1)
                except TimeoutError:
                    fatal('Timeout when trying to receive frame %d', idx + 1)
                enforce(received['ext'] == rf.extended and
                        received['data'] == rf.data and
                        received['id'] == rf.id,
                        'Received frame %d [%r] does not match the reference [%r]', idx + 1, received, rf)

            info('Test exchange OK (2x%d frames)', len(random_frames))

        info('Testing power supply...')

        drv_target.execute_cli_command('cfg set can.power_on 0')       # Bus power OFF
        time.sleep(2)
        stat = yaml.load(drv_target.execute_cli_command('stat'))
        enforce(BUS_VOLTAGE_RANGE_OFF[0] <= stat['bus_voltage'] <= BUS_VOLTAGE_RANGE_OFF[1],
                'Invalid voltage on the bus (power is turned OFF): %r volts; '
                'there may be a short circuit on the board', stat['bus_voltage'])
        info('Bus voltage: %r', stat['bus_voltage'])

        drv_target.execute_cli_command('cfg set can.power_on 1')       # Bus power ON
        time.sleep(2)
        stat = yaml.load(drv_target.execute_cli_command('stat'))
        enforce(BUS_VOLTAGE_RANGE_ON[0] <= stat['bus_voltage'] <= BUS_VOLTAGE_RANGE_ON[1],
                'Invalid voltage on the bus (power is turned ON): %r volts; '
                'the power supply circuit is malfunctioning', stat['bus_voltage'])
        info('Bus voltage: %r', stat['bus_voltage'])

        info('Power supply is OK')

        info('Testing LED indicators...')
        # LED1 - CAN Power      - Red
        # LED2 - Terminator ON  - Orange
        # LED3 - Status         - Blue
        # LED4 - CAN Activity   - Green
        enforce(input('Is LED1 (CAN power, RED) turned on?', yes_no=True),
                'CAN Power LED is not working')

        enforce(input('Is LED2 (terminator, ORANGE) turned on?', yes_no=True),
                'Terminator and/or its LED are not working')

        enforce(input('Is LED3 (status, BLUE) blinking about once a second?', yes_no=True),
                'Status LED is not working')

        def generate_traffic():
            drv_target.send(0, b'', False)
            time.sleep(0.2)

        with BackgroundSpinner(generate_traffic):
            enforce(input('Is LED4 (activity, GREEN) blinking quickly?', yes_no=True),
                    'Activity LED is not working, or the bus has been disconnected')

        info('Resetting configuration to factory defaults...')
        drv_target.execute_cli_command('cfg erase')

        info('Installing signature...')

        # Getting the signature
        info('Requesting signature for unique ID %s', binascii.hexlify(unique_id).decode())
        gen_sign_response = licensing_api.generate_signature(unique_id, PRODUCT_NAME)
        if gen_sign_response.new:
            info('New signature has been generated')
        else:
            info('This particular device has been signed earlier, reusing existing signature')
        base64_signature = b64encode(gen_sign_response.signature).decode()
        logger.info('Generated signature in Base64: %s', base64_signature)

        # Installing the signature; this may fail if the device has been signed earlier - the failure will be ignored
        out = drv_target.execute_cli_command('zubax_id %s' % base64_signature)
        logger.debug('Signature installation response (may fail, which is OK): %r', out)

        # Reading the signature back and verifying it
        installed_signature = read_zubax_id(drv_target)['hw_signature']
        logger.info('Installed signature in Base64: %s', installed_signature)
        enforce(b64decode(installed_signature) == gen_sign_response.signature,
                'Written signature does not match the generated signature')

        info('Signature has been installed and verified')


run(licensing_api, process_one_device)
