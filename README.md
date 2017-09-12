# [Zubax Babel](https://zubax.com/products/babel)

[![Join the chat at https://gitter.im/Zubax/general](https://img.shields.io/badge/GITTER-join%20chat-green.svg)](https://gitter.im/Zubax/general)

Zubax Babel is a high performance USB-CAN and UART-CAN adapter that can be used as a
standalone device or as an embeddable module for original equipment manufacturers (OEM).

Babel implements the quasi-standard SLCAN protocol (aka LAWICEL protocol) that is understood by
majority of CAN software products, including the Linux SocketCAN framework.
If Babel is used with [UAVCAN](http://uavcan.org) networks,
we recommend to use the [UAVCAN GUI Tool](http://uavcan.org/GUI_Tool),
which fully supports all of the advanced features available in Zubax Babel.

[**ZUBAX BABEL HOMEPAGE**](https://zubax.com/products/babel).

## Features

* Standard SLCAN (aka LAWICEL) protocol.
* CAN 2.0 A/B 10 kbps to 1 Mbps
([Dronecode/UAVCAN standard CAN connectors](http://uavcan.org/Specification/8._Hardware_design_recommendations)).
* USB 2.0 full speed (CDC ACM) (Micro USB type B).
* TTL UART (5V tolerant) 2400 to 3000000 baud
([DroneCode standard connector](https://wiki.dronecode.org/workgroup/connectors/start#dcd-mini)).
* Can be used as an OEM module or as a development board.
* Embedded bootloader supporting the standard XMODEM/YMODEM protocols over USB and UART.
* Embedded 120&#8486; CAN termination resistor that can be enabled and disabled by a command.
* Optional 5 V / 400 mA bus power supply that can be enabled and disabled by a command.
* CAN bus voltage measurement.
* Can be powered from USB, UART, CAN, or via the SMD pads.

## Relevant Information

An article that describes a passive data link delay compensation algorithm -
relevant for the CAN frame timestamp recovery problem on the host side:
[A Passive Solution to the Sensor Synchronization Problem, by Edwin Olson](https://april.eecs.umich.edu/pdfs/olson2010.pdf).
This algoritm is employed in the SLCAN backend in [PyUAVCAN library](http://uavcan.org/Implementations/Pyuavcan).

## Firmware

If you're not running Linux or OSX natively, you can download
[Bistromathic - a Linux virtual machine pre-configured for embedded development](https://files.zubax.com/vm/bistromathic.ova).

### Change Log

#### v1.1

* CAN terminator is turned ON by default.
* CAN power is turned ON by default.
* Configuration parameters are saved automatically immediately after modification.
It is no longer necessary to save configuration manually.

#### v1.0

Initial release.

### Building

Install the ARM GCC toolchain version 6.3 or newer.
Clone this repository, init all submodules (`git submodule update --init --recursive`),
then execute the following from the repository root:

```bash
cd firmware
make -j8 RELEASE=1   # Omit RELEASE=1 to build the debug version
```

Invoking the make tool from the firmware directory will also build the bootloader.
The option `RELEASE` defaults to 0 (off); when set to a non-zero value,
it will build the firmware in the release configuration rather than the debug configuration.
The debug configuration adds a bunch of runtime checks, which make things slower,
so it should be used only for development purposes.

When the firmware is built, the `build` directory will contain the following files:

* `com.zubax.*.application.bin` - the application binary suitable for loading via the bootloader,
with correct firmware descriptor and CRC.
* `com.zubax.*.compound.bin` - the above image combined with the bootloader; can be flashed on an empty MCU.
* `compound.elf` - ELF file with embedded bootloader and the correct image CRC; can be used for symbol-level debugging.
Since this ELF includes the bootloader and has a correct firmware descriptor,
it can be flashed and executed directly with an SWD debugger, no extra steps required.

### Loading

#### Via the Debug Port

Use the [Zubax Dronecode Probe](https://kb.zubax.com/x/iIAh) or any other JTAG/SWD debugger.
This helper script should do everything automatically (execute from the firmware directory):

```bash
./zubax_chibios/tools/blackmagic_flash.sh
```

#### Via the USB/UART Bootloader

The bootloader selects between USB and UART (DroneCode debug port) automatically:
if USB is connected, it will be used, and the UART port will be ignored;
if USB is not connected, UART will be used instead.
The UART port in the bootloader operates at 115200-8N1.

1. Connect to the device's CLI via the USB virtual serial port or via UART (if USB is not connected),
and execute the command `bootloader` to enter the bootloader. The device will restart.
2. Immediately after rebooting the device will enter the bootloader and stay there.
Note that if the bootloader can't find a correct firmware image in the memory,
it will never boot the firmware, so in this case the first step should be skipped.
You can always detect if the device is in the bootloader or in the application by executing `zubax_id`:
if it's running the bootloader, there will be a field `mode` set to the value `bootloader`.
Also the status LED will be glowing solid rather than blinking or being turned off.
3. In the bootloader's CLI execute `download`. This will start the X/YMODEM receiver.
Transmit the firmware image into the serial port using either YMODEM, XMODEM, or XMODEM-1K.
For example:

```bash
sz -vv --ymodem --1k $file > $port < $port
```

The steps above can be automated with the script `firmware/zubax_chibios/tools/flash_via_serial_bootloader.sh`.

### Hardware Timer Usage

* TIM2 (32-bit) - System tick timer (ChibiOS default, see `STM32_ST_TIM`)
* TIM5 (32-bit) - CAN timestamping

## License

The firmware is licensed under the terms of GNU GPL v3.

> Copyright (C) 2015 Zubax Robotics info@zubax.com
>
> This program is free software: you can redistribute it and/or modify it under the terms of the
> GNU General Public License as published by the Free Software Foundation, either version 3 of the License,
> or (at your option) any later version.
>
> This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
> without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
> See the GNU General Public License for more details.
>
> You should have received a copy of the GNU General Public License along with this program.
> If not, see http://www.gnu.org/licenses/.
