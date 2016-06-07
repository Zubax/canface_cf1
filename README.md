# Zubax Babel

Zubax Babel is a high performance USB-CAN and UART-CAN adapter that can be used as a standalone device or as an embeddable
module for original equipment manufacturers (OEM).
Zubax Babel implements the quasi-standard SLCAN protocol that is understood by most CAN software products,
including the Linux SocketCAN framework.
If Zubax Babel is used with [UAVCAN](http://uavcan.org),
we recommend to use the [UAVCAN GUI Tool](https://github.com/UAVCAN/gui_tool),
which fully supports all of the advanced features available in Zubax Babel.

## Features

* Standard SLCAN (aka LAWICEL) protocol
* CAN 2.0 A/B 10kbps to 1Mbps
([DroneCode/UAVCAN standard CAN connectors](http://uavcan.org/Specification/8._Hardware_design_recommendations))
* USB 2.0 full speed (CDC ACM) (Micro USB type B)
* TTL UART (5V tolerant) 2400 to 3000000 baud/sec
([DroneCode standard connector](https://wiki.dronecode.org/workgroup/connectors/start#dcd-mini))
* Embedded 120 Ohm CAN termination resistor that can be enabled and disabled programmatically
* Optional 5 V / 400 mA bus power supply that can be enabled and disabled programmatically
* Bus voltage measurement
* Can be used as an OEM module or as a demoboard
* Embedded bootloader supporting standard XMODEM/YMODEM over USB and UART

**This is a work-in-progress repo.**

TODO: Add links to docs and product page.

TODO: Add building and flashing instructions.

## Firmware

### Hardware timer usage

* TIM2 (32-bit) - System tick timer (ChibiOS default, see `STM32_ST_TIM`)
* TIM5 (32-bit) - CAN timestamping

## Relevant information

Article on passive delay compensation algorithm - relevant for CAN frame timestamp recovery on the host side:
[A Passive Solution to the Sensor Synchronization Problem, by Edwin Olson](https://april.eecs.umich.edu/pdfs/olson2010.pdf).
This algoritm is employed in the SLCAN backend in [PyUAVCAN library](http://uavcan.org/Implementations/Pyuavcan).

Also see the enclosed file documenting the common features of the SLCAN protocol: 
[Generic_SLCAN_API.pdf](Generic_SLCAN_API.pdf).

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
