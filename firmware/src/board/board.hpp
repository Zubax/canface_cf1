/*
 * Copyright (C) 2015  Zubax Robotics  <info@zubax.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#pragma once

#include <cstdint>
#include <array>
#include <zubax_chibios/os.hpp>

namespace board
{

os::watchdog::Timer init(unsigned watchdog_timeout_msec, os::config::Param<unsigned>& cfg_uart_baudrate);

void reconfigureUART(unsigned baudrate);

__attribute__((noreturn))
void die();

void restart();

void setStatusLED(bool state);
void setTrafficLED(bool state);

void enableCANPower(bool state);
void enableCANTerminator(bool state);

float getBusVoltage();

typedef std::array<std::uint8_t, 16> UniqueID;
UniqueID readUniqueID();

typedef std::array<std::uint8_t, 128> DeviceSignature;
bool tryReadDeviceSignature(DeviceSignature& out_sign);
bool tryWriteDeviceSignature(const DeviceSignature& sign);

struct HardwareVersion
{
    std::uint8_t major;
    std::uint8_t minor;
};

HardwareVersion detectHardwareVersion();

}
