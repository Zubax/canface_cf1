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

#include "board.hpp"
#include <cstring>
#include <ch.hpp>
#include <hal.h>
#include <zubax_chibios/os.hpp>
#include <zubax_chibios/watchdog.hpp>
#include <zubax_chibios/config.hpp>
#include <unistd.h>

/// Provided by linker
const extern std::uint8_t DeviceSignatureStorage[];

namespace board
{

void init()
{
    halInit();
    chSysInit();
    sdStart(&STDOUT_SD, NULL);

    os::lowsyslog(PRODUCT_NAME_STRING " %d.%d.%x\n", FW_VERSION_MAJOR, FW_VERSION_MINOR, GIT_HASH);
    os::watchdog::init();

    while (true)
    {
        const int res = os::config::init();
        if (res >= 0)
        {
            break;
        }
        os::lowsyslog("Config init failed %i\n", res);
        ::sleep(1);
    }
}

__attribute__((noreturn))
void die(int error)
{
    os::lowsyslog("Fatal error %i\n", error);
    while (1)
    {
        //setStatusLed(false);          // TODO: LED indication
        ::usleep(25000);
        //setStatusLed(true);
        ::usleep(25000);
    }
}

void restart()
{
    NVIC_SystemReset();
}

void readUniqueID(UniqueID& out_bytes)
{
    std::memcpy(out_bytes.data(), reinterpret_cast<const void*>(0x1FFFF7AC), std::tuple_size<UniqueID>::value);
}

bool tryReadDeviceSignature(DeviceSignature& out_sign)
{
    std::memcpy(out_sign.data(), &DeviceSignatureStorage[0], std::tuple_size<DeviceSignature>::value);

    bool valid = false;
    for (auto x : out_sign)
    {
        if (x != 0xFF && x != 0x00)          // All 0xFF/0x00 is not a valid signature, it's empty storage
        {
            valid = true;
            break;
        }
    }

    return valid;
}

bool tryWriteDeviceSignature(const DeviceSignature& sign)
{
    {
        DeviceSignature dummy;
        if (tryReadDeviceSignature(dummy))
        {
            return false;               // Already written
        }
    }

    // Before flash can be written, the source must be aligned.
    alignas(4) std::uint8_t aligned_buffer[std::tuple_size<DeviceSignature>::value];
    std::copy(std::begin(sign), std::end(sign), std::begin(aligned_buffer));

    stm32_flash_writer::Writer writer;

    return writer.write(&DeviceSignatureStorage[0], &aligned_buffer[0], sizeof(aligned_buffer));
}

}

/*
 * Early init from ChibiOS
 */
extern "C"
{

void __early_init(void)
{
    stm32_clock_init();
}

void boardInit(void)
{
}

}
