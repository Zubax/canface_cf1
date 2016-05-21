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

#include <ch.hpp>
#include <hal.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <zubax_chibios/os.hpp>
#include <zubax_chibios/platform/stm32/flash_writer.hpp>

#include "board/board.hpp"
#include "usb_cdc.hpp"
#include "cli.hpp"
#include "bootloader/bootloader.hpp"


namespace app
{
namespace
{

constexpr unsigned WatchdogTimeoutMSec = 1500;
constexpr unsigned ApplicationBootDelayMSec = 4000;
constexpr unsigned WatchdogTimeoutWhenBootingApplicationMSec = 10000;

/**
 * This class contains logic and hardcoded values that are SPECIFIC FOR THIS PARTICULAR MCU AND APPLICATION.
 */
class AppStorageBackend : public bootloader::IAppStorageBackend
{
    static constexpr unsigned FlashPageSize = 2048;
    static constexpr unsigned ApplicationAddress = FLASH_BASE + APPLICATION_OFFSET;

    static unsigned getFlashSize()
    {
        return 1024 * *reinterpret_cast<std::uint16_t*>(0x1FFFF7CC);
    }

    static bool correctOffsetAndSize(std::size_t& offset, std::size_t& size)
    {
        const auto flash_end = FLASH_BASE + getFlashSize();
        offset += ApplicationAddress;
        if (offset >= flash_end)
        {
            return false;
        }
        if ((offset + size) >= flash_end)
        {
            size = flash_end - offset;
        }
        return true;
    }

public:
    static constexpr std::int16_t ErrEraseFailed = 9001;
    static constexpr std::int16_t ErrWriteFailed = 9002;

    int beginUpgrade()   override { return 0; }
    int endUpgrade(bool) override { return 0; }

    int write(std::size_t offset, const void* data, std::size_t size) override
    {
        if (!correctOffsetAndSize(offset, size))
        {
            return 0;
        }
        os::stm32::FlashWriter writer;

        // Blank check byte-by-byte, erase page only if needed
        for (std::size_t blank_check_pos = offset; blank_check_pos < (offset + size); blank_check_pos++)
        {
            if (UNLIKELY(*reinterpret_cast<const std::uint8_t*>(blank_check_pos) != 0xFF))
            {
                DEBUG_LOG("Erasing page at %x\n", blank_check_pos);
                const bool ok = writer.erasePageAt(blank_check_pos);
                if (!ok)
                {
                    return -ErrEraseFailed;
                }
            }
        }

        // Write
        return writer.write(reinterpret_cast<const void*>(offset), data, size) ? size : -ErrWriteFailed;
    }

    int read(std::size_t offset, void* data, std::size_t size) override
    {
        if (!correctOffsetAndSize(offset, size))
        {
            return 0;
        }
        std::memmove(data, reinterpret_cast<const void*>(offset), size);
        return size;
    }
};

}
}

int main()
{
    auto watchdog = board::init(app::WatchdogTimeoutMSec);

    chibios_rt::BaseThread::setPriority(LOWPRIO);

    /*
     * USB initialization
     */
    const auto uid = board::readUniqueID();

    usb_cdc::DeviceSerialNumber sn;
    std::fill(sn.begin(), sn.end(), 0);
    std::copy(uid.begin(), uid.end(), sn.begin());

    watchdog.reset();
    usb_cdc::init(sn);                  // Must not exceed watchdog timeout
    watchdog.reset();

    board::setStatusLED(true);

    /*
     * Bootloader logic initialization
     */
    app::AppStorageBackend backend;

    bootloader::Bootloader bl(backend, app::ApplicationBootDelayMSec);

    cli::init(bl);

    /*
     * Main loop
     */
    while (!os::isRebootRequested())
    {
        watchdog.reset();
        (void)bl.getState();
        ::sleep(1);
    }

    watchdog.reset();

    if (os::isRebootRequested())
    {
        ::sleep(1);             // Providing some time for other components to react
        board::restart();
    }
}
