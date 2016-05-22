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
#include "bootloader/app_shared.hpp"


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

/**
 * This struct is used to exchange data with the application.
 * Its format allows for future extensions.
 */
struct AppShared
{
    std::uint32_t reserved_a = 0;                               ///< Reserved for future use
    std::uint32_t reserved_b = 0;                               ///< Reserved for future use

    /*
     * UAVCAN part
     */
    std::uint32_t can_bus_speed = 0;                            ///< Reserved for future use
    std::uint8_t uavcan_node_id = 0;                            ///< Reserved for future use

    static constexpr std::uint8_t UAVCANFileNameMaxLength = 201;
    char uavcan_file_name[UAVCANFileNameMaxLength] = {};        ///< Reserved for future use

    /*
     * General part
     */
    bool stay_in_bootloader = false;
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
     * Parsing the app shared struct
     */
    {
        // This is so ugly it's almost perfect. FIXME invent something prettier than this.
        auto app_shared_marshaller = bootloader::app_shared::makeAppSharedMarshaller<app::AppShared>(
            &CAN->sFilterRegister[0].FR1,  &CAN->sFilterRegister[0].FR2,
            &CAN->sFilterRegister[1].FR1,  &CAN->sFilterRegister[1].FR2,
            &CAN->sFilterRegister[2].FR1,  &CAN->sFilterRegister[2].FR2,
            &CAN->sFilterRegister[3].FR1,  &CAN->sFilterRegister[3].FR2,
            &CAN->sFilterRegister[4].FR1,  &CAN->sFilterRegister[4].FR2,
            &CAN->sFilterRegister[5].FR1,  &CAN->sFilterRegister[5].FR2,
            &CAN->sFilterRegister[6].FR1,  &CAN->sFilterRegister[6].FR2,
            &CAN->sFilterRegister[7].FR1,  &CAN->sFilterRegister[7].FR2,
            &CAN->sFilterRegister[8].FR1,  &CAN->sFilterRegister[8].FR2,
            &CAN->sFilterRegister[9].FR1,  &CAN->sFilterRegister[9].FR2,
            &CAN->sFilterRegister[10].FR1, &CAN->sFilterRegister[10].FR2,
            &CAN->sFilterRegister[11].FR1, &CAN->sFilterRegister[11].FR2,
            &CAN->sFilterRegister[12].FR1, &CAN->sFilterRegister[12].FR2,
            &CAN->sFilterRegister[13].FR1, &CAN->sFilterRegister[13].FR2,
            &CAN->sFilterRegister[14].FR1, &CAN->sFilterRegister[14].FR2,
            &CAN->sFilterRegister[15].FR1, &CAN->sFilterRegister[15].FR2,
            &CAN->sFilterRegister[16].FR1, &CAN->sFilterRegister[16].FR2,
            &CAN->sFilterRegister[17].FR1, &CAN->sFilterRegister[17].FR2,
            &CAN->sFilterRegister[18].FR1, &CAN->sFilterRegister[18].FR2,
            &CAN->sFilterRegister[19].FR1, &CAN->sFilterRegister[19].FR2,
            &CAN->sFilterRegister[20].FR1, &CAN->sFilterRegister[20].FR2,
            &CAN->sFilterRegister[21].FR1, &CAN->sFilterRegister[21].FR2,
            &CAN->sFilterRegister[22].FR1, &CAN->sFilterRegister[22].FR2,
            &CAN->sFilterRegister[23].FR1, &CAN->sFilterRegister[23].FR2,
            &CAN->sFilterRegister[24].FR1, &CAN->sFilterRegister[24].FR2,
            &CAN->sFilterRegister[25].FR1, &CAN->sFilterRegister[25].FR2,
            &CAN->sFilterRegister[26].FR1, &CAN->sFilterRegister[26].FR2,
            &CAN->sFilterRegister[27].FR1, &CAN->sFilterRegister[27].FR2
        );

        const auto app_shared = app_shared_marshaller.read(bootloader::app_shared::AutoErase::EraseAfterRead);
        if (app_shared.second)
        {
            if (app_shared.first.stay_in_bootloader)
            {
                DEBUG_LOG("Boot cancelled by apps request\n");
                bl.cancelBoot();
            }
        }
        else
        {
            DEBUG_LOG("App shared struct not found\n");
        }
    }

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
