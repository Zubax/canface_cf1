/*
 * Copyright (C) 2016  Zubax Robotics  <info@zubax.com>
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

#include <hal.h>
#include <zubax_chibios/bootloader/app_shared.hpp>

namespace bootloader_app_interface
{
/**
 * This struct is used to exchange data between the bootloader and the application.
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
    std::uint8_t uavcan_fw_server_node_id = 0;                  ///< Reserved for future use

    static constexpr std::uint8_t UAVCANFileNameMaxLength = 201;
    char uavcan_file_name[UAVCANFileNameMaxLength] = {};        ///< Reserved for future use

    /*
     * General part
     */
    bool stay_in_bootloader = false;
};


static inline auto makeMarshaller()
{
    // This is so ugly it's almost perfect. FIXME invent something prettier than this.
    return bootloader::app_shared::makeAppSharedMarshaller<AppShared>(
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
}


static inline auto readAndErase()
{
    return makeMarshaller().read(bootloader::app_shared::AutoErase::EraseAfterRead);
}

static inline void write(const AppShared& apsh)
{
    makeMarshaller().write(apsh);
}

}
