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
#include <zubax_chibios/os.hpp>

#include "board/board.hpp"
#include "usb_cdc.hpp"
#include "can_bus.hpp"

namespace app
{
namespace
{

constexpr unsigned WatchdogTimeoutMSec = 1500;

auto init()
{
    /*
     * Basic initialization
     */
    auto watchdog = board::init(WatchdogTimeoutMSec);

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

    return watchdog;
}

}
}

int main()
{
    auto watchdog = app::init();

    board::enableCANPower(true);
    board::enableCANTerminator(true);

    const auto can_res = can::start(1000000, can::OptionLoopback);
    os::lowsyslog("CAN res: %d\n", can_res);

    while (true)
    {
        watchdog.reset();
        board::setStatusLED(true);
        board::setTrafficLED(true);

        int res = can::send(can::Frame(), 200);
        os::lowsyslog("CAN res: %d\n", res);

        watchdog.reset();
        board::setStatusLED(false);
        board::setTrafficLED(false);

        ::usleep(200000);
    }
}
