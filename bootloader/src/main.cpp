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

#include "board/board.hpp"
#include "usb_cdc.hpp"
#include "cli.hpp"


namespace app
{
namespace
{

constexpr unsigned WatchdogTimeoutMSec = 1500;
constexpr unsigned ApplicationBootDelayMSec = 4000;
constexpr unsigned WatchdogTimeoutWhenBootingApplicationMSec = 10000;


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

    /*
     * CLI initialization
     */
    cli::init();

    return watchdog;
}

}
}

int main()
{
    /*
     * Initializing
     */
    auto watchdog = app::init();

    board::setStatusLED(true);

    chibios_rt::BaseThread::setPriority(LOWPRIO + 10);

    /*
     * Main loop
     */
    while (true)
    {
        watchdog.reset();
        ::sleep(1);
    }
}
