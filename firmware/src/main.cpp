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
#include <cstdio>
#include <zubax_chibios/os.hpp>

#include "board/board.hpp"
#include "usb_cdc.hpp"
#include "can_bus.hpp"

namespace app
{
namespace
{

constexpr unsigned WatchdogTimeoutMSec = 1500;


os::config::Param<bool> can_power_on("can.power_on", false);
os::config::Param<bool> can_terminator_on("can.terminator_on", false);


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

class BackgroundThread : public chibios_rt::BaseStaticThread<128>
{
    static constexpr unsigned BaseFrameMSec = 25;

    static std::pair<unsigned, unsigned> getStatusOnOffDurationMSec()
    {
        if (can::isStarted())
        {
            switch (can::getStatus().state)
            {
            case can::Status::State::ErrorActive:
            {
                return {50, 950};
            }
            case can::Status::State::ErrorPassive:
            {
                return {50, 250};
            }
            case can::Status::State::BusOff:
            {
                return {50, 50};
            }
            default:
            {
                assert(0);
                return {50, 50};
            }
            }
        }
        return {0, 0};
    }

    static void updateLED()
    {
        // Traffic LED
        board::setTrafficLED(can::hadActivity());

        // Status LED
        static bool status_on = false;
        static unsigned status_remaining = 0;

        if (status_remaining > 0)
        {
            status_remaining -= 1;
        }

        if (status_remaining <= 0)
        {
            const auto onoff = getStatusOnOffDurationMSec();
            if (onoff.first == 0 || status_on)
            {
                board::setStatusLED(false);
                status_on = false;
                status_remaining = onoff.second / BaseFrameMSec;
            }
            else
            {
                board::setStatusLED(true);
                status_on = true;
                status_remaining = onoff.first / BaseFrameMSec;
            }
        }
    }

    static void updateConfigs()
    {
        // TODO: only on request
        board::enableCANPower(can_power_on);
        board::enableCANTerminator(can_terminator_on);
    }

    void main() override
    {
        os::watchdog::Timer wdt;
        wdt.startMSec(WatchdogTimeoutMSec);     // This thread is quite low-priority, so large timeout is OK

        ::systime_t next_step_at = chVTGetSystemTime();

        while (true)
        {
            wdt.reset();

            updateLED();                        // LEDs must be served first in order to reduce jitter
            updateConfigs();

            next_step_at += MS2ST(BaseFrameMSec);
            os::sleepUntilChTime(next_step_at);
        }
    }
} background_thread_;


class RxThread : public chibios_rt::BaseStaticThread<512>
{
    static constexpr unsigned ReadTimeoutMSec = 5;

    void main() override
    {
        os::watchdog::Timer wdt;
        wdt.startMSec(ReadTimeoutMSec * 2);

        while (true)
        {
            wdt.reset();

            can::RxFrame rxf;
            const int res = can::receive(rxf, ReadTimeoutMSec);
            if (res > 0)
            {
                // TODO: SLCAN
                os::lowsyslog("%s: %s %u 0x%08x\n",
                              rxf.loopback ? "LB" : "RX",
                              rxf.failed ? "FAILED" : "OK",
                              unsigned(rxf.timestamp_systick), unsigned(rxf.frame.id));
            }
            else if (res == -can::ErrNotStarted)
            {
                ::usleep(1000);
            }
            else
            {
                ;
            }
        }
    }
} rx_thread_;

}
}

int main()
{
    auto watchdog = app::init();

    const auto can_res = can::start(125000, can::OptionLoopback);
    os::lowsyslog("CAN res: %d\n", can_res);

    chibios_rt::BaseThread::setPriority(NORMALPRIO);

    app::background_thread_.start(LOWPRIO);
    app::rx_thread_.start(NORMALPRIO + 1);

    while (true)
    {
        watchdog.reset();

        static can::Frame txf;
        txf.data[0]++;
        txf.dlc = 1;
        int res = can::send(txf, 200);
        os::lowsyslog("CAN res: %d\n", res);

        const auto stats = can::getStatistics();
        const auto status = can::getStatus();
        os::lowsyslog("err=%u  rx=%u  tx=%u  ptxmbx=%u  rec=%u  tec=%u  state=%u\n",
                      unsigned(stats.errors), unsigned(stats.frames_rx), unsigned(stats.frames_tx),
                      unsigned(stats.peak_tx_mailbox_index),
                      status.receive_error_counter, status.transmit_error_counter, unsigned(status.state));

        ::usleep(200000);
    }
}
