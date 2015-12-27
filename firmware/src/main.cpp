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
constexpr unsigned SLCANMaxFrameSize = 40;

os::config::Param<bool> cfg_can_power_on     ("can.power_on",           false);
os::config::Param<bool> cfg_can_terminator_on("can.terminator_on",      false);

os::config::Param<bool> cfg_timestamping_on("slcan.timestamping_on",    true);
os::config::Param<bool> cfg_flags_on       ("slcan.flags_on",           true);
os::config::Param<bool> cfg_loopback_on    ("slcan.loopback_on",        false);

struct ParamCache
{
    bool timestamping_on;
    bool flags_on;
    bool loopback_on;

    void reload()
    {
        timestamping_on = cfg_timestamping_on;
        flags_on        = cfg_flags_on;
        loopback_on     = cfg_loopback_on;
    }
} param_cache;

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

class BackgroundThread : public chibios_rt::BaseStaticThread<256>
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

    static void reloadConfigs()
    {
        os::lowsyslog("Reloading configs\n");           ///< TODO: remove later

        board::enableCANPower(cfg_can_power_on);
        board::enableCANTerminator(cfg_can_terminator_on);

        param_cache.reload();
    }

    void main() override
    {
        os::watchdog::Timer wdt;
        wdt.startMSec(WatchdogTimeoutMSec);     // This thread is quite low-priority, so large timeout is OK

        reloadConfigs();

        ::systime_t next_step_at = chVTGetSystemTime();
        unsigned cfg_modcnt = 0;

        while (true)
        {
            wdt.reset();

            updateLED();                        // LEDs must be served first in order to reduce jitter

            const unsigned new_cfg_modcnt = os::config::getModificationCounter();
            if (new_cfg_modcnt != cfg_modcnt)
            {
                cfg_modcnt = new_cfg_modcnt;
                reloadConfigs();
            }

            next_step_at += MS2ST(BaseFrameMSec);
            os::sleepUntilChTime(next_step_at);
        }
    }
} background_thread_;


class RxThread : public chibios_rt::BaseStaticThread<400>
{
    static constexpr unsigned ReadTimeoutMSec = 5;
    static constexpr unsigned WriteTimeoutMSec = 50;

    static inline std::uint8_t hex(std::uint8_t x)
    {
        const auto n = std::uint8_t((x & 0xF) + '0');
        return (n > '9') ? std::uint8_t(n + 'A' - '9' - 1) : n;
    }

    /**
     * General frame format:
     *  <type> <id> <dlc> <data> [timestamp msec] [flags]
     * Types:
     *  R - RTR extended
     *  r - RTR standard
     *  T - Data extended
     *  t - Data standard
     * Flags:
     *  L - this frame is a loopback frame; timestamp field contains TX timestamp
     */
    static void reportFrame(const can::RxFrame& f)
    {
        std::uint8_t buffer[SLCANMaxFrameSize];
        std::uint8_t* p = &buffer[0];

        if (f.failed)
        {
            return;
        }

        if (f.loopback && !param_cache.loopback_on)
        {
            return;
        }

        /*
         * Frame type
         */
        if (f.frame.isRemoteTransmissionRequest())
        {
            *p++ = f.frame.isExtended() ? 'R' : 'r';
        }
        else if (f.frame.isErrorFrame())
        {
            return;     // Not supported
        }
        else
        {
            *p++ = f.frame.isExtended() ? 'T' : 't';
        }

        /*
         * ID
         */
        {
            const std::uint32_t id = f.frame.id & f.frame.MaskExtID;
            if (f.frame.isExtended())
            {
                *p++ = hex(id >> 28);
                *p++ = hex(id >> 24);
                *p++ = hex(id >> 20);
                *p++ = hex(id >> 16);
                *p++ = hex(id >> 12);
            }
            *p++ = hex(id >> 8);
            *p++ = hex(id >> 4);
            *p++ = hex(id >> 0);
        }

        /*
         * DLC
         */
        *p++ = char('0' + f.frame.dlc);

        /*
         * Data
         */
        for (unsigned i = 0; i < f.frame.dlc; i++)
        {
            const std::uint8_t byte = f.frame.data[i];
            *p++ = hex(byte >> 4);
            *p++ = hex(byte);
        }

        /*
         * Timestamp
         */
        if (param_cache.timestamping_on)
        {
            const auto msec = std::uint16_t(f.timestamp_systick / (CH_CFG_ST_FREQUENCY / 1000));
            *p++ = hex(msec >> 12);
            *p++ = hex(msec >> 8);
            *p++ = hex(msec >> 4);
            *p++ = hex(msec >> 0);
        }

        /*
         * Flags
         */
        if (param_cache.flags_on)
        {
            if (f.loopback)
            {
                *p++ = 'L';
            }
        }

        /*
         * Finalization
         */
        *p++ = '\r';
        const auto frame_size = unsigned(p - &buffer[0]);
        assert(frame_size <= sizeof(buffer));

        os::MutexLocker mlocker(os::getStdOutMutex());
        chnWriteTimeout(os::getStdOutStream(), &buffer[0], frame_size, MS2ST(WriteTimeoutMSec));
    }

    void main() override
    {
        os::watchdog::Timer wdt;
        wdt.startMSec((ReadTimeoutMSec + WriteTimeoutMSec) * 2);

        while (true)
        {
            wdt.reset();

            can::RxFrame rxf;
            const int res = can::receive(rxf, ReadTimeoutMSec);
            if (res > 0)
            {
                reportFrame(rxf);
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
