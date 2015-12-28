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

        os::MutexLocker mlocker(os::getStdIOMutex());
        chnWriteTimeout(os::getStdIOStream(), &buffer[0], frame_size, MS2ST(WriteTimeoutMSec));
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


void processCommand(int argc, char *argv[])
{
    std::printf("Command: ");
    for (int i = 0; i < argc; i++)
    {
        std::printf("%s", argv[i]);
    }
    std::puts("");
}


class CommandParser
{
    static constexpr unsigned BufferSize = 200;
    static constexpr unsigned MaxArgs = 5;
    char buf_[BufferSize + 1];
    std::uint8_t pos_ = 0;
    mutable char* token_ptr = nullptr;

    char* tokenize(char* str) const
    {
        if (str)
        {
            token_ptr = str;
        }
        char* token = token_ptr;
        if (!token)
        {
            return NULL;
        }
        static const char* const Delims = " \t";
        token += std::strspn(token, Delims);
        token_ptr = std::strpbrk(token, Delims);
        if (token_ptr)
        {
            *token_ptr++ = '\0';
        }
        return *token ? token : NULL;
    }

    void tokenizeAndProcess(char* buf) const
    {
        char* args[MaxArgs] = {nullptr};
        std::uint8_t idx = 0;
        char* token = tokenize(buf);
        while (token != nullptr)
        {
            args[idx++] = token;
            if (idx >= MaxArgs)
            {
                break;
            }
            token = tokenize(nullptr);
        }

        if (args[0] != nullptr)
        {
            processCommand(idx, args);
        }
    }

public:
    void addByte(std::uint8_t byte)
    {
        if (byte == '\r')                       // End of command (SLCAN)
        {
            if (pos_ > 0)
            {
                buf_[pos_] = '\0';
                tokenizeAndProcess(&buf_[0]);
            }
            reset();
        }
        else if (byte >= 32 && byte <= 126)     // Normal printable ASCII character
        {
            if (pos_ < BufferSize)
            {
                buf_[pos_] = char(byte);
                pos_ += 1;
            }
            else
            {
                reset();                        // Buffer overrun; silently drop the data
            }
        }
        else if (byte == 8 || byte == 127)      // DEL or BS (backspace)
        {
            if (pos_ > 0)
            {
                pos_ -= 1;
            }
        }
        else
        {
            reset();                            // Invalid byte - drop the current command
        }

        // Invariants
        assert(pos_ <= BufferSize);
    }

    void reset()
    {
        pos_ = 0;
    }
} command_parser_;

}
}

int main()
{
    /*
     * Initializing
     */
    auto watchdog = app::init();

    chibios_rt::BaseThread::setPriority(NORMALPRIO);

    app::background_thread_.start(LOWPRIO);
    app::rx_thread_.start(NORMALPRIO + 1);

    // This delay is not required, but it allows the USB driver to complete initialization before the loop begins
    watchdog.reset();
    ::sleep(1);
    watchdog.reset();

    /*
     * Running the serial port processing loop
     */
    static constexpr unsigned ReadTimeoutMSec = 5;

    const auto usb_serial = reinterpret_cast<::BaseChannel*>(usb_cdc::getSerialUSBDriver());
    const auto uart_port = reinterpret_cast<::BaseChannel*>(&STDOUT_SD);

    while (true)
    {
        watchdog.reset();

        const auto read_res = chnGetTimeout(os::getStdIOStream(), MS2ST(ReadTimeoutMSec));
        if (read_res >= 0)
        {
            app::command_parser_.addByte(static_cast<std::uint8_t>(read_res));
        }
        else if (read_res == STM_TIMEOUT)
        {
            // Switching interfaces if necessary
            const bool using_usb = os::getStdIOStream() == usb_serial;
            const bool usb_connected = usb_cdc::getState() == usb_cdc::State::Connected;
            if (using_usb != usb_connected)
            {
                os::lowsyslog("Using %s\n", usb_connected ? "USB" : "UART");
                os::setStdIOStream(usb_connected ? usb_serial : uart_port);
                app::command_parser_.reset();
            }
        }
        else
        {
            ;   // Some other error - ignoring silently
        }
    }
}
