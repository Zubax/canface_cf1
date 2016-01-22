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
#include <zubax_chibios/util/base64.hpp>

#include "board/board.hpp"
#include "usb_cdc.hpp"
#include "can_bus.hpp"


namespace app
{
namespace
{

constexpr unsigned WatchdogTimeoutMSec = 1500;
constexpr unsigned CANTxTimeoutMSec = 50;

os::config::Param<unsigned> cfg_can_bitrate  ("can.bitrate",            1000000, 10000, 1000000); // Exposed via SLCAN
os::config::Param<bool> cfg_can_power_on     ("can.power_on",           false);
os::config::Param<bool> cfg_can_terminator_on("can.terminator_on",      false);

os::config::Param<bool> cfg_timestamping_on("slcan.timestamping_on",    true);                    // Exposed via SLCAN
os::config::Param<bool> cfg_flags_on       ("slcan.flags_on",           true);

os::config::Param<unsigned> cfg_baudrate("uart.baudrate", SERIAL_DEFAULT_BITRATE, 2400, 3000000); // Exposed via SLCAN

struct ParamCache
{
    bool timestamping_on;
    bool flags_on;

    void reload()
    {
        timestamping_on = cfg_timestamping_on;
        flags_on        = cfg_flags_on;
    }
} param_cache;


auto init()
{
    /*
     * Basic initialization
     */
    auto watchdog = board::init(WatchdogTimeoutMSec, cfg_baudrate);

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

class BackgroundThread : public chibios_rt::BaseStaticThread<512>
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
        board::enableCANPower(cfg_can_power_on);
        board::enableCANTerminator(cfg_can_terminator_on);

        board::reconfigureUART(cfg_baudrate.get());

        param_cache.reload();
    }

    void main() override
    {
        // This thread does not have a watchdog, it's intentional
        reloadConfigs();

        ::systime_t next_step_at = chVTGetSystemTime();
        unsigned cfg_modcnt = 0;

        while (true)
        {
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


inline std::uint8_t nibble2hex(std::uint8_t x)
{
    // Allocating in RAM because it's faster
    static std::uint8_t ConversionTable[] __attribute__((section(".data"))) =
    {
        '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
    };
    return ConversionTable[x & 0x0F];
}


class RxThread : public chibios_rt::BaseStaticThread<512>
{
    static constexpr unsigned ReadTimeoutMSec = 5;
    static constexpr unsigned WriteTimeoutMSec = 50;

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
        constexpr unsigned SLCANMaxFrameSize = 40;
        std::uint8_t buffer[SLCANMaxFrameSize];
        std::uint8_t* p = &buffer[0];

        if UNLIKELY(f.failed)
        {
            return;
        }

        /*
         * Frame type
         */
        if UNLIKELY(f.frame.isRemoteTransmissionRequest())
        {
            *p++ = f.frame.isExtended() ? 'R' : 'r';
        }
        else if UNLIKELY(f.frame.isErrorFrame())
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
            if LIKELY(f.frame.isExtended())
            {
                *p++ = nibble2hex(id >> 28);
                *p++ = nibble2hex(id >> 24);
                *p++ = nibble2hex(id >> 20);
                *p++ = nibble2hex(id >> 16);
                *p++ = nibble2hex(id >> 12);
            }
            *p++ = nibble2hex(id >> 8);
            *p++ = nibble2hex(id >> 4);
            *p++ = nibble2hex(id >> 0);
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
            *p++ = nibble2hex(byte >> 4);
            *p++ = nibble2hex(byte);
        }

        /*
         * Timestamp
         */
        if LIKELY(param_cache.timestamping_on)
        {
            const auto msec = std::uint16_t(f.timestamp_systick / (CH_CFG_ST_FREQUENCY / 1000));
            *p++ = nibble2hex(msec >> 12);
            *p++ = nibble2hex(msec >> 8);
            *p++ = nibble2hex(msec >> 4);
            *p++ = nibble2hex(msec >> 0);
        }

        /*
         * Flags
         */
        if LIKELY(param_cache.flags_on)
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
            if LIKELY(res > 0)
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

// Using global state is UGLY, but it's also FASTER
static bool hex2nibble_error;
std::uint8_t hex2nibble(char ch)
{
    // Must go into RAM, not flash, because flash is slow
    static std::uint8_t ConversionTable[] __attribute__((section(".data"))) =
    {
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255,
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9,           // 0..9
        255, 255, 255, 255, 255, 255, 255,
        10, 11, 12, 13, 14, 15,                 // A..F
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255,
        10, 11, 12, 13, 14, 15,                 // a..f
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255
    };
    static_assert(sizeof(ConversionTable) == 256, "ConversionTable");
    const std::uint8_t out = ConversionTable[int(ch)];
    if UNLIKELY(out == 255)
    {
        hex2nibble_error = true;
    }
    return out;
}

/**
 * General frame format:
 *  <type> <id> <dlc> <data>
 * The emitting functions below are highly optimized for speed.
 */
inline bool emitFrameDataExt(const char* cmd)
{
    can::Frame f;
    hex2nibble_error = false;
    f.id = f.FlagEFF |
           (hex2nibble(cmd[1]) << 28) |
           (hex2nibble(cmd[2]) << 24) |
           (hex2nibble(cmd[3]) << 20) |
           (hex2nibble(cmd[4]) << 16) |
           (hex2nibble(cmd[5]) << 12) |
           (hex2nibble(cmd[6]) <<  8) |
           (hex2nibble(cmd[7]) <<  4) |
           (hex2nibble(cmd[8]) <<  0);
    if UNLIKELY(cmd[9] < '0' || cmd[9] > ('0' + can::Frame::MaxDataLen))
    {
        return false;
    }
    f.dlc = cmd[9] - '0';
    assert(f.dlc <= can::Frame::MaxDataLen);
    {
        const char* p = &cmd[10];
        for (unsigned i = 0; i < f.dlc; i++)
        {
            f.data[i] = (hex2nibble(*p) << 4) | hex2nibble(*(p + 1));
            p += 2;
        }
    }
    if UNLIKELY(hex2nibble_error)
    {
        return false;
    }
    return 0 <= can::send(f, CANTxTimeoutMSec);
}

inline bool emitFrameDataStd(const char* cmd)
{
    can::Frame f;
    hex2nibble_error = false;
    f.id = (hex2nibble(cmd[1]) << 8) |
           (hex2nibble(cmd[2]) << 4) |
           (hex2nibble(cmd[3]) << 0);
    if UNLIKELY(cmd[4] < '0' || cmd[4] > ('0' + can::Frame::MaxDataLen))
    {
        return false;
    }
    f.dlc = cmd[4] - '0';
    assert(f.dlc <= can::Frame::MaxDataLen);
    {
        const char* p = &cmd[5];
        for (unsigned i = 0; i < f.dlc; i++)
        {
            f.data[i] = (hex2nibble(*p) << 4) | hex2nibble(*(p + 1));
            p += 2;
        }
    }
    if UNLIKELY(hex2nibble_error)
    {
        return false;
    }
    return 0 <= can::send(f, CANTxTimeoutMSec);
}

inline bool emitFrameRTRExt(const char* cmd)
{
    can::Frame f;
    hex2nibble_error = false;
    f.id = f.FlagEFF | f.FlagRTR |
           (hex2nibble(cmd[1]) << 28) |
           (hex2nibble(cmd[2]) << 24) |
           (hex2nibble(cmd[3]) << 20) |
           (hex2nibble(cmd[4]) << 16) |
           (hex2nibble(cmd[5]) << 12) |
           (hex2nibble(cmd[6]) <<  8) |
           (hex2nibble(cmd[7]) <<  4) |
           (hex2nibble(cmd[8]) <<  0);
    if UNLIKELY(cmd[9] < '0' || cmd[9] > ('0' + can::Frame::MaxDataLen))
    {
        return false;
    }
    f.dlc = cmd[9] - '0';
    assert(f.dlc <= can::Frame::MaxDataLen);
    if UNLIKELY(hex2nibble_error)
    {
        return false;
    }
    return 0 <= can::send(f, CANTxTimeoutMSec);
}

inline bool emitFrameRTRStd(const char* cmd)
{
    can::Frame f;
    hex2nibble_error = false;
    f.id = f.FlagRTR |
           (hex2nibble(cmd[1]) << 8) |
           (hex2nibble(cmd[2]) << 4) |
           (hex2nibble(cmd[3]) << 0);
    if UNLIKELY(cmd[4] < '0' || cmd[4] > ('0' + can::Frame::MaxDataLen))
    {
        return false;
    }
    f.dlc = cmd[4] - '0';
    assert(f.dlc <= can::Frame::MaxDataLen);
    if UNLIKELY(hex2nibble_error)
    {
        return false;
    }
    return 0 <= can::send(f, CANTxTimeoutMSec);
}


class CommandProcessor
{
    const char* cmdConfig(int argc, char** argv)
    {
        (void)os::config::executeCLICommand(argc - 1, &argv[1]);
        return getASCIIStatusCode(true);
    }

    const char* cmdZubaxID(int argc, char** argv)
    {
        if (argc == 1)
        {
            std::printf("product_id   : '%s'\n", PRODUCT_ID_STRING);
            std::printf("product_name : '%s'\n", PRODUCT_NAME_STRING);

            std::printf("sw_version   : '%u.%u'\n", FW_VERSION_MAJOR, FW_VERSION_MINOR);
            std::printf("sw_vcs_commit: %u\n", unsigned(GIT_HASH));
            std::printf("sw_build_date: %s\n", __DATE__);

            auto hw_version = board::detectHardwareVersion();
            std::printf("hw_version   : '%u.%u'\n", hw_version.major, hw_version.minor);

            char base64_buf[os::base64::predictEncodedDataLength(std::tuple_size<board::DeviceSignature>::value) + 1];

            const auto uid = board::readUniqueID();
            std::printf("hw_unique_id : '%s'\n", os::base64::encode(uid, base64_buf));

            // TODO: read and report the signature
        }
        else if (argc == 2)
        {
            // TODO: signature installation
            (void)argv;
        }
        else
        {
            return getASCIIStatusCode(false);
        }

        return getASCIIStatusCode(true);
    }

    static bool startsWith(const char* const str, const char* const prefix)
    {
        return std::strncmp(prefix, str, std::strlen(prefix)) == 0;
    }

    const char* processComplexCommand(char* buf, const char* (CommandProcessor::*handler)(int, char**))
    {
        // Replying with echo
        std::puts(buf);

        // Parsing the line
        class Tokenizer
        {
            char* token_ptr = nullptr;

        public:
            char* tokenize(char* str)
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
        };

        constexpr unsigned MaxArgs = 5;
        char* args[MaxArgs] = {nullptr};
        std::uint8_t idx = 0;
        Tokenizer tokenizer;
        char* token = tokenizer.tokenize(buf);
        while (token != nullptr)
        {
            args[idx++] = token;
            if (idx >= MaxArgs)
            {
                break;
            }
            token = tokenizer.tokenize(nullptr);
        }

        // Invoking the handler
        if (args[0] != nullptr)
        {
            return (this->*handler)(idx, args);
        }
        return getASCIIStatusCode(false);
    }

    static inline const char* getASCIIStatusCode(bool status) { return status ? "\r" : "\a"; }

public:
    /**
     * Accepts command string, returns response string or nullptr if no response is needed.
     */
    const char* processCommand(char* cmd)
    {
        /*
         * High-traffic SLCAN commands go first
         */
        if LIKELY(cmd[0] == 'T')
        {
            return emitFrameDataExt(cmd) ? "Z\r" : "\a";
        }
        else if LIKELY(cmd[0] == 't')
        {
            return emitFrameDataStd(cmd) ? "z\r" : "\a";
        }
        else if LIKELY(cmd[0] == 'R')
        {
            return emitFrameRTRExt(cmd) ? "Z\r" : "\a";
        }
        else if LIKELY(cmd[0] == 'r')
        {
            return emitFrameRTRStd(cmd) ? "z\r" : "\a";
        }
        else
        {
            ; // Looking further
        }

        /*
         * Regular SLCAN commands
         */
        switch (cmd[0])
        {
        case 'S':               // Set CAN bitrate
        {
            if (cmd[1] < '0' || cmd[1] > '9')
            {
                return getASCIIStatusCode(false);
            }

            unsigned br = unsigned(std::atoi(&cmd[1]));
            switch (br)
            {
            case 0: br =   10000; break;
            case 1: br =   20000; break;
            case 2: br =   50000; break;
            case 3: br =  100000; break;
            case 4: br =  125000; break;
            case 5: br =  250000; break;
            case 6: br =  500000; break;
            case 7: br =  800000; break;
            case 8: br = 1000000; break;
            default: break;
            }
            DEBUG_LOG("Bitrate %u\n", br);

            return getASCIIStatusCode(cfg_can_bitrate.setAndSave(br) >= 0);
        }
        case 'O':               // Open CAN in normal mode
        {
            DEBUG_LOG("Open normal\n");
            return getASCIIStatusCode(0 <= can::start(cfg_can_bitrate.get()));
        }
        case 'L':               // Open CAN in listen-only mode
        {
            DEBUG_LOG("Open silent\n");
            return getASCIIStatusCode(0 <= can::start(cfg_can_bitrate.get(), can::OptionSilentMode));
        }
        case 'l':               // Open CAN with loopback enabled
        {
            DEBUG_LOG("Open loopback\n");
            return getASCIIStatusCode(0 <= can::start(cfg_can_bitrate.get(), can::OptionLoopback));
        }
        case 'C':               // Close CAN
        {
            can::stop();
            DEBUG_LOG("Closed\n");
            return getASCIIStatusCode(true);
        }
        case 'M':               // Set CAN acceptance filter ID
        {
            // TODO: implement
            return getASCIIStatusCode(true);
        }
        case 'm':               // Set CAN acceptance filter mask
        {
            // TODO: implement
            return getASCIIStatusCode(true);
        }
        case 'U':               // Set UART baud rate, see http://www.can232.com/docs/can232_v3.pdf
        {
            if (cmd[1] < '0' || cmd[1] > '9')
            {
                return getASCIIStatusCode(false);
            }

            unsigned baudrate = unsigned(std::atoi(&cmd[1]));
            switch (baudrate)
            {
            case 0: baudrate = 230400; break;
            case 1: baudrate = 115200; break;
            case 2: baudrate =  57600; break;
            case 3: baudrate =  38400; break;
            case 4: baudrate =  19200; break;
            case 5: baudrate =   9600; break;
            case 6: baudrate =   2400; break;
            default: break;
            }
            DEBUG_LOG("Baudrate %u\n", baudrate);

            return getASCIIStatusCode(cfg_baudrate.setAndSave(baudrate) >= 0);
        }
        case 'Z':               // Enable/disable RX and loopback timestamping
        {
            if (cmd[1] < '0' || cmd[1] > '1')
            {
                return getASCIIStatusCode(false);
            }

            const bool on = cmd[1] == '1';
            DEBUG_LOG("Timestamping %u\n", unsigned(on));

            return getASCIIStatusCode(cfg_timestamping_on.setAndSave(on) >= 0);
        }
        case 'F':               // Get status flags
        {
            static constexpr unsigned FlagRxOverrun    = 1 << 3;
            static constexpr unsigned FlagErrorPassive = 1 << 5;
            static constexpr unsigned FlagBusOff       = 1 << 7;        ///< Instead of bus error flag

            std::uint8_t response = 0;

            const auto status = can::getStatus();

            // CAN state flags
            if (status.state == status.State::ErrorPassive ||
                status.state == status.State::BusOff)
            {
                response |= FlagErrorPassive;
            }
            if (status.state == status.State::BusOff)
            {
                response |= FlagBusOff;
            }

            // RX overrun flag
            const auto stats = can::getStatistics();

            static std::uint64_t last_rx_overrun_cnt = 0;
            const std::uint64_t rx_overrun_cnt = stats.hw_rx_overruns + stats.sw_rx_overruns;

            if (rx_overrun_cnt > last_rx_overrun_cnt)
            {
                response |= FlagRxOverrun;
            }
            last_rx_overrun_cnt = rx_overrun_cnt;

            DEBUG_LOG("Flags %02X\n", unsigned(response));

            // Responding
            std::printf("F%02X\r", unsigned(response));
            return nullptr;
        }
        case 'V':               // HW/SW version
        {
            std::printf("V%x%x%x%x\r", HW_VERSION, 0, FW_VERSION_MAJOR, FW_VERSION_MINOR);
            return nullptr;
        }
        case 'N':               // Serial number
        {
            char buf[std::tuple_size<board::UniqueID>::value * 2 + 1] = { '\0' };
            char* pos = &buf[0];
            for (auto x : board::readUniqueID())
            {
                *pos++ = nibble2hex(x >> 4);
                *pos++ = nibble2hex(x);
            }
            *pos++ = '\0';
            std::printf("N%s\r", &buf[0]);
            return nullptr;
        }
        default:
        {
            break;
        }
        }

        /*
         * Complex commands
         */
        if (startsWith(cmd, "cfg"))
        {
            return processComplexCommand(cmd, &CommandProcessor::cmdConfig);
        }
        else if (startsWith(cmd, "zubax_id"))
        {
            return processComplexCommand(cmd, &CommandProcessor::cmdZubaxID);
        }
        else if (startsWith(cmd, "_stat"))
        {
            // TODO: implement
        }
        else if (startsWith(cmd, "_reboot"))
        {
            // TODO: implement
        }
        else
        {
            ;   // No handler
        }

        return getASCIIStatusCode(false);
    }
};


class CommandParser
{
    static constexpr unsigned BufferSize = 200;
    char buf_[BufferSize + 1];
    std::uint8_t pos_ = 0;

    CommandProcessor proc_;

public:
    /**
     * Please keep in mind that this function is strongly optimized for speed.
     */
    inline void addByte(const std::uint8_t byte)
    {
        if LIKELY((byte >= 32 && byte <= 126))                  // Normal printable ASCII character
        {
            if LIKELY(pos_ < BufferSize)
            {
                buf_[pos_] = char(byte);
                pos_ += 1;
            }
            else
            {
                reset();                                        // Buffer overrun; silently drop the data
            }
        }
        else if LIKELY(byte == '\r')                            // End of command (SLCAN)
        {
            // Processing the command
            buf_[pos_] = '\0';
            const char* const response = proc_.processCommand(reinterpret_cast<char*>(&buf_[0]));
            reset();

            // Sending the response if provided
            if LIKELY(response != nullptr)
            {
                os::MutexLocker mlocker(os::getStdIOMutex());
                chnWriteTimeout(os::getStdIOStream(), reinterpret_cast<const std::uint8_t*>(response),
                                std::strlen(response), MS2ST(1));
            }
        }
        else if UNLIKELY(byte == 8 || byte == 127)              // DEL or BS (backspace)
        {
            if (pos_ > 0)
            {
                pos_ -= 1;
            }
        }
        else                                                    // This also includes Ctrl+C, Ctrl+D
        {
            reset();                                            // Invalid byte - drop the current command
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
    app::rx_thread_.start(NORMALPRIO - 1);

    // This delay is not required, but it allows the USB driver to complete initialization before the loop begins
    watchdog.reset();
    ::sleep(1);
    watchdog.reset();

    /*
     * Running the serial port processing loop
     */
    static constexpr unsigned ReadTimeoutMSec = 5;

    const auto usb_serial = usb_cdc::getSerialUSBDriver();
    const auto uart_port = &STDOUT_SD;

    while (true)
    {
        watchdog.reset();

        ::BaseChannel* const stdio_stream = os::getStdIOStream();
        const bool using_usb = reinterpret_cast<::BaseChannel*>(stdio_stream) ==
                               reinterpret_cast<::BaseChannel*>(usb_serial);
        std::size_t nread = using_usb ? usb_serial->iqueue.q_counter : uart_port->iqueue.q_counter;

        static std::uint8_t buf[64];
        nread = chnReadTimeout(stdio_stream, buf, std::max(1U, std::min<unsigned>(sizeof(buf), nread)),
                               MS2ST(ReadTimeoutMSec));

        if LIKELY(nread > 0)
        {
            for (unsigned i = 0; i < nread; i++)
            {
                app::command_parser_.addByte(buf[i]);
            }
        }
        else
        {
            // Switching interfaces if necessary
            const bool usb_connected = usb_cdc::getState() == usb_cdc::State::Connected;
            if (using_usb != usb_connected)
            {
                os::setStdIOStream(usb_connected ?
                                   reinterpret_cast<::BaseChannel*>(usb_serial) :
                                   reinterpret_cast<::BaseChannel*>(uart_port));
                app::command_parser_.reset();
            }
        }
    }
}
