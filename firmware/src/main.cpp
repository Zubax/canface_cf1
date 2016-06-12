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
#include <chprintf.h>

#include "board/board.hpp"
#include "usb_cdc.hpp"
#include "can_bus.hpp"

// This is ugly, do something better.
#include "../../bootloader/src/bootloader_app_interface.hpp"


namespace app
{
namespace
{
/**
 * This is the Brickproof Bootloader's app descriptor.
 * Details: https://github.com/PX4/Firmware/tree/nuttx_next/src/drivers/bootloaders/src/uavcan
 */
static const volatile struct __attribute__((packed))
{
    std::uint8_t signature[8]   = {'A','P','D','e','s','c','0','0'};
    std::uint64_t image_crc     = 0;
    std::uint32_t image_size    = 0;
    std::uint32_t vcs_commit    = GIT_HASH;
    std::uint8_t major_version  = FW_VERSION_MAJOR;
    std::uint8_t minor_version  = FW_VERSION_MINOR;
    std::uint8_t reserved[6]    = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
} _app_descriptor __attribute__((section(".app_descriptor")));


constexpr unsigned WatchdogTimeoutMSec = 1500;
constexpr unsigned CANTxTimeoutMSec = 50;

os::config::Param<unsigned> cfg_can_bitrate  ("can.bitrate",            1000000, 10000, 1000000); // Exposed via SLCAN
os::config::Param<bool> cfg_can_power_on     ("can.power_on",           false);
os::config::Param<bool> cfg_can_terminator_on("can.terminator_on",      false);

os::config::Param<bool> cfg_timestamping_on("slcan.timestamping_on",    true);                    // Exposed via SLCAN
os::config::Param<bool> cfg_flags_on       ("slcan.flags_on",           false);

os::config::Param<unsigned> cfg_baudrate("uart.baudrate", SERIAL_DEFAULT_BITRATE, 2400, 3000000); // Exposed via SLCAN

/**
 * This struct keeps cached configuration parameters for quick access.
 * It is updated automatically when configuration changes from the background thread.
 */
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
        if (can::isOpen())
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

            if (os::isRebootRequested())
            {
                ::usleep(10000);        // Providing time to send the response
                can::close();
                NVIC_SystemReset();
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
            // SLCAN format - [0, 60000) milliseconds
            const auto slcan_timestamp = std::uint16_t(f.timestamp_usec / 1000U);
            *p++ = nibble2hex(slcan_timestamp >> 12);
            *p++ = nibble2hex(slcan_timestamp >> 8);
            *p++ = nibble2hex(slcan_timestamp >> 4);
            *p++ = nibble2hex(slcan_timestamp >> 0);
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
            else if (res == -can::ErrClosed)
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
    void cmdConfig(int argc, char** argv)
    {
        (void)os::config::executeCLICommand(argc - 1, &argv[1]);
    }

    void cmdZubaxID(int argc, char** argv)
    {
        if (argc == 1)
        {
            std::printf("product_id   : '%s'\n", PRODUCT_ID_STRING);
            std::printf("product_name : '%s'\n", PRODUCT_NAME_STRING);

            std::printf("sw_version   : '%u.%u'\n", FW_VERSION_MAJOR, FW_VERSION_MINOR);
            std::printf("sw_vcs_commit: %u\n", unsigned(GIT_HASH));
            std::printf("sw_build_date: %s\n", __DATE__);

            {
                auto hw_version = board::detectHardwareVersion();
                std::printf("hw_version   : '%u.%u'\n", hw_version.major, hw_version.minor);
            }

            char base64_buf[os::base64::predictEncodedDataLength(std::tuple_size<board::DeviceSignature>::value) + 1];

            const auto uid = board::readUniqueID();
            std::printf("hw_unique_id : '%s'\n", os::base64::encode(uid, base64_buf));

            std::memset(&base64_buf[0], 0, sizeof(base64_buf));
            for (unsigned i = 0; i < uid.size(); i++)
            {
                chsnprintf(&base64_buf[i * 2], 3, "%02x", uid[i]);
            }
            std::printf("hw_info_url  : http://device.zubax.com/device_info?uid=%s\n", &base64_buf[0]);

            board::DeviceSignature signature;
            if (board::tryReadDeviceSignature(signature))
            {
                std::printf("hw_signature : '%s'\n", os::base64::encode(signature, base64_buf));
            }
        }
        else if (argc == 2)
        {
            const char* const encoded = argv[1];
            board::DeviceSignature sign;

            if (!os::base64::decode(sign, encoded))
            {
                std::puts("ERROR: Invalid base64");
            }

            if (!board::tryWriteDeviceSignature(sign))
            {
                std::puts("ERROR: Write failed");
            }
        }
        else
        {
            std::puts("ERROR: Invalid usage");
        }
    }

    void cmdStat(int, char**)
    {
        static constexpr auto FormatString = "%-22s: %s\n";

        // We can't use printf() for conversion because ChibiOS's printf() implementation does not support `long long`.
#       define STAT_PRINT_ONE_KEY(object, field) \
            std::printf(FormatString, STRINGIZE(field), os::uintToString(object . field).c_str());

        std::printf(FormatString, "open",  can::isOpen() ? "true" : "false");

        {
            const auto status = can::getStatus();

            std::printf(FormatString, "state", status.getStateAsString());
            STAT_PRINT_ONE_KEY(status, receive_error_counter)
            STAT_PRINT_ONE_KEY(status, transmit_error_counter)
        }

        {
            const auto statistics = can::getStatistics();

            STAT_PRINT_ONE_KEY(statistics, errors)
            STAT_PRINT_ONE_KEY(statistics, bus_off_events)
            STAT_PRINT_ONE_KEY(statistics, sw_rx_queue_overruns)
            STAT_PRINT_ONE_KEY(statistics, hw_rx_queue_overruns)
            STAT_PRINT_ONE_KEY(statistics, frames_tx)
            STAT_PRINT_ONE_KEY(statistics, frames_rx)
            STAT_PRINT_ONE_KEY(statistics, tx_queue_capacity)
            STAT_PRINT_ONE_KEY(statistics, tx_queue_peak_usage)
            STAT_PRINT_ONE_KEY(statistics, rx_queue_capacity)
            STAT_PRINT_ONE_KEY(statistics, rx_queue_peak_usage)
            STAT_PRINT_ONE_KEY(statistics, tx_mailbox_peak_usage)
        }

        std::printf("%-22s: %.1f\n", "bus_voltage", board::getBusVoltage());
    }

    void cmdReboot(int, char**)
    {
        os::requestReboot();
    }

    void cmdBootloader(int, char**)
    {
        bootloader_app_interface::AppShared apsh;
        apsh.stay_in_bootloader = true;
        bootloader_app_interface::write(apsh);

        os::requestReboot();
    }

    static bool startsWith(const char* const str, const char* const prefix)
    {
        return std::strncmp(prefix, str, std::strlen(prefix)) == 0;
    }

    const char* processComplexCommand(char* buf, void (CommandProcessor::*handler)(int, char**))
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
            (this->*handler)(idx, args);
        }

        // Returning the end of the multi-line response marker
        return "\x03\r\n";
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
        else if LIKELY(cmd[0] == 'r' && cmd[1] <= '9')  // The second condition is needed to avoid greedy matching
        {                                               // See long commands below
            return emitFrameRTRStd(cmd) ? "z\r" : "\a";
        }
        else
        {
            ; // Looking further
        }

        /*
         * Complex commands
         * These are handled before the single-letter SLCAN commands to avoid unwanted greedy matching
         */
        if (startsWith(cmd, "cfg"))
        {
            return processComplexCommand(cmd, &CommandProcessor::cmdConfig);
        }
        else if (startsWith(cmd, "zubax_id"))
        {
            return processComplexCommand(cmd, &CommandProcessor::cmdZubaxID);
        }
        else if (startsWith(cmd, "stat"))
        {
            return processComplexCommand(cmd, &CommandProcessor::cmdStat);
        }
        else if (startsWith(cmd, "bootloader"))
        {
            return processComplexCommand(cmd, &CommandProcessor::cmdBootloader);
        }
        else if (startsWith(cmd, "reboot"))
        {
            return processComplexCommand(cmd, &CommandProcessor::cmdReboot);
        }
        else
        {
            ;   // No handler
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
            return getASCIIStatusCode(0 <= can::open(cfg_can_bitrate.get()));
        }
        case 'L':               // Open CAN in listen-only mode
        {
            DEBUG_LOG("Open silent\n");
            return getASCIIStatusCode(0 <= can::open(cfg_can_bitrate.get(), can::OptionSilentMode));
        }
        case 'l':               // Open CAN with loopback enabled
        {
            DEBUG_LOG("Open loopback\n");
            return getASCIIStatusCode(0 <= can::open(cfg_can_bitrate.get(), can::OptionLoopback));
        }
        case 'C':               // Close CAN
        {
            can::close();
            DEBUG_LOG("Closed\n");
            return getASCIIStatusCode(true);
        }
        case 'M':               // Set CAN acceptance filter ID
        case 'm':               // Set CAN acceptance filter mask
        {
            /*
             * These SLCAN commands were designed to work with the SJA1000 controller.
             * SJA1000's hardware acceptance filters are very peculiar to say the least,
             * so implementing compatible commands would require to implement all of that ugly trickery
             * to mimic SJA1000.
             * I see litle value in prolonging the life of such brain-dead conventions, so this feature will
             * remain unimplemented until someone explicitly requested it.
             * If you REALLY need this feature, please contact us at http://productforums.zubax.com.
             */
            return getASCIIStatusCode(true);    // Returning success for compatibility reasons
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
            const std::uint64_t rx_overrun_cnt = stats.hw_rx_queue_overruns + stats.sw_rx_queue_overruns;

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

    /*
     * Running the serial port processing loop
     */
    static constexpr unsigned ReadTimeoutMSec = 5;

    const auto usb_port = usb_cdc::getSerialUSBDriver();
    const auto uart_port = &STDOUT_SD;

    while (true)
    {
        watchdog.reset();

        ::BaseChannel* const stdio_stream = os::getStdIOStream();

        static std::uint8_t buf[128];

        // First, read as much as possible without blocking to maximize throughput; if nothing is available, block
        std::size_t nread = chnReadTimeout(stdio_stream, buf, sizeof(buf), TIME_IMMEDIATE);
        if UNLIKELY(nread == 0)         // This branch will be taken only when traffic is low
        {
            nread = chnReadTimeout(stdio_stream, buf, 1, MS2ST(ReadTimeoutMSec));
        }

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
            const bool using_usb = reinterpret_cast<::BaseChannel*>(stdio_stream) ==
                                   reinterpret_cast<::BaseChannel*>(usb_port);
            const bool usb_connected = usb_cdc::getState() == usb_cdc::State::Connected;
            if (using_usb != usb_connected)
            {
                DEBUG_LOG("Switching to %s\n", usb_connected ? "USB" : "UART");
                os::setStdIOStream(usb_connected ?
                                   reinterpret_cast<::BaseChannel*>(usb_port) :
                                   reinterpret_cast<::BaseChannel*>(uart_port));
                app::command_parser_.reset();
            }
        }
    }
}
