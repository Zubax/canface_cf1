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

#include "usb_cdc.hpp"
#include "cli.hpp"
#include "bootloader/loaders/ymodem.hpp"
#include <board/board.hpp>
#include <unistd.h>
#include <cstdio>
#include <zubax_chibios/os.hpp>
#include <zubax_chibios/util/base64.hpp>
#include <zubax_chibios/util/shell.hpp>
#include <ch.hpp>
#include <hal.h>

namespace cli
{
namespace
{

static bootloader::Bootloader* g_bootloader_ = nullptr;


static void printBootloaderState(os::shell::BaseChannelWrapper& ios)
{
    assert(g_bootloader_ != nullptr);
    const auto st = g_bootloader_->getState();
    ios.print("%s (%d)\n", bootloader::stateToString(st), st);
}


class RebootCommand : public os::shell::ICommandHandler
{
    const char* getName() const override { return "reboot"; }

    void execute(os::shell::BaseChannelWrapper&, int, char**) override
    {
        ::usleep(10000);
        board::restart();
    }
} static cmd_reboot;


class ZubaxIDCommand : public os::shell::ICommandHandler
{
    const char* getName() const override { return "zubax_id"; }

    void execute(os::shell::BaseChannelWrapper& ios, int, char**) override
    {
        ios.print("product_id   : '%s'\n", PRODUCT_ID_STRING);
        ios.print("product_name : '%s'\n", PRODUCT_NAME_STRING);
        ios.print("mode         : bootloader\n");

        ios.print("bl_version   : '%u.%u'\n", BL_VERSION_MAJOR, BL_VERSION_MINOR);
        ios.print("bl_vcs_commit: %u\n", GIT_HASH);
        ios.print("bl_build_date: %s\n", __DATE__);

        auto hw_version = board::detectHardwareVersion();
        ios.print("hw_version   : '%u.%u'\n", hw_version.major, hw_version.minor);

        char base64_buf[os::base64::predictEncodedDataLength(std::tuple_size<board::DeviceSignature>::value) + 1];

        ios.print("hw_unique_id : '%s'\n", os::base64::encode(board::readUniqueID(), base64_buf));

        board::DeviceSignature signature;
        if (board::tryReadDeviceSignature(signature))
        {
            ios.print("hw_signature : '%s'\n", os::base64::encode(signature, base64_buf));
        }

        assert(g_bootloader_ != nullptr);
        const auto appinfo = g_bootloader_->getAppInfo();
        if (appinfo.second)
        {
            const auto inf = appinfo.first;
            ios.print("fw_version   : '%u.%u'\n", inf.major_version, inf.minor_version);
            ios.print("fw_vcs_commit: %u\n", inf.vcs_commit);
        }
    }
} static cmd_zubax_id;


class StateCommand : public os::shell::ICommandHandler
{
    const char* getName() const override { return "state"; }
    void execute(os::shell::BaseChannelWrapper& ios, int, char**) override { printBootloaderState(ios); }
} static cmd_state;


class CancelBootCommand : public os::shell::ICommandHandler
{
    const char* getName() const override { return "cancel_boot"; }

    void execute(os::shell::BaseChannelWrapper& ios, int, char**) override
    {
        assert(g_bootloader_ != nullptr);
        g_bootloader_->cancelBoot();
        printBootloaderState(ios);
    }
} static cmd_cancel_boot;


class RequestBootCommand : public os::shell::ICommandHandler
{
    const char* getName() const override { return "request_boot"; }

    void execute(os::shell::BaseChannelWrapper& ios, int, char**) override
    {
        assert(g_bootloader_ != nullptr);
        g_bootloader_->requestBoot();
        printBootloaderState(ios);
    }
} static cmd_request_boot;


class DownloadCommand : public os::shell::ICommandHandler
{
    const char* getName() const override { return "download"; }

    void execute(os::shell::BaseChannelWrapper& ios, int, char**) override
    {
        assert(g_bootloader_ != nullptr);

        bootloader::ymodem_loader::YModemReceiver loader(ios.getChannel());     // TODO pushing stack really hard

        int res = g_bootloader_->upgradeApp(loader);
        if (res < 0)
        {
            ios.print("ERROR %d\n", res);
        }
    }
} static cmd_download;


class CLIThread : public chibios_rt::BaseStaticThread<2048>
{
    os::shell::Shell<> shell_;

    void main() override
    {
        for (;;)
        {
            const auto usb_port = usb_cdc::getSerialUSBDriver();
            const auto uart_port = &STDOUT_SD;

            const bool using_usb = os::getStdIOStream() == reinterpret_cast<::BaseChannel*>(usb_port);
            const bool usb_connected = usb_cdc::getState() == usb_cdc::State::Connected;

            if (using_usb != usb_connected)
            {
                DEBUG_LOG("Switching to %s\n", usb_connected ? "USB" : "UART");
                os::setStdIOStream(usb_connected ?
                                   reinterpret_cast<::BaseChannel*>(usb_port) :
                                   reinterpret_cast<::BaseChannel*>(uart_port));
                shell_.reset();
            }

            os::shell::BaseChannelWrapper wrapper(os::getStdIOStream());
            shell_.runFor(wrapper, 100);
        }
    }

public:
    CLIThread() :
        shell_(os::shell::Mode::Silent)
    {
        shell_.addCommandHandler(&cmd_reboot);
        shell_.addCommandHandler(&cmd_zubax_id);
        shell_.addCommandHandler(&cmd_state);
        shell_.addCommandHandler(&cmd_cancel_boot);
        shell_.addCommandHandler(&cmd_request_boot);
        shell_.addCommandHandler(&cmd_download);
    }
} cli_thread;

} // namespace

void init(bootloader::Bootloader& bl)
{
    g_bootloader_ = &bl;
    cli_thread.start(LOWPRIO + 1);
}

}
