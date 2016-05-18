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
#include "shell.hpp"
#include <board/board.hpp>
#include <unistd.h>
#include <cstdio>
#include <zubax_chibios/os.hpp>
#include <zubax_chibios/util/base64.hpp>
#include <ch.hpp>
#include <hal.h>

namespace cli
{
namespace
{

class RebootCommand : public shell::ICommandHandler
{
    const char* getName() const override { return "reboot"; }

    void execute(shell::BaseChannelWrapper&, int, char**) override
    {
        ::usleep(10000);
        board::restart();
    }
} static cmd_reboot;


class ZubaxIDCommand : public shell::ICommandHandler
{
    const char* getName() const override { return "zubax_id"; }

    void execute(shell::BaseChannelWrapper& ios, int, char**) override
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

        // TODO: Print firmware version from the descriptor
    }
} static cmd_zubax_id;


class CLIThread : public chibios_rt::BaseStaticThread<1024>
{
    shell::Shell<> shell_;

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

            shell::BaseChannelWrapper wrapper(os::getStdIOStream());
            shell_.runFor(wrapper, 100);
        }
    }

public:
    CLIThread() :
        shell_(shell::Mode::Silent)
    {
        shell_.addCommandHandler(&cmd_reboot);
        shell_.addCommandHandler(&cmd_zubax_id);
    }
} cli_thread;

} // namespace

void init()
{
    cli_thread.start(LOWPRIO + 1);
}

}
