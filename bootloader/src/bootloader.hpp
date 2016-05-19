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

#include <zubax_chibios/os.hpp>
#include <cstdint>
#include <utility>
#include <array>
#include <cassert>


namespace bootloader
{
/**
 * Bootloader states. Some of the states are designed as commands to the outer logic, e.g. @ref ReadyToBoot
 * means that the application should be started.
 */
enum class State
{
    NoAppToBoot,
    BootDelay,
    BootCancelled,
    AppUpgradeInProgress,
    ReadyToBoot,
};

/**
 * These fields are defined by the Brickproof Bootloader specification.
 */
struct __attribute__((packed)) AppInfo
{
    std::uint64_t image_crc = 0;
    std::uint32_t image_size = 0;
    std::uint32_t vcs_commit = 0;
    std::uint8_t major_version = 0;
    std::uint8_t minor_version = 0;
};

/**
 * This interface abstracts the target-specific ROM routines.
 * Upgrade scenario:
 *  1. beginUpgrade()
 *  2. write() repeated until finished.
 *  3. endUpgrade(success or not)
 */
class IAppStorageBackend
{
public:
    virtual ~IAppStorageBackend() { }

    /**
     * @return 0 on success, negative on error
     */
    virtual int beginUpgrade() = 0;

    /**
     * @return number of bytes written; negative on error
     */
    virtual int write(std::size_t offset, const void* data, std::size_t size) = 0;

    /**
     * @return 0 on success, negative on error
     */
    virtual int endUpgrade(bool success) = 0;

    /**
     * @return number of bytes read; negative on error
     */
    virtual int read(std::size_t offset, void* data, std::size_t size) = 0;
};

/**
 * Inherit this class to implement firmware loading protocol, from remote to the local storage.
 */
class IDownloadBehavior
{
public:
    virtual ~IDownloadBehavior() { }
};

/**
 * Main bootloader controller.
 */
class Bootloader
{
    State state_;
    IAppStorageBackend& backend_;

    const unsigned boot_delay_msec_;
    ::systime_t boot_delay_started_at_st_;

    chibios_rt::Mutex mutex_;

    /**
     * Refer to the Brickproof Bootloader specs.
     */
    struct __attribute__((packed)) AppDescriptor
    {
        std::array<std::uint8_t, 8> signature;
        AppInfo app_info;
        std::array<std::uint8_t, 6> reserved;

        static constexpr std::array<std::uint8_t, 8> getSignatureValue()
        {
            return {'A','P','D','e','s','c','0','0'};
        }

        bool isValid() const
        {
            const auto sgn = getSignatureValue();

            return
                std::equal(std::begin(signature), std::end(signature), std::begin(sgn)) &&
                (app_info.image_size > 0) &&
                (app_info.image_size < 0xFFFFFFFFU);
        }
    };
    static_assert(sizeof(AppDescriptor) == 32, "Invalid packing");

    std::pair<AppDescriptor, bool> locateAppDescriptor();

public:
    /**
     * Time since boot will be measured starting from the moment when the object was constructed.
     */
    Bootloader(IAppStorageBackend& backend, unsigned boot_delay_msec);

    /**
     * @ref State.
     */
    State getState();

    /**
     * Returns info about the application, if any.
     * @return First component is the application, second component is the status:
     *         true means that the info is valid, false means that there is no application to work with.
     */
    std::pair<AppInfo, bool> getAppInfo();

    /**
     * Switches the state to @ref BootCancelled, if allowed.
     */
    void cancelBoot();

    /**
     * Switches the state to @ref ReadyToBoot, if allowed.
     */
    void requestBoot();

    /**
     * Erases the application and loads a new one from the specified channel using YMODEM protocol.
     */
    void upgradeApp(IDownloadBehavior& downloader);
};

}
