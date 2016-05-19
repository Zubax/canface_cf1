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

#include "bootloader.hpp"
#include <array>
#include <cassert>


namespace bootloader
{
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

/**
 * This is used to verify integrity of the application.
 * CRC-64-WE
 * Description: http://reveng.sourceforge.net/crc-catalogue/17plus.htm#crc.cat-bits.64
 * Initial value: 0xFFFFFFFFFFFFFFFF
 * Poly: 0x42F0E1EBA9EA3693
 * Reverse: no
 * Output xor: 0xFFFFFFFFFFFFFFFF
 * Check: 0x62EC59E3F1A4F00A
 */
class CRC64WE
{
    std::uint64_t crc_;

public:
    CRC64WE() : crc_(0xFFFFFFFFFFFFFFFFULL) { }

    void add(std::uint8_t byte)
    {
        static constexpr std::uint64_t Poly = 0x42F0E1EBA9EA3693;
        crc_ ^= std::uint64_t(byte) << 56;
        for (int i = 0; i < 8; i++)
        {
            crc_ = (crc_ & (std::uint64_t(1) << 63)) ? (crc_ << 1) ^ Poly : crc_ << 1;
        }
    }

    void add(const void* data, unsigned len)
    {
        auto bytes = static_cast<const std::uint8_t*>(data);
        assert(bytes != nullptr);
        while (len --> 0)
        {
            add(*bytes++);
        }
    }

    std::uint64_t get() const { return crc_ ^ 0xFFFFFFFFFFFFFFFFULL; }
};


namespace
{

State g_state = State::NoAppToBoot;

IAppStorageBackend* g_backend = nullptr;

unsigned g_boot_delay_msec = 0;

::systime_t g_boot_delay_started_at_st = 0;

chibios_rt::Mutex g_mutex;

std::pair<AppDescriptor, bool> locateAppDescriptor()
{
    if (g_backend == nullptr)
    {
        assert(false);
        return {AppDescriptor(), false};
    }

    constexpr auto Step = 8;

    for (std::size_t offset = 0;; offset += Step)
    {
        // Reading the storage in 8 bytes increments until we've found the signature
        {
            std::uint8_t signature[Step] = {};
            int res = g_backend->read(offset, signature, sizeof(signature));
            if (res != sizeof(signature))
            {
                break;
            }
            const auto reference = AppDescriptor::getSignatureValue();
            if (!std::equal(std::begin(signature), std::end(signature), std::begin(reference)))
            {
                continue;
            }
        }

        // Reading the entire descriptor
        AppDescriptor desc;
        {
            int res = g_backend->read(offset, &desc, sizeof(desc));
            if (res != sizeof(desc))
            {
                break;
            }
            if (!desc.isValid())
            {
                continue;
            }
        }

        // Checking firmware CRC
        {
            constexpr auto WordSize = 4;
            const auto crc_offset_in_words = (offset + offsetof(AppDescriptor, app_info.image_crc)) / WordSize;
            const auto image_size_in_words = desc.app_info.image_size / WordSize;

            CRC64WE crc;

            for (unsigned i = 0; i < image_size_in_words; i++)
            {
                std::uint32_t word = 0;
                if ((i != crc_offset_in_words) && (i != (crc_offset_in_words + 1)))
                {
                    int res = g_backend->read(i * WordSize, &word, WordSize);
                    if (res != WordSize)
                    {
                        continue;
                    }
                }

                crc.add(&word, WordSize);
            }

            if (crc.get() != desc.app_info.image_crc)
            {
                DEBUG_LOG("App descriptor found, but CRC is invalid (%s != %s)\n",
                          os::uintToString(crc.get()).c_str(),
                          os::uintToString(desc.app_info.image_crc).c_str());
                continue;       // Look further...
            }
        }

        // Returning if the descriptor is correct
        DEBUG_LOG("App descriptor located at offset %x\n", unsigned(offset));
        return {desc, true};
    }

    return {AppDescriptor(), false};
}

} // namespace

void init(IAppStorageBackend* backend, unsigned boot_delay_msec)
{
    os::MutexLocker mlock(g_mutex);

    assert(g_backend == nullptr);
    g_backend = backend;
    g_boot_delay_msec = boot_delay_msec;
    g_boot_delay_started_at_st = chVTGetSystemTime();

    /*
     * Inspecting the application storage looking for the descriptor
     */
    const auto appdesc_result = locateAppDescriptor();
    if (appdesc_result.second)
    {
        DEBUG_LOG("App found; version %d.%d.%x, %d bytes\n",
                  appdesc_result.first.app_info.major_version,
                  appdesc_result.first.app_info.minor_version,
                  unsigned(appdesc_result.first.app_info.vcs_commit),
                  unsigned(appdesc_result.first.app_info.image_size));
        g_state = State::BootDelay;
    }
    else
    {
        DEBUG_LOG("App not found\n");
        g_state = State::NoAppToBoot;
    }
}

State getState()
{
    os::MutexLocker mlock(g_mutex);

    if ((g_state == State::BootDelay) &&
        (chVTTimeElapsedSinceX(g_boot_delay_started_at_st) >= MS2ST(g_boot_delay_msec)))
    {
        DEBUG_LOG("Boot delay expired\n");
        g_state = State::ReadyToBoot;
    }

    return g_state;
}

std::pair<AppInfo, bool> getAppInfo()
{
    os::MutexLocker mlock(g_mutex);
    const auto res = locateAppDescriptor();
    return {res.first.app_info, res.second};
}

void cancelBoot()
{
    os::MutexLocker mlock(g_mutex);

    switch (g_state)
    {
        case State::BootDelay:
        case State::ReadyToBoot:
        {
            g_state = State::BootCancelled;
            DEBUG_LOG("Boot cancelled\n");
            break;
        }
        case State::NoAppToBoot:
        case State::BootCancelled:
        case State::AppUpgradeInProgress:
        {
            break;
        }
    }
}

void requestBoot()
{
    os::MutexLocker mlock(g_mutex);

    switch (g_state)
    {
        case State::BootDelay:
        case State::BootCancelled:
        {
            g_state = State::ReadyToBoot;
            DEBUG_LOG("Boot requested\n");
            break;
        }
        case State::NoAppToBoot:
        case State::AppUpgradeInProgress:
        case State::ReadyToBoot:
        {
            break;
        }
    }
}

void upgradeAppViaYModem(::BaseChannel* channel)
{
    os::MutexLocker mlock(g_mutex);

    (void)channel;
}

}
