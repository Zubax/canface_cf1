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
#include <cassert>
#include <tuple>
#include <type_traits>
#include <cstring>
#include "util.hpp"


namespace bootloader
{
/**
 * Utilities that can be useful both in the bootloader and in the application.
 */
namespace app_shared
{
/**
 * Defines whether it is necessary to perform compile-time checks of the storage utilization.
 */
enum class StorageUtilizationCheckMode
{
    RequireFullStorageUtilization,//!< Abort if one or more blocks are unused
    AllowUnderutilizedStorage     //!< Ignore unused blocks
};

/**
 * This option allows to erase the structure automatically once it's read
 */
enum class AutoErase
{
    EraseAfterRead,
    DoNotErase
};

/**
 * Implementation details, do not use directly
 */
namespace impl_
{

template <
    typename Container,
    StorageUtilizationCheckMode StorageUtilizationCheck,
    typename Pointers
    >
class AppSharedMarshaller
{
    Pointers pointers_;

    struct ContainerWrapper
    {
        static constexpr unsigned CRCSize = 8;

        Container container;

    private:
        std::uint8_t crc_bytes[CRCSize] = {}; // We don't want to force any additional alignment, so use array of bytes

    public:
        ContainerWrapper() : container()  { }

        ContainerWrapper(const Container& c) :
            container(c)
        {
            CRC64WE crc_computer;
            crc_computer.add(&container, sizeof(container));
            const auto computed = crc_computer.get();
            std::memmove(&crc_bytes[0], &computed, CRCSize);
        }

        bool isCRCValid() const
        {
            CRC64WE crc_computer;
            crc_computer.add(&container, sizeof(container));
            const auto computed = crc_computer.get();
            return 0 == std::memcmp(&computed, &crc_bytes[0], CRCSize);
        }
    };

    //static_assert(std::is_pod<Container>::value, "Container must be a POD type");

    template <int N>
    struct IntegerAsType
    {
        static constexpr int Value = N;
    };

    template <unsigned A, unsigned B>
    struct ConstexprMin                                         // In GCC 4.8, std::min() is not yet constexpr.
    {
        static constexpr unsigned Result = (A < B) ? A : B;
    };

    template <unsigned MaxSize, typename T>                     // Holy pants why auto doesn't work here
    IntegerAsType<ConstexprMin<sizeof(T), MaxSize>::Result> readOne(void* destination, const volatile T* ptr)
    {
        const T x = *ptr;                                       // Guaranteeing proper pointer access
        std::memmove(destination, &x, ConstexprMin<sizeof(T), MaxSize>::Result);
        return {};
    }

    template <unsigned MaxSize>
    IntegerAsType<MaxSize> readOne(void* destination, const void* ptr)
    {
        std::memmove(destination, ptr, MaxSize);                // Raw memory access
        return {};
    }

    template <unsigned MaxSize, typename T>
    IntegerAsType<ConstexprMin<sizeof(T), MaxSize>::Result> writeOne(const void* source, volatile T* ptr)
    {
        T x = T();
        std::memmove(&x, source, ConstexprMin<sizeof(T), MaxSize>::Result);
        *ptr = x;                                               // Guaranteeing proper pointer access
        return {};
    }

    template <unsigned MaxSize>
    IntegerAsType<MaxSize> writeOne(const void* source, void* ptr)
    {
        std::memmove(ptr, source, MaxSize);                     // Raw memory access
        return {};
    }

    template <bool WriteNotRead, unsigned PtrIndex, unsigned RemainingSize>
    typename std::enable_if<(RemainingSize > 0)>::type unwindReadWrite(void* structure)
    {
        static_assert(PtrIndex < std::tuple_size<Pointers>::value, "Storage is not large enough for the structure");
        const auto ret = WriteNotRead ?
                         writeOne<RemainingSize>(structure, std::get<PtrIndex>(pointers_)) :
                         readOne<RemainingSize>(structure, std::get<PtrIndex>(pointers_));

        constexpr auto Increment = decltype(ret)::Value;
        static_assert(RemainingSize >= Increment, "Rock is dead");

        structure = static_cast<void*>(static_cast<std::uint8_t*>(structure) + Increment);
        unwindReadWrite<WriteNotRead, PtrIndex + 1U, RemainingSize - Increment>(structure);
    }

    template <bool, unsigned PtrIndex, unsigned RemainingSize>
    typename std::enable_if<(RemainingSize == 0)>::type unwindReadWrite(void*)
    {
        static_assert((StorageUtilizationCheck == StorageUtilizationCheckMode::RequireFullStorageUtilization) ?
                      PtrIndex == std::tuple_size<Pointers>::value :
                      true,
                      "Not all scattered storage blocks are used. "
                      "Disable this error by using option AllowUnderutilizedStorage.");
    }

public:
    AppSharedMarshaller(const Pointers& ptrs) : pointers_(ptrs) { }

    /**
     * Checks if the data is available and reads it.
     * @return A tuple of two items:
     *         First - the data structure
     *         Second - true if data exists, false if not. In the latter case the value of the first item is undefined.
     */
    std::pair<Container, bool> read(AutoErase auto_erase = AutoErase::DoNotErase)
    {
        ContainerWrapper wrapper;

        unwindReadWrite<false, 0, sizeof(wrapper)>(&wrapper);

        const bool valid = wrapper.isCRCValid();

        if (valid && (auto_erase == AutoErase::EraseAfterRead))
        {
            erase();
        }

        return {wrapper.container, valid};
    }

    /**
     * Writes the data. This function cannot fail.
     */
    void write(const Container& cont)
    {
        ContainerWrapper wrapper(cont);
        unwindReadWrite<true, 0, sizeof(wrapper)>(&wrapper);
    }

    /**
     * Invalidates the stored data.
     */
    void erase()
    {
        ContainerWrapper wrapper;
        std::memset(&wrapper, 0, sizeof(wrapper));
        unwindReadWrite<true, 0, sizeof(wrapper)>(&wrapper);
    }
};

} // namespace impl_

/**
 * Constructs an object that can be used to store and retrieve data for exchange with the application.
 *
 * Usage example:
 *
 *     auto marshaller = makeAppSharedMarshaller<DataType>(&REG_A, &REG_B, &REG_C, &REG_D, &REG_E, &REG_F);
 *     // Reading data:
 *     auto result = marshaller.read();
 *     if (result.second)
 *     {
 *         // Process the data...
 *     }
 *     else
 *     {
 *         // Data is not available (not stored)
 *     }
 *     // Writing data:
 *     marshaller.write(the_data);
 *     // Erasing data:
 *     marshaller.erase();
 *
 * @tparam Container                    Payload data type, i.e. a structure that should be stored or read.
 *
 * @tparam StorageUtilizationCheck      Defines whether compilation should be aborted if some of the pointers are not
 *                                      used, i.e. if the Container structure is smaller than the allocated storage.
 *                                      Refer to @ref StorageUtilizationCheckMode for info.
 *
 * @param pointers                      List of pointers to registers or memory where the structure will be stored or
 *                                      retrieved from. Pointer type defines access mode and size, e.g. a uint32
 *                                      pointer will be accessed in 32-bit mode, and its memory block will be used to
 *                                      store exactly 4 bytes, etc. Supported pointer sizes are 8, 16, 32, and 64 bit.
 *
 * @return                              An instance of @ref impl_::AppSharedMarshaller<>.
 *                                      The returned instance supports methods read(), write(), and erase(), that can
 *                                      be used to read, write, and erase the storage, respectively.
 */
template <
    typename Container,
    StorageUtilizationCheckMode StorageUtilizationCheck =
        StorageUtilizationCheckMode::AllowUnderutilizedStorage,
    typename... RegisterPointers
    >
auto makeAppSharedMarshaller(RegisterPointers... pointers)
{
    typedef impl_::AppSharedMarshaller<Container,
                                       StorageUtilizationCheck,
                                       decltype(std::make_tuple(pointers...))> Type;
    return Type(std::make_tuple(pointers...));
}

} // namespace app_shared
} // namespace bootloader
