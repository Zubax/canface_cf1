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

#include "../bootloader.hpp"
#include <zubax_chibios/os.hpp>
#include <cstdint>
#include <array>
#include <utility>


namespace bootloader
{
namespace ymodem_loader
{
/**
 * Downloads data using YMODEM or XMODEM protocol over the specified ChibiOS channel
 * (e.g. serial port, USB CDC ACM, TCP, ...).
 *
 * This class will request Checksum mode, in order to retain compatibility both with XMODEM and YMODEM senders
 * (YMODEM-compatible senders do support checksum mode as well as CRC mode).
 * Both 1K and 128-byte blocks are supported.
 * Overall, the following protocols are supported:
 *      - YMODEM
 *      - XMODEM
 *      - XMODEM-1K
 *
 * Reference: http://pauillac.inria.fr/~doligez/zmodem/ymodem.txt
 */
class YModemReceiver : public IDownloader
{
    static constexpr unsigned BlockSizeXModem = 128;
    static constexpr unsigned BlockSize1K     = 1024;
    static constexpr unsigned WorstCaseBlockSizeWithCRC = BlockSize1K + 2;

    static constexpr unsigned SendTimeoutMSec = 1000;

    static constexpr unsigned InitialTimeoutMSec      = 60000;
    static constexpr unsigned NextBlockTimeoutMSec    = 10000;
    static constexpr unsigned BlockPayloadTimeoutMSec = 1000;

    ::BaseChannel* const channel_;

    std::uint8_t buffer_[WorstCaseBlockSizeWithCRC];

    static int ioResultToErrorCode(int res);

    static std::uint8_t computeChecksum(const void* data, unsigned size);

    int send(std::uint8_t byte);

    int receive(void* data, unsigned size, unsigned timeout_msec);

    void flushReadQueue();

    void abort();

    enum class BlockReceptionResult
    {
        Success,
        Timeout,
        EndOfTransmission,
        TransmissionCancelled,
        ProtocolError,
        SystemError
    };

    /**
     * Reads a block from the channel. This function does not transmit anything.
     * @return First component: @ref BlockReceptionResult
     *         Second component: system error code, if applicable
     */
    std::pair<BlockReceptionResult, int> receiveBlock(unsigned& out_size,
                                                      std::uint8_t& out_sequence);

    static bool tryParseZeroBlock(const std::uint8_t* const data,
                                  const unsigned size,
                                  bool& out_is_null_block,
                                  std::uint32_t& out_file_size);

    static int processDownloadedBlock(IDownloadStreamSink& sink, void* data, unsigned size);

public:
    YModemReceiver(::BaseChannel* channel) :
        channel_(channel)
    { }

    int download(IDownloadStreamSink& sink) override;
};

}
}
