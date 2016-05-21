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

#include "ymodem.hpp"
#include <ch.hpp>
#include <hal.h>
#include <numeric>
#include <unistd.h>

// Oh C, never change.
#ifdef CAN
#undef CAN
#endif

namespace bootloader
{
namespace ymodem_loader
{
namespace
{

struct ControlCharacters
{
    static constexpr std::uint8_t SOH = 0x01;
    static constexpr std::uint8_t STX = 0x02;
    static constexpr std::uint8_t EOT = 0x04;
    static constexpr std::uint8_t ACK = 0x06;
    static constexpr std::uint8_t NAK = 0x15;
    static constexpr std::uint8_t CAN = 0x18;
    static constexpr std::uint8_t C   = 0x43;
};

}

int YModemReceiver::sendResultToErrorCode(int res)
{
    if (res >= 0)
    {
        return -ErrChannelWriteTimedOut;
    }
    return res;
}

std::uint8_t YModemReceiver::computeChecksum(const void* data, unsigned size)
{
    auto p = static_cast<const std::uint8_t*>(data);
    return std::accumulate(p, p + size, 0);
}

int YModemReceiver::send(std::uint8_t byte)
{
    DEBUG_LOG("YMODEM TX 0x%x\n", byte);
    int res = chnPutTimeout(channel_, byte, MS2ST(SendTimeoutMSec));
    if (res != STM_OK)
    {
        if (res > 0)    // Making sure the error code is inverted
        {
            res = -res;
        }
        return res;
    }
    return 1;           // Number of bytes transferred, always 1
}

int YModemReceiver::receive(void* data, unsigned size, unsigned timeout_msec)
{
    /*
     * The spec says:
     *          Once into a receiving a block, the receiver goes into a one-second timeout
     *          for each character and the checksum.
     * This timeout logic is not compliant, as it imposes the same overall timeout for entire block. Who cares anyway.
     */
    return chnReadTimeout(channel_, static_cast<std::uint8_t*>(data), size, MS2ST(timeout_msec));
}

void YModemReceiver::flushReadQueue()
{
    for (;;)
    {
        const auto x = chnGetTimeout(channel_, MS2ST(1));
        if (x < 0)
        {
            break;
        }
        DEBUG_LOG("YMODEM FLUSH RX 0x%x\n", unsigned(x));
    }
}

void YModemReceiver::abort()
{
    constexpr auto Times = 5;           // Multiple CAN are required!
    for (int i = 0; i < Times; i++)
    {
        if (send(ControlCharacters::CAN) != 1)
        {
            break;
        }
    }
}

std::pair<YModemReceiver::BlockReceptionResult, int>
YModemReceiver::receiveBlock(unsigned& out_size, std::uint8_t& out_sequence)
{
    // Header byte
    std::uint8_t header_byte = 0;
    int res = receive(&header_byte, 1, NextBlockTimeoutMSec);
    if (res < 0)
    {
        return { BlockReceptionResult::SystemError, res };
    }
    if (res != 1)
    {
        return { BlockReceptionResult::Timeout, 0 };
    }

    switch (header_byte)
    {
    case ControlCharacters::STX:
    {
        out_size = BlockSize1K;
        break;
    }
    case ControlCharacters::SOH:
    {
        out_size = BlockSizeXModem;
        break;
    }
    case ControlCharacters::EOT:
    {
        DEBUG_LOG("YMODEM RX EOT\n");
        return { BlockReceptionResult::EndOfTransmission, 0 };
    }
    case ControlCharacters::CAN:
    {
        DEBUG_LOG("YMODEM RX CAN\n");
        return { BlockReceptionResult::TransmissionCancelled, 0 };
    }
    default:
    {
        DEBUG_LOG("YMODEM unexpected header 0x%x\n", header_byte);
        return { BlockReceptionResult::ProtocolError, 0 };
    }
    }

    // Sequence ID
    std::uint8_t sequence_id_bytes[2] = {};
    res = receive(sequence_id_bytes, 2, BlockPayloadTimeoutMSec);
    if (res < 0)
    {
        return { BlockReceptionResult::SystemError, res };
    }
    if (res != 2)
    {
        return { BlockReceptionResult::Timeout, 0 };
    }
    if (sequence_id_bytes[0] != static_cast<std::uint8_t>(~sequence_id_bytes[1]))       // Invalid sequence ID
    {
        DEBUG_LOG("YMODEM non-inverted sequence ID: 0x%x 0x%x\n", sequence_id_bytes[0], sequence_id_bytes[1]);
        return { BlockReceptionResult::ProtocolError, 0 };
    }
    out_sequence = sequence_id_bytes[0];

    // Payload
    constexpr auto ChecksumSize = 1;
    const auto block_size_with_checksum = out_size + ChecksumSize;
    res = receive(buffer_, block_size_with_checksum, BlockPayloadTimeoutMSec);
    if (res < 0)
    {
        return { BlockReceptionResult::SystemError, res };
    }
    if (unsigned(res) != block_size_with_checksum)
    {
        return { BlockReceptionResult::Timeout, 0 };
    }

    // Checksum validation
    if (computeChecksum(buffer_, out_size) != buffer_[out_size])
    {
        DEBUG_LOG("YMODEM checksum error, not %d\n", buffer_[out_size]);
        return { BlockReceptionResult::ProtocolError, 0 };
    }

    return { BlockReceptionResult::Success, 0 };
}

bool YModemReceiver::tryParseZeroBlock(const std::uint8_t* const data,
                                       const unsigned size,
                                       bool& out_is_null_block,
                                       std::uint32_t& out_file_size)
{
    assert(size == BlockSizeXModem || size == BlockSize1K);

    // Initializing defaults
    out_is_null_block = true;   // Paranoia
    out_file_size = 0;          // I.e. unknown

    unsigned offset = 0;

    // Skipping the file name
    while ((offset < size) && (data[offset] != 0))
    {
        offset++;
    }
    if (offset >= size)
    {
        return false;                                   // No null termination, invalid block
    }
    DEBUG_LOG("YMODEM file name: '%s'\n", reinterpret_cast<const char*>(data));

    // Setting the null block indication, aborting if it is null block because it won't contain file size
    out_is_null_block = offset == 0;                    // No filename means null block (end of session)
    if (out_is_null_block)
    {
        return true;
    }

    // Not a null block - parsing the file size
    offset++;
    DEBUG_LOG("YMODEM all fields: '%s'\n", reinterpret_cast<const char*>(&data[offset]));
    while ((offset < size) && (data[offset] != 0) && (data[offset] != ' '))
    {
        if (data[offset] < '0' ||
            data[offset] > '9')
        {
            out_file_size = 0;                          // Bad character before termination
            break;
        }
        out_file_size *= 10;
        out_file_size += std::uint32_t(data[offset] - std::uint8_t('0'));
        offset++;
    }
    DEBUG_LOG("YMODEM file size int: %u\n", unsigned(out_file_size));

    return true;
}

int YModemReceiver::processDownloadedBlock(IDownloadStreamSink& sink, void* data, unsigned size)
{
    DEBUG_LOG("YMODEM received block of %d bytes\n", size);
    return sink.handleNextDataChunk(data, size);
}

int YModemReceiver::download(IDownloadStreamSink& sink)
{
    // This thing will make sure there's no residual garbage in the RX buffer afterwards
    struct Flusher
    {
        YModemReceiver& parent;
        Flusher(YModemReceiver& x) : parent(x) { }
        ~Flusher() { parent.flushReadQueue(); }
    } flusher_(*this);

    // State variables
    std::uint32_t remaining_file_size = 0;
    bool file_size_known = false;
    std::uint8_t expected_sequence_id = 123;             // Arbitrary invalid value

    enum class Mode
    {
        XModem,
        YModem
    } mode = Mode::XModem;

    /*
     * Initiating the transfer, receiving the first block.
     * The sequence ID will be 0 in case of YMODEM, and 1 in case of XMODEM.
     */
    const auto started_at_st = chVTGetSystemTime();
    for (;;)
    {
        DEBUG_LOG("Trying to initiate X/YMODEM transfer...\n");

        // Abort if we couldn't get it going in InitialTimeoutMSec
        if (chVTTimeElapsedSinceX(started_at_st) > MS2ST(InitialTimeoutMSec))
        {
            abort();
            return -ErrRetriesExhausted;
        }

        // Requesting transmission in checksum mode
        int res = send(ControlCharacters::NAK);
        if (res != 1)
        {
            abort();
            return sendResultToErrorCode(res);
        }

        // Receiving the block
        unsigned size = 0;
        const auto block_rx_res = receiveBlock(size, expected_sequence_id);
        if (block_rx_res.first == BlockReceptionResult::Success)
        {
            ;
        }
        else if (block_rx_res.first == BlockReceptionResult::Timeout ||
                 block_rx_res.first == BlockReceptionResult::ProtocolError ||
                 block_rx_res.first == BlockReceptionResult::EndOfTransmission)
        {
            continue;   // EOT cannot be sent in response to the first block, it's an error; trying again...
        }
        else if (block_rx_res.first == BlockReceptionResult::TransmissionCancelled)
        {
            abort();
            return -ErrTransferCancelledByRemote;
        }
        else
        {
            assert(block_rx_res.first == BlockReceptionResult::SystemError);
            abort();
            return block_rx_res.second;
        }

        // Processing the block
        if (expected_sequence_id == 0)
        {
            mode = Mode::YModem;

            bool is_null_block = true;
            const bool zero_block_valid = tryParseZeroBlock(buffer_, size, is_null_block, remaining_file_size);

            DEBUG_LOG("YMODEM zero block: valid=%d null=%d size=%u\n",
                      zero_block_valid, is_null_block, unsigned(remaining_file_size));

            if (!zero_block_valid)
            {
                // Invalid zero block, that's a fatal error, it's checksum protected after all
                // Retrying here would make no sense, it's not a line hit, it's badly formed packet!
                abort();
                return -ErrProtocolError;
            }
            if (is_null_block)
            {
                // Null block means that the sender is refusing to transmit the file
                // No point retrying too, the sender isn't going to change their mind
                abort();
                return -ErrRemoteRefusedToProvideFile;
            }
            file_size_known = remaining_file_size > 0;

            // The zero block requires a dedicated ACK, sending it now
            res = send(ControlCharacters::ACK);
            if (res != 1)
            {
                abort();
                return sendResultToErrorCode(res);
            }
        }
        else if (expected_sequence_id == 1)
        {
            mode = Mode::XModem;
            DEBUG_LOG("YMODEM zero block skipped (XMODEM mode)\n");

            res = processDownloadedBlock(sink, buffer_, size);
            if (res < 0)
            {
                abort();
                return res;
            }
            file_size_known = false;
        }
        else                            // Invalid sequence number
        {
            abort();
            return -ErrProtocolError;
        }

        // Done!
        expected_sequence_id += 1;
        break;
    }

    assert(file_size_known ? true : (remaining_file_size == 0));

    /*
     * Receiving the file
     */
    bool ack = mode == Mode::XModem;    // YMODEM requires another NAK after the zero block
    unsigned remaining_retries = MaxRetries;
    for (;;)
    {
        // Limiting retries
        if (remaining_retries <= 0)
        {
            abort();
            return -ErrRetriesExhausted;
        }
        remaining_retries--;

        // Confirming or re-requesting
        int res = send(ack ? ControlCharacters::ACK : ControlCharacters::NAK);
        if (res != 1)
        {
            abort();
            return sendResultToErrorCode(res);
        }
        ack = false;

        // Receiving the block
        unsigned size = 0;
        std::uint8_t sequence_id = 0;
        const auto block_rx_res = receiveBlock(size, sequence_id);
        if (block_rx_res.first == BlockReceptionResult::Success)
        {
            ;
        }
        else if (block_rx_res.first == BlockReceptionResult::Timeout ||
                 block_rx_res.first == BlockReceptionResult::ProtocolError)
        {
            continue;
        }
        else if (block_rx_res.first == BlockReceptionResult::EndOfTransmission)
        {
            if ((file_size_known) && (remaining_file_size != 0))
            {
                // The sender said that we're done, liar!
                DEBUG_LOG("YMODEM ended %u bytes early\n", unsigned(remaining_file_size));
                abort();
                return -ErrProtocolError;
            }
            // Done, exiting and sending the final ACK
            DEBUG_LOG("YMODEM end OK\n");
            break;
        }
        else if (block_rx_res.first == BlockReceptionResult::TransmissionCancelled)
        {
            DEBUG_LOG("YMODEM cancelled\n");
            abort();
            return -ErrTransferCancelledByRemote;
        }
        else
        {
            assert(block_rx_res.first == BlockReceptionResult::SystemError);
            abort();
            return block_rx_res.second;
        }
        remaining_retries = MaxRetries;                         // Reset retries on successful reception

        // Processing the block
        if ((sequence_id + 1) == expected_sequence_id)          // Duplicate block, acknowledge silently
        {
            DEBUG_LOG("YMODEM duplicate block skipped\n");
            ack = true;
            continue;
        }
        if (sequence_id != expected_sequence_id)                // Totally wrong sequence, abort
        {
            DEBUG_LOG("YMODEM wrong sequence ID\n");
            abort();
            return -ErrProtocolError;
        }
        expected_sequence_id += 1;

        // Making sure we're not past the end of file
        if (file_size_known)
        {
            if (remaining_file_size == 0)
            {
                DEBUG_LOG("YMODEM transmission past the end of file\n");
                abort();
                return -ErrProtocolError;
            }
            if (size > remaining_file_size)
            {
                size = remaining_file_size;
            }
            remaining_file_size -= size;
        }

        // Sending the block over
        res = processDownloadedBlock(sink, buffer_, size);
        if (res < 0)
        {
            abort();
            return res;
        }

        // Done, continue to the next block
        ack = true;
    }

    /*
     * Final response and then leaving.
     * Errors can be ignored - we got what we wanted anyway.
     */
    DEBUG_LOG("YMODEM finalizing\n");

    (void)send(ControlCharacters::ACK);         // If it fails, who cares.

    if (mode == Mode::YModem)
    {
        // Letting the sender know we don't want any other files. Is this compliant?
        abort();
    }

    return ErrOK;
}

}
}
