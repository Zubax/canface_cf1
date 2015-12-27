/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cstdint>
#include <cstring>
#include <cassert>

/**
 * This implementation has been borrowed from libuavcan.
 */
namespace can
{
/**
 * Driver error codes.
 * These values can be returned from driver functions negated.
 */
//static const std::int16_t ErrUnknown               = 1000; ///< Reserved for future use
static const std::int16_t ErrNotImplemented          = 1001; ///< Feature not implemented
static const std::int16_t ErrInvalidBitRate          = 1002; ///< Bit rate not supported
static const std::int16_t ErrLogic                   = 1003; ///< Internal logic error
static const std::int16_t ErrUnsupportedFrame        = 1004; ///< Frame not supported (e.g. RTR, CAN FD, etc)
static const std::int16_t ErrMsrInakNotSet           = 1005; ///< INAK bit of the MSR register is not 1
static const std::int16_t ErrMsrInakNotCleared       = 1006; ///< INAK bit of the MSR register is not 0
static const std::int16_t ErrBitRateNotDetected      = 1007; ///< Auto bit rate detection could not be finished

/**
 * Frame definition like in libuavcan
 */
struct Frame
{
    static constexpr std::uint32_t MaskStdID = 0x000007FFU;
    static constexpr std::uint32_t MaskExtID = 0x1FFFFFFFU;
    static constexpr std::uint32_t FlagEFF = 1U << 31;          ///< Extended frame format
    static constexpr std::uint32_t FlagRTR = 1U << 30;          ///< Remote transmission request
    static constexpr std::uint32_t FlagERR = 1U << 29;          ///< Error frame

    static constexpr std::uint8_t MaxDataLen = 8;

    std::uint32_t id = 0;                                       ///< CAN ID with flags (above)
    std::uint8_t data[MaxDataLen] = {};
    std::uint8_t dlc = 0;                                       ///< Data Length Code

    Frame() { }

    Frame(std::uint32_t can_id, const void* can_data, std::uint8_t data_len) :
        id(can_id),
        dlc(data_len)
    {
        assert(can_data != nullptr);
        assert(data_len <= MaxDataLen);
        (void)std::memcpy(this->data, can_data, data_len);
    }

    bool isExtended()                  const { return id & FlagEFF; }
    bool isRemoteTransmissionRequest() const { return id & FlagRTR; }
    bool isErrorFrame()                const { return id & FlagERR; }

    /**
     * CAN frame arbitration rules, particularly STD vs EXT:
     *     Marco Di Natale - "Understanding and using the Controller Area Network"
     *     http://www6.in.tum.de/pub/Main/TeachingWs2013MSE/CANbus.pdf
     */
    bool priorityHigherThan(const Frame& rhs) const;
    bool priorityLowerThan(const Frame& rhs) const { return rhs.priorityHigherThan(*this); }
};

/**
 * RX frame data.
 */
struct RxFrame
{
    ::systime_t timestamp_systick = 0;
    Frame frame;
    std::uint8_t loopback : 1;
    std::uint8_t failed : 1;

    RxFrame() :
        loopback(false),
        failed(false)
    { }
};

static constexpr unsigned OptionSilentMode = 1;
static constexpr unsigned OptionLoopback   = 2;

/**
 * @param bitrate
 * @return negative on error
 */
int start(std::uint32_t bitrate, unsigned options = 0);

/**
 * Stops the controller.
 */
void stop();

/**
 * @param frame
 * @param timeout_ms
 * @retval 0 - timeout
 *         1 - frame successfully scheduled for transmission
 *         negative - error
 */
int send(const Frame& frame, std::uint16_t timeout_ms);

/**
 * @param out_frame
 * @param timeout_ms
 * @retval 0 - timeout
 *         1 - frame received
 *         negative - error
 */
int receive(RxFrame& out_frame, std::uint16_t timeout_ms);

}
