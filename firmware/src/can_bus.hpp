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
static const std::int16_t ErrClosed                  = 1008; ///< The driver is not started

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

/**
 * CAN bus status info.
 */
struct Statistics
{
    std::uint64_t errors                  = 0;
    std::uint64_t bus_off_events          = 0;
    std::uint64_t sw_rx_queue_overruns    = 0;
    std::uint64_t hw_rx_queue_overruns    = 0;
    std::uint64_t frames_tx               = 0;
    std::uint64_t frames_rx               = 0;
    std::uint16_t tx_queue_capacity       = 0;
    std::uint16_t tx_queue_peak_usage     = 0;
    std::uint16_t rx_queue_capacity       = 0;
    std::uint16_t rx_queue_peak_usage     = 0;
    std::uint8_t tx_mailbox_peak_usage    = 0;
    std::uint8_t last_hw_error_code       = 0;
};

struct Status
{
    std::uint8_t receive_error_counter = 0;
    std::uint8_t transmit_error_counter = 0;    ///< Only 8 least significant bits (9th bit is not exposed by bxCAN)

    enum class State : std::uint8_t
    {
        ErrorActive,            ///< i.e. normal mode
        ErrorPassive,
        BusOff
    } state = State::ErrorActive;

    const char* getStateAsString() const
    {
        switch (state)
        {
        case State::ErrorActive:  return "error_active";
        case State::ErrorPassive: return "error_passive";
        case State::BusOff:       return "bus_off";
        default:
        {
            assert(false);
            return "???";
        }
        }
    }
};

/**
 * Open options
 */
static constexpr unsigned OptionSilentMode = 1;
static constexpr unsigned OptionLoopback   = 2;

/**
 * Opens the channel and resets all associated statistics.
 * @param bitrate
 * @return negative on error
 */
int open(std::uint32_t bitrate, unsigned options = 0);

/**
 * Closes the channel.
 * Note that this call does not reset the statistics; @ref open() does.
 */
void close();

/**
 * True if open. While not open, other API functions may not be available.
 */
bool isOpen();

/**
 * It is safe to call @ref send() and @ref receive() concurrently from different threads.
 * @param frame
 * @param timeout_ms
 * @retval 0 - timeout
 *         1 - frame successfully scheduled for transmission
 *         negative - error
 */
int send(const Frame& frame, std::uint16_t timeout_ms);

/**
 * It is safe to call @ref send() and @ref receive() concurrently from different threads.
 * @param out_frame
 * @param timeout_ms
 * @retval 0 - timeout
 *         1 - frame received
 *         negative - error
 */
int receive(RxFrame& out_frame, std::uint16_t timeout_ms);

/**
 * Returns the statistics collected since the last @ref open() call.
 * Note that the statistics object is large, and access to it is protected by a critical section,
 * so accessing this function may introduce a few microsecond latency to communications.
 */
Statistics getStatistics();

/**
 * Returns current state of the CAN controller.
 * If the channel is not open, this function may return garbage.
 */
Status getStatus();

/**
 * Returns true if there were CAN exchange since the previous call to this function.
 */
bool hadActivity();

}
