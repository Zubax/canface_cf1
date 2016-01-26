/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <zubax_chibios/os.hpp>
#include <hal.h>
#include <unistd.h>
#include <type_traits>
#include <algorithm>
#include <new>
#include "can_bus.hpp"


#ifndef CAN_IRQ_TRACE
# define CAN_IRQ_TRACE  0
#endif
#if CAN_IRQ_TRACE
# define CAN_IRQ_TRACE_BEGIN(pin)       palSetPad(GPIOA, GPIOA_PIN_##pin)
# define CAN_IRQ_TRACE_END(pin)       palClearPad(GPIOA, GPIOA_PIN_##pin)
#else
# define CAN_IRQ_TRACE_BEGIN(pin)       ((void)0)
# define CAN_IRQ_TRACE_END(pin)         ((void)0)
#endif


#define CAN_GPT         GPTD5


namespace can
{
namespace
{

constexpr unsigned IRQPriority = CORTEX_MAX_KERNEL_PRIORITY;
constexpr unsigned NumTxMailboxes = 3;

class RxQueue
{
    typedef std::uint8_t Size;
    static constexpr Size Capacity = 255;

    RxFrame buf_[Capacity];
    Size in_ = 0;
    Size out_ = 0;
    Size len_ = 0;
    Size peak_len_ = 0;

public:
    /**
     * @retval true - OK, false - Overflow
     */
    bool push(const RxFrame& frame)
    {
        buf_[in_] = frame;
        in_++;
        if UNLIKELY(in_ >= Capacity)
        {
            in_ = 0;
        }
        if UNLIKELY(len_ >= Capacity)
        {
            len_ = Capacity;
            out_++;
            if UNLIKELY(out_ >= Capacity)
            {
                out_ = 0;
            }
            return false;
        }
        len_++;
        if UNLIKELY(peak_len_ < len_)
        {
            peak_len_ = len_;
        }
        return true;
    }

    void pop(RxFrame& out_frame)
    {
        if LIKELY(len_ > 0)
        {
            out_frame = buf_[out_];
            out_++;
            if UNLIKELY(out_ >= Capacity)
            {
                out_ = 0;
            }
            len_--;
        }
        else { assert(0); }
    }

    void reset()
    {
        in_ = 0;
        out_ = 0;
        len_ = 0;
    }

    Size getLength() const    { return len_; }
    Size getPeakUsage() const { return peak_len_; }
    Size getCapacity() const  { return Capacity; }
};

/**
 * Frame makeFrame(unsigned id)
 * {
 *     static unsigned count = 0;
 *     const auto s = std::to_string(count++);
 *     return Frame(id, s.c_str(), s.length());
 * }
 * void flush(TxQueue& q)
 * {
 *     std::cout << "Peak usage: " << q.getPeakUsage() << std::endl;
 *     while (q.peek())
 *     {
 *         std::cout << q.peek()->toString() << std::endl;
 *         q.pop();
 *     }
 * }
 * int main()
 * {
 *     TxQueue ptxq;
 *     ENFORCE(ptxq.getPeakUsage() == 0);
 *     ENFORCE(ptxq.getCapacity() == 30);
 *     ENFORCE(ptxq.peek() == nullptr);
 *     ptxq.push(makeFrame(5));
 *     ptxq.push(makeFrame(3));
 *     ptxq.push(makeFrame(4));
 *     ptxq.push(makeFrame(4));
 *     ptxq.push(makeFrame(2));
 *     ptxq.push(makeFrame(4));
 *     flush(ptxq);
 * }
 */
class TxQueue
{
    static constexpr unsigned Capacity = 100;

    struct TxFrame
    {
        TxFrame* next = nullptr;
        Frame frame;

        TxFrame(const Frame& f) : frame(f) { }
    };

    class Allocator
    {
        union Node
        {
            alignas(TxFrame) std::uint8_t data[sizeof(TxFrame)];
            Node* next;
        };

        alignas(Node) std::uint8_t pool_[Capacity * sizeof(TxFrame)];
        Node* free_list_;

        unsigned used_ = 0;
        unsigned max_used_ = 0;

    public:
        Allocator() :
            free_list_(reinterpret_cast<Node*>(pool_))
        {
            (void)std::fill_n(pool_, sizeof(pool_), 0);
            for (unsigned i = 0; (i + 1) < (Capacity - 1 + 1); i++) // -Werror=type-limits
            {
                // coverity[dead_error_line : FALSE]
                free_list_[i].next = free_list_ + i + 1;
            }
            free_list_[Capacity - 1].next = nullptr;
        }

        template <typename... Args>
        TxFrame* allocate(Args... args)
        {
            if UNLIKELY(free_list_ == nullptr)
            {
                return nullptr;
            }

            void* const pmem = free_list_;
            free_list_ = free_list_->next;

            // Statistics
            assert(used_ < Capacity);
            used_++;
            if UNLIKELY(used_ > max_used_)
            {
                max_used_ = used_;
            }

            return new (pmem) TxFrame(args...);
        }

        void deallocate(void* ptr)
        {
            if UNLIKELY(ptr == nullptr)
            {
                assert(false);
                return;
            }

            auto p = static_cast<Node*>(ptr);
            p->next = free_list_;
            free_list_ = p;

            // Statistics
            assert(used_ > 0);
            used_--;
        }

        unsigned getPeakNumUsedBlocks() const { return max_used_; }
    };

    Allocator allocator_;
    TxFrame* head_ = nullptr;

public:
    bool push(const Frame& frame)
    {
        auto* const txf = allocator_.allocate(frame);
        if UNLIKELY(txf == nullptr)
        {
            return false;
        }

        if LIKELY(head_ == nullptr || frame.priorityHigherThan(head_->frame))
        {
            txf->next = head_;
            head_ = txf;
        }
        else
        {
            auto* p = head_;
            while (p->next != nullptr)
            {
                if UNLIKELY(frame.priorityHigherThan(p->next->frame))
                {
                    break;
                }
                p = p->next;
            }
            txf->next = p->next;
            p->next = txf;
        }

        return true;
    }

    void pop()
    {
        if LIKELY(head_ != nullptr)
        {
            auto next = head_->next;
            head_->~TxFrame();
            allocator_.deallocate(head_);
            head_ = next;
        }
        else
        {
            assert(false);
        }
    }

    const Frame* peek() const
    {
        return (head_ == nullptr) ? nullptr : &head_->frame;
    }

    unsigned getPeakUsage() const { return allocator_.getPeakNumUsedBlocks(); }
    unsigned getCapacity() const { return Capacity; }
};


struct Timings
{
    std::uint16_t prescaler = 0;
    std::uint8_t sjw = 0;
    std::uint8_t bs1 = 0;
    std::uint8_t bs2 = 0;
};


struct TxItem
{
    Frame frame;
    bool pending = false;
};

/*
 * Internal functions
 */
int computeTimings(const std::uint32_t target_bitrate, Timings& out_timings)
{
    if (target_bitrate < 1)
    {
        return -ErrInvalidBitRate;
    }

    /*
     * Hardware configuration
     */
    const std::uint32_t pclk = STM32_PCLK1;

    static const int MaxBS1 = 16;
    static const int MaxBS2 = 8;

    /*
     * Ref. "Automatic Baudrate Detection in CANopen Networks", U. Koppe, MicroControl GmbH & Co. KG
     *      CAN in Automation, 2003
     *
     * According to the source, optimal quanta per bit are:
     *   Bitrate        Optimal Maximum
     *   1000 kbps      8       10
     *   500  kbps      16      17
     *   250  kbps      16      17
     *   125  kbps      16      17
     */
    const int max_quanta_per_bit = (target_bitrate >= 1000000) ? 10 : 17;

    assert(max_quanta_per_bit <= (MaxBS1 + MaxBS2));

    static const int MaxSamplePointLocation = 900;

    /*
     * Computing (prescaler * BS):
     *   BITRATE = 1 / (PRESCALER * (1 / PCLK) * (1 + BS1 + BS2))       -- See the Reference Manual
     *   BITRATE = PCLK / (PRESCALER * (1 + BS1 + BS2))                 -- Simplified
     * let:
     *   BS = 1 + BS1 + BS2                                             -- Number of time quanta per bit
     *   PRESCALER_BS = PRESCALER * BS
     * ==>
     *   PRESCALER_BS = PCLK / BITRATE
     */
    const std::uint32_t prescaler_bs = pclk / target_bitrate;

    /*
     * Searching for such prescaler value so that the number of quanta per bit is highest.
     */
    std::uint8_t bs1_bs2_sum = max_quanta_per_bit - 1;

    while ((prescaler_bs % (1 + bs1_bs2_sum)) != 0)
    {
        if (bs1_bs2_sum <= 2)
        {
            return -ErrInvalidBitRate;          // No solution
        }
        bs1_bs2_sum--;
    }

    const std::uint32_t prescaler = prescaler_bs / (1 + bs1_bs2_sum);
    if ((prescaler < 1U) || (prescaler > 1024U))
    {
        return -ErrInvalidBitRate;              // No solution
    }

    /*
     * Now we have a constraint: (BS1 + BS2) == bs1_bs2_sum.
     * We need to find the values so that the sample point is as close as possible to the optimal value.
     *
     *   Solve[(1 + bs1)/(1 + bs1 + bs2) == 7/8, bs2]  (* Where 7/8 is 0.875, the recommended sample point location *)
     *   {{bs2 -> (1 + bs1)/7}}
     *
     * Hence:
     *   bs2 = (1 + bs1) / 7
     *   bs1 = (7 * bs1_bs2_sum - 1) / 8
     *
     * Sample point location can be computed as follows:
     *   Sample point location = (1 + bs1) / (1 + bs1 + bs2)
     *
     * Since the optimal solution is so close to the maximum, we prepare two solutions, and then pick the best one:
     *   - With rounding to nearest
     *   - With rounding to zero
     */
    struct BsPair
    {
        std::uint8_t bs1;
        std::uint8_t bs2;
        std::uint16_t sample_point_permill;

        BsPair() :
            bs1(0),
            bs2(0),
            sample_point_permill(0)
        { }

        BsPair(std::uint8_t bs1_bs2_sum, std::uint8_t arg_bs1) :
            bs1(arg_bs1),
            bs2(bs1_bs2_sum - bs1),
            sample_point_permill(1000 * (1 + bs1) / (1 + bs1 + bs2))
        {
            assert(bs1_bs2_sum > arg_bs1);
        }

        bool isValid() const { return (bs1 >= 1) && (bs1 <= MaxBS1) && (bs2 >= 1) && (bs2 <= MaxBS2); }
    };

    BsPair solution(bs1_bs2_sum, ((7 * bs1_bs2_sum - 1) + 4) / 8);      // First attempt with rounding to nearest

    if (solution.sample_point_permill > MaxSamplePointLocation)
    {
        solution = BsPair(bs1_bs2_sum, (7 * bs1_bs2_sum - 1) / 8);      // Second attempt with rounding to zero
    }

    /*
     * Final validation
     * Helpful Python:
     * def sample_point_from_btr(x):
     *     assert 0b0011110010000000111111000000000 & x == 0
     *     ts2,ts1,brp = (x>>20)&7, (x>>16)&15, x&511
     *     return (1+ts1+1)/(1+ts1+1+ts2+1)
     *
     */
    if ((target_bitrate != (pclk / (prescaler * (1 + solution.bs1 + solution.bs2)))) || !solution.isValid())
    {
        assert(0);
        return -ErrLogic;
    }

    DEBUG_LOG("Timings: quanta/bit: %d, sample point location: %.1f%%\n",
              int(1 + solution.bs1 + solution.bs2), float(solution.sample_point_permill) / 10.F);

    out_timings.prescaler = std::uint16_t(prescaler - 1U);
    out_timings.sjw = 0;                                        // Which means one
    out_timings.bs1 = std::uint8_t(solution.bs1 - 1);
    out_timings.bs2 = std::uint8_t(solution.bs2 - 1);
    return 0;
}

bool waitMSRINAKBitStateChange(bool target_state)
{
    constexpr unsigned Timeout = 1000;
    for (unsigned wait_ack = 0; wait_ack < Timeout; wait_ack++)
    {
        const bool state = (CAN->MSR & CAN_MSR_INAK) != 0;
        if (state == target_state)
        {
            return true;
        }
        ::usleep(1000);
    }
    return false;
}

/**
 * TODO: use a different synchronization primitive, not semaphore
 */
class Event
{
    chibios_rt::CounterSemaphore sem_;

public:
    Event() : sem_(0) { }

    void waitForSysTicks(unsigned systicks) { (void)sem_.wait(systicks); }

    void signalI() { sem_.signalI(); }
};


/*
 * Statistics are kept even after the interface is closed
 */
Statistics statistics_;

/*
 * Driver state
 */
struct DriverState
{
    RxQueue rx_queue;
    TxQueue tx_queue;
    Event rx_event;
    Event tx_event;
    TxItem pending_tx[NumTxMailboxes];
    bool had_activity = false;

    const bool loopback;

    DriverState(bool option_loopback) :
        loopback(option_loopback)
    { }

    ~DriverState()
    {
        // Making sure the latest statistics will be preserved after the state is destroyed.
        updateStatistics();
    }

    void pushRxFromISR(const RxFrame& rxf)
    {
        os::CriticalSectionLocker cs_locker;

        if (!rx_queue.push(rxf))
        {
            statistics_.sw_rx_queue_overruns++;
        }
        rx_event.signalI();

        if (!rxf.loopback && !rxf.failed)
        {
            had_activity = true;
            statistics_.frames_rx++;
        }
    }

    /// FIXME This is ugly but I don't have a better idea at the moment.
    void updateStatistics() const
    {
        statistics_.tx_queue_capacity   = tx_queue.getCapacity();
        statistics_.tx_queue_peak_usage = tx_queue.getPeakUsage();
        statistics_.rx_queue_capacity   = rx_queue.getCapacity();
        statistics_.rx_queue_peak_usage = rx_queue.getPeakUsage();
    }
};


chibios_rt::Mutex rx_mutex_;
chibios_rt::Mutex tx_mutex_;
DriverState* state_ = nullptr;


class CommonMutexLocker
{
    os::MutexLocker rx_;
    os::MutexLocker tx_;
public:
    CommonMutexLocker() :
        rx_(rx_mutex_),
        tx_(tx_mutex_)
    { }
};

/*
 * TX management helpers
 */
/// Must be invoked from ISR or Critical Section
inline bool canAcceptNewTxFrameCS(const Frame& frame)
{
    /*
     * We can accept more frames only if the following conditions are satisfied:
     *  - There is at least one TX mailbox free (obvious enough);
     *  - The priority of the new frame is higher than priority of all TX mailboxes.
     */
    {
        static constexpr std::uint32_t TME = CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2;
        const std::uint32_t tme = CAN->TSR & TME;

        if LIKELY(tme == TME)   // All TX mailboxes are free (as in freedom).
        {
            return true;
        }

        if UNLIKELY(tme == 0)   // All TX mailboxes are busy transmitting.
        {
            return false;
        }
    }

    for (unsigned mbx = 0; mbx < NumTxMailboxes; mbx++)
    {
        if (state_->pending_tx[mbx].pending && !frame.priorityHigherThan(state_->pending_tx[mbx].frame))
        {
            return false;       // There's a mailbox whose priority is higher or equal the priority of the new frame.
        }
    }

    return true;                // This new frame will be added to a free TX mailbox in the next @ref send().
}

/// Must be invoked from ISR or Critical Section
inline void loadTxMailboxCS(const Frame& frame)
{
    /*
     * Seeking for an empty slot
     */
    std::uint8_t txmailbox = 0xFF;
    if ((CAN->TSR & CAN_TSR_TME0) == CAN_TSR_TME0)
    {
        txmailbox = 0;
    }
    else if ((CAN->TSR & CAN_TSR_TME1) == CAN_TSR_TME1)
    {
        txmailbox = 1;
    }
    else if ((CAN->TSR & CAN_TSR_TME2) == CAN_TSR_TME2)
    {
        txmailbox = 2;
    }
    else
    {
        assert(false);
        return;         // No transmission for you.
    }

    statistics_.tx_mailbox_peak_usage = std::max(statistics_.tx_mailbox_peak_usage, txmailbox); // Statistics

    /*
     * Setting up the mailbox
     */
    auto& mb = CAN->sTxMailBox[txmailbox];
    if (frame.isExtended())
    {
        mb.TIR = ((frame.id & Frame::MaskExtID) << 3) | CAN_TI0R_IDE;
    }
    else
    {
        mb.TIR = ((frame.id & Frame::MaskStdID) << 21);
    }

    if (frame.isRemoteTransmissionRequest())
    {
        mb.TIR |= CAN_TI0R_RTR;
    }

    mb.TDTR = frame.dlc;

    mb.TDHR = (std::uint32_t(frame.data[7]) << 24) |
              (std::uint32_t(frame.data[6]) << 16) |
              (std::uint32_t(frame.data[5]) << 8)  |
              (std::uint32_t(frame.data[4]) << 0);
    mb.TDLR = (std::uint32_t(frame.data[3]) << 24) |
              (std::uint32_t(frame.data[2]) << 16) |
              (std::uint32_t(frame.data[1]) << 8)  |
              (std::uint32_t(frame.data[0]) << 0);

    mb.TIR |= CAN_TI0R_TXRQ;  // Go. Transmission starts here.

    /*
     * Registering the pending transmission
     */
    auto& txi = state_->pending_tx[txmailbox];
    txi.frame   = frame;
    txi.pending = true;
}

/// This function is invoked at the end of every IRQ handler
__attribute__((always_inline))
inline void endOfInterruptHandlerHook()
{
    /*
     * We count errors here, piggybacking on other IRQs instead of enabling interrupt generation on LEC change
     * (see flag CAN_IER_LECIE). This is because in certain states the macrocell can generate much more LEC change
     * interrupts than the MCU can handle (e.g. if the bus wires are shorted, the controller may trigger an
     * IRQ every few microseconds), which essentially halts all other functions of the application and eventually
     * can lead to a watchdog timeout.
     * Since LEC may be changed multiple times between invocations of this function, we may fail to count some errors,
     * but this is acceptable, and there is no better way anyway.
     */
    if UNLIKELY((CAN->ESR & CAN_ESR_LEC) != 0)
    {
        statistics_.errors++;
        CAN->ESR = 0;                   // Reset error code in order to not count this error twice
    }
}

/*
 * Interrupt handlers
 */
inline void handleTxMailboxInterrupt(const std::uint8_t mailbox_index, const bool txok,
                                     const std::uint32_t timestamp_usec)
{
    assert(mailbox_index < NumTxMailboxes);

    /*
     * Updating statistics
     */
    if (txok)
    {
        state_->had_activity = true;
        statistics_.frames_tx++;
    }

    /*
     * Updating the state and reporting the loopback event
     */
    {
        auto& txi = state_->pending_tx[mailbox_index];

        if (state_->loopback && txi.pending)
        {
            RxFrame rxf;
            rxf.frame           = txi.frame;
            rxf.loopback        = true;
            rxf.failed          = !txok;
            rxf.timestamp_usec  = timestamp_usec;

            state_->pushRxFromISR(rxf);
        }

        txi.pending = false;
    }

    /*
     * Handling other frames scheduled for transmission
     */
    if (const auto top = state_->tx_queue.peek())
    {
        if (canAcceptNewTxFrameCS(*top))
        {
            loadTxMailboxCS(*top);
            state_->tx_queue.pop();
        }
    }

    /*
     * Reporting a TX event
     */
    {
        os::CriticalSectionLocker cs_locker;
        state_->tx_event.signalI();
    }

    endOfInterruptHandlerHook();
}

inline void handleRxInterrupt(const std::uint8_t fifo_index, const std::uint32_t timestamp_usec)
{
    static constexpr unsigned CAN_RFR_FMP_MASK = 3;

    assert(fifo_index < 2);

    volatile std::uint32_t& rfr_reg = (fifo_index == 0) ? CAN->RF0R : CAN->RF1R;
    if ((rfr_reg & CAN_RFR_FMP_MASK) == 0)
    {
        assert(0);  // Weird, IRQ is here but no data to read
        return;
    }

    /*
     * Register overflow
     */
    if ((rfr_reg & CAN_RF0R_FOVR0) != 0)
    {
        statistics_.hw_rx_queue_overruns++;
    }

    /*
     * Read the frame contents
     */
    RxFrame rxf;
    rxf.timestamp_usec = timestamp_usec;

    const auto& rf = CAN->sFIFOMailBox[fifo_index];

    if ((rf.RIR & CAN_RI0R_IDE) == 0)
    {
        rxf.frame.id = Frame::MaskStdID & (rf.RIR >> 21);
    }
    else
    {
        rxf.frame.id = Frame::MaskExtID & (rf.RIR >> 3);
        rxf.frame.id |= Frame::FlagEFF;
    }

    if ((rf.RIR & CAN_RI0R_RTR) != 0)
    {
        rxf.frame.id |= Frame::FlagRTR;
    }

    rxf.frame.dlc = rf.RDTR & 15;

    {
        const std::uint32_t r = rf.RDLR;
        rxf.frame.data[0] = std::uint8_t(0xFF & (r >> 0));
        rxf.frame.data[1] = std::uint8_t(0xFF & (r >> 8));
        rxf.frame.data[2] = std::uint8_t(0xFF & (r >> 16));
        rxf.frame.data[3] = std::uint8_t(0xFF & (r >> 24));
    }
    {
        const std::uint32_t r = rf.RDHR;
        rxf.frame.data[4] = std::uint8_t(0xFF & (r >> 0));
        rxf.frame.data[5] = std::uint8_t(0xFF & (r >> 8));
        rxf.frame.data[6] = std::uint8_t(0xFF & (r >> 16));
        rxf.frame.data[7] = std::uint8_t(0xFF & (r >> 24));
    }

    rfr_reg = CAN_RF0R_RFOM0 | CAN_RF0R_FOVR0 | CAN_RF0R_FULL0;  // Release FIFO entry we just read

    /*
     * Store with timeout into the FIFO buffer and signal update event
     */
    state_->pushRxFromISR(rxf);

    endOfInterruptHandlerHook();
}

inline void handleStatusChangeInterrupt(const std::uint32_t timestamp_usec)
{
    CAN->MSR = CAN_MSR_ERRI;        // Clear error interrupt flag

    /*
     * Handle bus-off event.
     * This event is edge-triggered, meaning that it will be generated only once per one bus-off occurence.
     */
    if UNLIKELY(bool(CAN->ESR & CAN_ESR_BOFF))
    {
        statistics_.bus_off_events++;

        bool tx_event_required = false;

        // Requesting transmission abort for all mailboxes
        CAN->TSR = CAN_TSR_ABRQ0 | CAN_TSR_ABRQ1 | CAN_TSR_ABRQ2;

        // Marking TX mailboxes empty
        for (unsigned i = 0; i < NumTxMailboxes; i++)
        {
            auto& tx = state_->pending_tx[i];
            if (tx.pending)
            {
                tx.pending = false;

                // If loopback is enabled, reporting that the transmission has failed.
                if (state_->loopback)
                {
                    RxFrame rxf;
                    rxf.frame           = tx.frame;
                    rxf.failed          = true;
                    rxf.loopback        = true;
                    rxf.timestamp_usec  = timestamp_usec;

                    state_->pushRxFromISR(rxf);
                }

                tx_event_required = true;
            }
        }

        // Flushing the TX queue
        while (const auto tx = state_->tx_queue.peek())
        {
            tx_event_required = true;

            // If loopback is enabled, reporting that the transmission has failed.
            if (state_->loopback)
            {
                RxFrame rxf;
                rxf.frame           = *tx;
                rxf.failed          = true;
                rxf.loopback        = true;
                rxf.timestamp_usec  = timestamp_usec;

                state_->pushRxFromISR(rxf);
            }

            state_->tx_queue.pop();
        }

        // Signaling an event if needed
        if (tx_event_required)
        {
            os::CriticalSectionLocker cs_locker;
            state_->tx_event.signalI();
        }
    }

    endOfInterruptHandlerHook();
}

/*
 * Timestamping
 */
__attribute__((always_inline))
inline std::uint32_t getTimestampUSec()
{
    const auto value = gptGetCounterX(&CAN_GPT);
    assert(value < TimestampRolloverIntervalUSec);
    return value;
}

} // namespace

inline bool Frame::priorityHigherThan(const Frame& rhs) const
{
    const uint32_t clean_id     = id     & MaskExtID;
    const uint32_t rhs_clean_id = rhs.id & MaskExtID;

    /*
     * STD vs EXT - if 11 most significant bits are the same, EXT loses.
     */
    const bool ext     = id     & FlagEFF;
    const bool rhs_ext = rhs.id & FlagEFF;
    if UNLIKELY(ext != rhs_ext)
    {
        const uint32_t arb11     = ext     ? (clean_id >> 18)     : clean_id;
        const uint32_t rhs_arb11 = rhs_ext ? (rhs_clean_id >> 18) : rhs_clean_id;
        if (arb11 != rhs_arb11)
        {
            return arb11 < rhs_arb11;
        }
        else
        {
            return rhs_ext;
        }
    }

    /*
     * RTR vs Data frame - if frame identifiers and frame types are the same, RTR loses.
     */
    const bool rtr     = id     & FlagRTR;
    const bool rhs_rtr = rhs.id & FlagRTR;
    if UNLIKELY(clean_id == rhs_clean_id && rtr != rhs_rtr)
    {
        return rhs_rtr;
    }

    /*
     * Plain ID arbitration - greater value loses.
     */
    return clean_id < rhs_clean_id;
}

int open(std::uint32_t bitrate, unsigned options)
{
    CommonMutexLocker mutex_locker;

    EXECUTE_ONCE_NON_THREAD_SAFE
    {
        /*
         * Initializing the GPT driver for timestamping, then starting the timer immediately.
         * The timer is started only once and never stopped in order to not disrupt time synchronization with the host.
         */
        static const GPTConfig gpt_cfg =
        {
            1000 * 1000,        // Clock rate [Hz]
            nullptr,            // Callback
            0,                  // CR2
            0                   // DIER
        };
        gptStart(&CAN_GPT, &gpt_cfg);
        gptStartContinuous(&CAN_GPT, TimestampRolloverIntervalUSec);

        /*
         * CAN macrocell and NVIC initialization (requires a critical section).
         */
        os::CriticalSectionLocker cs_lock;

        RCC->APB1ENR  |=  RCC_APB1ENR_CANEN;
        RCC->APB1RSTR |=  RCC_APB1RSTR_CANRST;
        RCC->APB1RSTR &= ~RCC_APB1RSTR_CANRST;
        nvicEnableVector(CAN_TX_IRQn, IRQPriority);
        nvicEnableVector(CAN_RX0_IRQn, IRQPriority);
        nvicEnableVector(CAN_RX1_IRQn, IRQPriority);
        nvicEnableVector(CAN_SCE_IRQn, IRQPriority);

#if CAN_IRQ_TRACE
        palSetPadMode(GPIOA, GPIOA_PIN_4, PAL_MODE_OUTPUT_PUSHPULL);
        palSetPadMode(GPIOA, GPIOA_PIN_5, PAL_MODE_OUTPUT_PUSHPULL);
        palSetPadMode(GPIOA, GPIOA_PIN_6, PAL_MODE_OUTPUT_PUSHPULL);
#endif
    }

    /*
     * We need to silence the controller in the first order, otherwise it may interfere with the following operations.
     */
    {
        os::CriticalSectionLocker cs_lock;

        // APB reset is the only way to guaranteedly bring the macrocell to the well-known initial state.
        RCC->APB1RSTR |=  RCC_APB1RSTR_CANRST;
        RCC->APB1RSTR &= ~RCC_APB1RSTR_CANRST;

        CAN->MCR &= ~CAN_MCR_SLEEP;     // Exit sleep mode
        CAN->MCR |= CAN_MCR_INRQ;       // Request init
        CAN->IER = 0;                   // Disable interrupts while initialization is in progress

        CAN->TSR = CAN_TSR_ABRQ0 | CAN_TSR_ABRQ1 | CAN_TSR_ABRQ2;   // Cancel all transmissions

        // This covers the case if this function was invoked while the controller was still active
        NVIC_ClearPendingIRQ(static_cast<IRQn_Type>(CAN_TX_IRQn));
        NVIC_ClearPendingIRQ(static_cast<IRQn_Type>(CAN_RX0_IRQn));
        NVIC_ClearPendingIRQ(static_cast<IRQn_Type>(CAN_RX1_IRQn));
        NVIC_ClearPendingIRQ(static_cast<IRQn_Type>(CAN_SCE_IRQn));
    }

    if (!waitMSRINAKBitStateChange(true))
    {
        return -ErrMsrInakNotSet;
    }

    /*
     * CAN timings for this bitrate
     */
    Timings timings;
    const int timings_res = computeTimings(bitrate, timings);
    if (timings_res < 0)
    {
        return timings_res;
    }
    DEBUG_LOG("Timings: presc=%u sjw=%u bs1=%u bs2=%u\n",
              unsigned(timings.prescaler), unsigned(timings.sjw), unsigned(timings.bs1), unsigned(timings.bs2));

    /*
     * Resetting driver state and statistics - CAN interrupts are disabled, so it's safe to modify it now
     */
    if (state_ != nullptr)
    {
        state_->~DriverState();
        state_ = nullptr;
    }

    static std::aligned_storage_t<sizeof(DriverState), alignof(DriverState)> _state_storage;
    state_ = new (&_state_storage) DriverState((options & OptionLoopback) != 0);

    statistics_ = Statistics();

    /*
     * Hardware initialization (the hardware has already confirmed initialization mode, see above)
     */
    CAN->MCR = CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_INRQ;  // RM page 648

    CAN->BTR = ((timings.sjw & 3U)  << 24) |
               ((timings.bs1 & 15U) << 16) |
               ((timings.bs2 & 7U)  << 20) |
               (timings.prescaler & 1023U) |
               (((options & OptionSilentMode) != 0) ? CAN_BTR_SILM : 0);

    // From now on the interrupts will be re-enabled
    CAN->IER = CAN_IER_TMEIE |          // TX mailbox empty
               CAN_IER_FMPIE0 |         // RX FIFO 0 is not empty
               CAN_IER_FMPIE1 |         // RX FIFO 1 is not empty
               CAN_IER_ERRIE |          // General error IRQ
               CAN_IER_BOFIE;           // Bus-off reached (seems to be edge-triggered)

    CAN->MCR &= ~CAN_MCR_INRQ;          // Leave init mode

    if (!waitMSRINAKBitStateChange(false))
    {
        state_->~DriverState();
        state_ = nullptr;
        return -ErrMsrInakNotCleared;
    }

    os::CriticalSectionLocker cs_lock;  // Entering a critical section for the rest of initialization

    /*
     * Default filter configuration
     */
    CAN->FMR |= CAN_FMR_FINIT;

    CAN->FMR &= 0xFFFFC0F1;
    CAN->FMR |= static_cast<std::uint32_t>(27) << 8;    // Refer to the bxCAN macrocell documentation for explanation

    CAN->FFA1R = 0;                     // All assigned to FIFO0 by default
    CAN->FM1R = 0;                      // Indentifier Mask mode

    CAN->FS1R = 0x1fff;
    CAN->sFilterRegister[0].FR1 = 0;
    CAN->sFilterRegister[0].FR2 = 0;
    CAN->FA1R = 1;

    CAN->FMR &= ~CAN_FMR_FINIT;

    return 0;
}

void close()
{
    CommonMutexLocker mutex_locker;
    os::CriticalSectionLocker cs_lock;

    CAN->IER = 0;                                               // Disable interrupts
    CAN->TSR = CAN_TSR_ABRQ0 | CAN_TSR_ABRQ1 | CAN_TSR_ABRQ2;   // Cancel all transmissions
    CAN->MCR = CAN_MCR_SLEEP | CAN_MCR_RESET;                   // Force software reset of the macrocell

    NVIC_ClearPendingIRQ(static_cast<IRQn_Type>(CAN_TX_IRQn));
    NVIC_ClearPendingIRQ(static_cast<IRQn_Type>(CAN_RX0_IRQn));
    NVIC_ClearPendingIRQ(static_cast<IRQn_Type>(CAN_RX1_IRQn));
    NVIC_ClearPendingIRQ(static_cast<IRQn_Type>(CAN_SCE_IRQn));

    if (state_ != nullptr)
    {
        state_->~DriverState();
        state_ = nullptr;
    }
}

bool isOpen()
{
    return state_ != nullptr;
}

int send(const Frame& frame, std::uint16_t timeout_ms)
{
    os::MutexLocker mutex_locker(tx_mutex_);

    if (state_ == nullptr)
    {
        return -ErrClosed;
    }

    if (frame.isErrorFrame() || frame.dlc > 8)
    {
        return -ErrUnsupportedFrame;
    }

    const auto started_at = chVTGetSystemTimeX();

    while (true)
    {
        {
            os::CriticalSectionLocker cs_locker;

            if (state_->tx_queue.push(frame))                   // Pushing the prioritized queue
            {
                const auto top = state_->tx_queue.peek();       // Returned frame may be different (always non-null)

                if (canAcceptNewTxFrameCS(*top))                // Checking if the hardware can accept the frame
                {
                    loadTxMailboxCS(*top);                      // Starting transmission
                    state_->tx_queue.pop();                     // Removing the top frame from the queue
                }

                return 1;
            }
        }

        // Blocking until next event or timeout
        const auto elapsed = chVTTimeElapsedSinceX(started_at);
        if (elapsed >= MS2ST(timeout_ms))
        {
            return 0;
        }
        state_->tx_event.waitForSysTicks(MS2ST(timeout_ms) - elapsed);
    }

    return -1;
}

int receive(RxFrame& out_frame, std::uint16_t timeout_ms)
{
    os::MutexLocker mutex_locker(rx_mutex_);

    if (state_ == nullptr)
    {
        return -ErrClosed;
    }

    const auto started_at = chVTGetSystemTimeX();

    while (true)
    {
        {
            os::CriticalSectionLocker cs_locker;
            if (state_->rx_queue.getLength() > 0)
            {
                state_->rx_queue.pop(out_frame);
                return 1;
            }
        }

        // Blocking until next event or timeout
        const auto elapsed = chVTTimeElapsedSinceX(started_at);
        if (elapsed >= MS2ST(timeout_ms))
        {
            return 0;
        }
        state_->rx_event.waitForSysTicks(MS2ST(timeout_ms) - elapsed);
    }

    return -1;
}

Statistics getStatistics()
{
    Statistics val;

    {
        os::CriticalSectionLocker cs_locker;

        // FIXME This is ugly but I don't have a better idea at the moment.
        if (state_ != nullptr)
        {
            state_->updateStatistics();
        }

        val = statistics_;
    }

    return val;
}

Status getStatus()
{
    const std::uint32_t esr = CAN->ESR;         // Access is atomic

    Status s;
    s.receive_error_counter  = 0xFF & (esr >> 24);
    s.transmit_error_counter = 0xFF & (esr >> 16);

    if (esr & CAN_ESR_BOFF)
    {
        s.state = s.State::BusOff;
    }
    else if (esr & CAN_ESR_EPVF)
    {
        s.state = s.State::ErrorPassive;
    }
    else
    {
        s.state = s.State::ErrorActive;
    }

    return s;
}

bool hadActivity()
{
    os::CriticalSectionLocker cs_locker;
    if (state_ != nullptr && state_->had_activity)
    {
        state_->had_activity = false;
        return true;
    }
    return false;
}

}

extern "C"
{

using namespace can;

CH_IRQ_HANDLER(STM32_CAN1_TX_HANDLER)
{
    CH_IRQ_PROLOGUE();
    CAN_IRQ_TRACE_BEGIN(4);

    const auto timestamp = getTimestampUSec();
    assert(state_ != nullptr);

    /*
     * TX interrupt handling
     * TXOK == false means that there was a hardware failure
     */
    const auto tsr = CAN->TSR;
    if (tsr & (CAN_TSR_RQCP0 | CAN_TSR_RQCP1 | CAN_TSR_RQCP2))
    {
        if (tsr & CAN_TSR_RQCP0)
        {
            const bool txok = (tsr & CAN_TSR_TXOK0) != 0;
            CAN->TSR = CAN_TSR_RQCP0;
            handleTxMailboxInterrupt(0, txok, timestamp);
        }
        if (tsr & CAN_TSR_RQCP1)
        {
            const bool txok = (tsr & CAN_TSR_TXOK1) != 0;
            CAN->TSR = CAN_TSR_RQCP1;
            handleTxMailboxInterrupt(1, txok, timestamp);
        }
        if (tsr & CAN_TSR_RQCP2)
        {
            const bool txok = (tsr & CAN_TSR_TXOK2) != 0;
            CAN->TSR = CAN_TSR_RQCP2;
            handleTxMailboxInterrupt(2, txok, timestamp);
        }
    }

    CAN_IRQ_TRACE_END(4);
    CH_IRQ_EPILOGUE();
}

CH_IRQ_HANDLER(STM32_CAN1_RX0_HANDLER)
{
    CH_IRQ_PROLOGUE();
    CAN_IRQ_TRACE_BEGIN(5);

    const auto timestamp = getTimestampUSec();
    assert(state_ != nullptr);

    while ((CAN->RF0R & CAN_RF0R_FMP0) != 0)
    {
        handleRxInterrupt(0, timestamp);
    }

    CAN_IRQ_TRACE_END(5);
    CH_IRQ_EPILOGUE();
}

CH_IRQ_HANDLER(STM32_CAN1_RX1_HANDLER)
{
    CH_IRQ_PROLOGUE();
    CAN_IRQ_TRACE_BEGIN(5);

    const auto timestamp = getTimestampUSec();
    assert(state_ != nullptr);

    while ((CAN->RF1R & CAN_RF1R_FMP1) != 0)
    {
        handleRxInterrupt(1, timestamp);
    }

    CAN_IRQ_TRACE_END(5);
    CH_IRQ_EPILOGUE();
}

CH_IRQ_HANDLER(STM32_CAN1_SCE_HANDLER)
{
    CH_IRQ_PROLOGUE();
    CAN_IRQ_TRACE_BEGIN(6);

    assert(state_ != nullptr);

    handleStatusChangeInterrupt(getTimestampUSec());

    CAN_IRQ_TRACE_END(6);
    CH_IRQ_EPILOGUE();
}

}
