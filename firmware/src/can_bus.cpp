/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <zubax_chibios/os.hpp>
#include <hal.h>
#include <unistd.h>
#include <type_traits>
#include <new>
#include "can_bus.hpp"

namespace can
{
namespace
{

constexpr unsigned IRQPriority = CORTEX_MAX_KERNEL_PRIORITY;
constexpr unsigned NumTxMailboxes = 3;

chibios_rt::Mutex mutex_;


class RxQueue
{
    static constexpr unsigned Capacity = 16;

    RxFrame buf_[Capacity];
    std::uint8_t in_ = 0;
    std::uint8_t out_ = 0;
    std::uint8_t len_ = 0;
    std::uint32_t overflow_cnt_ = 0;

    void registerOverflow()
    {
        if (overflow_cnt_ < 0xFFFFFFFF)
        {
            overflow_cnt_++;
        }
    }

public:
    void push(const RxFrame& frame)
    {
        buf_[in_] = frame;
        in_++;
        if (in_ >= Capacity)
        {
            in_ = 0;
        }
        len_++;
        if (len_ > Capacity)
        {
            len_ = Capacity;
            registerOverflow();
            out_++;
            if (out_ >= Capacity)
            {
                out_ = 0;
            }
        }
    }

    void pop(RxFrame& out_frame)
    {
        if (len_ > 0)
        {
            out_frame = buf_[out_];
            out_++;
            if (out_ >= Capacity)
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
        overflow_cnt_ = 0;
    }

    unsigned getLength() const { return len_; }

    std::uint32_t getOverflowCount() const { return overflow_cnt_; }
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


class BusEvent
{
    chibios_rt::CounterSemaphore sem_ = chibios_rt::CounterSemaphore(0);

public:
    bool waitMSec(const unsigned msec)
    {
        return sem_.wait(msec) == MSG_OK;
    }

    void signal()
    {
        sem_.signal();
    }

    void signalFromInterrupt()
    {
        chSysLockFromISR();
        sem_.signalI();
        chSysUnlockFromISR();
    }
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

    // TODO: remove later
    os::lowsyslog("Timings: quanta/bit: %d, sample point location: %.1f%%\n",
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

/*
 * Driver state
 */
struct State
{
    std::uint64_t error_cnt = 0;
    std::uint64_t tx_cnt    = 0;
    std::uint64_t rx_cnt    = 0;

    RxQueue rx_queue;
    BusEvent bus_event;
    TxItem pending_tx[NumTxMailboxes];

    std::uint8_t last_hw_error_code = 0;
    std::uint8_t peak_tx_mailbox_index = 0;
    bool had_activity = false;

    const bool loopback;

    State(bool option_loopback) :
        loopback(option_loopback)
    { }
};

State* state_ = nullptr;

} // namespace

int start(std::uint32_t bitrate, unsigned options)
{
    os::MutexLocker mutex_locker(mutex_);

    EXECUTE_ONCE_NON_THREAD_SAFE
    {
        os::CriticalSectionLocker cs_lock;

        RCC->APB1ENR  |=  RCC_APB1ENR_CANEN;
        RCC->APB1RSTR |=  RCC_APB1RSTR_CANRST;
        RCC->APB1RSTR &= ~RCC_APB1RSTR_CANRST;
        nvicEnableVector(CEC_CAN_IRQn, IRQPriority);
    }

    /*
     * We need to silence the controller in the first order, otherwise it may interfere with the following operations.
     */
    {
        os::CriticalSectionLocker cs_lock;

        CAN->MCR &= ~CAN_MCR_SLEEP;     // Exit sleep mode
        CAN->MCR |= CAN_MCR_INRQ;       // Request init
        CAN->IER = 0;                   // Disable interrupts while initialization is in progress
    }

    if (!waitMSRINAKBitStateChange(true))
    {
        return -ErrMsrInakNotSet;
    }

    /*
     * Resetting driver state - CAN interrupts are disabled, so it's safe to modify it now
     */
    static std::aligned_storage_t<sizeof(State), alignof(State)> _state_storage;
    state_ = new (&_state_storage) State((options & OptionLoopback) != 0);

    /*
     * CAN timings for this bitrate
     */
    Timings timings;
    const int timings_res = computeTimings(bitrate, timings);
    if (timings_res < 0)
    {
        return timings_res;
    }
    // TODO: remove later
    os::lowsyslog("Timings: presc=%u sjw=%u bs1=%u bs2=%u\n",
                  unsigned(timings.prescaler), unsigned(timings.sjw), unsigned(timings.bs1), unsigned(timings.bs2));

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
               CAN_IER_LECIE;           // Last error code change

    CAN->MCR &= ~CAN_MCR_INRQ;          // Leave init mode

    if (!waitMSRINAKBitStateChange(false))
    {
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

void stop()
{
    os::MutexLocker mutex_locker(mutex_);
}

int send(const Frame& frame, unsigned timeout_ms)
{
    os::MutexLocker mutex_locker(mutex_);
    (void)frame;
    (void)timeout_ms;
    return -1;
}

int receive(Frame& out_frame, unsigned timeout_ms)
{
    os::MutexLocker mutex_locker(mutex_);
    (void)out_frame;
    (void)timeout_ms;
    return -1;
}

}
