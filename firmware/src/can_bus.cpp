/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <zubax_chibios/os.hpp>
#include <hal.h>
#include "can_bus.hpp"

namespace can
{
namespace
{

constexpr unsigned IRQPriority = 2;
constexpr unsigned RxQueueSize = 8;

chibios_rt::Mutex mutex_;


} // namespace

int start(std::uint32_t bitrate, unsigned options)
{
    os::MutexLocker mutex_locker(mutex_);

    EXECUTE_ONCE_NON_THREAD_SAFE
    {
        os::CriticalSectionLocker lock;

        RCC->APB1ENR  |=  RCC_APB1ENR_CANEN;
        RCC->APB1RSTR |=  RCC_APB1RSTR_CANRST;
        RCC->APB1RSTR &= ~RCC_APB1RSTR_CANRST;
        nvicEnableVector(CEC_CAN_IRQn, IRQPriority);
    }

    /*
     * We need to silence the controller in the first order, otherwise it may interfere with the following operations.
     */
    {
        os::CriticalSectionLocker lock;

        CAN->MCR &= ~CAN_MCR_SLEEP;     // Exit sleep mode
        CAN->MCR |= CAN_MCR_INRQ;       // Request init
        CAN->IER = 0;                   // Disable interrupts while initialization is in progress
    }

    (void)bitrate;
    (void)options;

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
