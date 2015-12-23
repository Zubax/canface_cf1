/*
 * Copyright (C) 2015  Zubax Robotics  <info@zubax.com>
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

/*
 * Based on ChibiOS demo application for STM32F051.
 */

#pragma once

#define STM32F042x6

/*
 * Pin definitions
 */
#define GPIOA_ADCIN0                    0
#define GPIOA_UART1_TX                  9
#define GPIOA_UART1_RX                  10
#define GPIOA_USB_DM                    11
#define GPIOA_USB_DP                    12
#define GPIOA_SWDIO                     13
#define GPIOA_SWCLK                     14

#define GPIOB_CAN_POWER_EN              1
#define GPIOB_CAN_TERMINATOR_EN         2
#define GPIOB_CAN_RX                    8
#define GPIOB_CAN_TX                    9
#define GPIOB_LED_STATUS                10
#define GPIOB_LED_TRAFFIC               11

#define GPIOF_OSC_IN                    0
#define GPIOF_OSC_OUT                   1

/*
 * I/O ports initial setup, this configuration is established soon after reset in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_2M(n)            (0U << ((n) * 2U))
#define PIN_OSPEED_10M(n)           (1U << ((n) * 2U))
#define PIN_OSPEED_40M(n)           (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

/*
 * GPIOA
 */
#define VAL_GPIOA_MODER             (PIN_MODE_ANALOG(GPIOA_ADCIN0) |\
                                     PIN_MODE_INPUT(1) |\
                                     PIN_MODE_INPUT(2) |\
                                     PIN_MODE_INPUT(3) |\
                                     PIN_MODE_INPUT(4) |\
                                     PIN_MODE_INPUT(5) |\
                                     PIN_MODE_INPUT(6) |\
                                     PIN_MODE_INPUT(7) |\
                                     PIN_MODE_INPUT(8) |\
                                     PIN_MODE_ALTERNATE(GPIOA_UART1_TX) |\
                                     PIN_MODE_ALTERNATE(GPIOA_UART1_RX) |\
                                     PIN_MODE_ANALOG(GPIOA_USB_DM) |\
                                     PIN_MODE_ANALOG(GPIOA_USB_DP) |\
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO) |\
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK) |\
                                     PIN_MODE_INPUT(15))

#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_ADCIN0) |\
                                     PIN_OTYPE_PUSHPULL(1) |\
                                     PIN_OTYPE_PUSHPULL(2) |\
                                     PIN_OTYPE_PUSHPULL(3) |\
                                     PIN_OTYPE_PUSHPULL(4) |\
                                     PIN_OTYPE_PUSHPULL(5) |\
                                     PIN_OTYPE_PUSHPULL(6) |\
                                     PIN_OTYPE_PUSHPULL(7) |\
                                     PIN_OTYPE_PUSHPULL(8) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART1_TX) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART1_RX) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_USB_DM) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_USB_DP) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWDIO) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWCLK) |\
                                     PIN_OTYPE_PUSHPULL(15))

#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_2M(GPIOA_ADCIN0) |\
                                     PIN_OSPEED_2M(1) |\
                                     PIN_OSPEED_2M(2) |\
                                     PIN_OSPEED_2M(3) |\
                                     PIN_OSPEED_2M(4) |\
                                     PIN_OSPEED_2M(5) |\
                                     PIN_OSPEED_2M(6) |\
                                     PIN_OSPEED_2M(7) |\
                                     PIN_OSPEED_2M(8) |\
                                     PIN_OSPEED_40M(GPIOA_UART1_TX) |\
                                     PIN_OSPEED_2M(GPIOA_UART1_RX) |\
                                     PIN_OSPEED_40M(GPIOA_USB_DM) |\
                                     PIN_OSPEED_40M(GPIOA_USB_DP) |\
                                     PIN_OSPEED_40M(GPIOA_SWDIO) |\
                                     PIN_OSPEED_40M(GPIOA_SWCLK) |\
                                     PIN_OSPEED_2M(15))

#define VAL_GPIOA_PUPDR             (PIN_PUPDR_FLOATING(GPIOA_ADCIN0) |\
                                     PIN_PUPDR_PULLDOWN(1) |\
                                     PIN_PUPDR_PULLDOWN(2) |\
                                     PIN_PUPDR_PULLDOWN(3) |\
                                     PIN_PUPDR_PULLDOWN(4) |\
                                     PIN_PUPDR_PULLDOWN(5) |\
                                     PIN_PUPDR_PULLDOWN(6) |\
                                     PIN_PUPDR_PULLDOWN(7) |\
                                     PIN_PUPDR_PULLDOWN(8) |\
                                     PIN_PUPDR_FLOATING(GPIOA_UART1_TX) |\
                                     PIN_PUPDR_PULLUP(GPIOA_UART1_RX) |\
                                     PIN_PUPDR_FLOATING(GPIOA_USB_DM) |\
                                     PIN_PUPDR_FLOATING(GPIOA_USB_DP) |\
                                     PIN_PUPDR_PULLUP(GPIOA_SWDIO) |\
                                     PIN_PUPDR_PULLDOWN(GPIOA_SWCLK) |\
                                     PIN_PUPDR_PULLDOWN(15))

#define VAL_GPIOA_ODR               (PIN_ODR_LOW(GPIOA_ADCIN0) |\
                                     PIN_ODR_LOW(1) |\
                                     PIN_ODR_LOW(2) |\
                                     PIN_ODR_LOW(3) |\
                                     PIN_ODR_LOW(4) |\
                                     PIN_ODR_LOW(5) |\
                                     PIN_ODR_LOW(6) |\
                                     PIN_ODR_LOW(7) |\
                                     PIN_ODR_LOW(8) |\
                                     PIN_ODR_HIGH(GPIOA_UART1_TX) |\
                                     PIN_ODR_HIGH(GPIOA_UART1_RX) |\
                                     PIN_ODR_LOW(GPIOA_USB_DM) |\
                                     PIN_ODR_LOW(GPIOA_USB_DP) |\
                                     PIN_ODR_LOW(GPIOA_SWDIO) |\
                                     PIN_ODR_LOW(GPIOA_SWCLK) |\
                                     PIN_ODR_LOW(15))

#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_ADCIN0,  0) |\
                                     PIN_AFIO_AF(1,  0) |\
                                     PIN_AFIO_AF(2,  0) |\
                                     PIN_AFIO_AF(3,  0) |\
                                     PIN_AFIO_AF(4,  0) |\
                                     PIN_AFIO_AF(5,  0) |\
                                     PIN_AFIO_AF(6,  0) |\
                                     PIN_AFIO_AF(7,  0) )
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(8,  0) |\
                                     PIN_AFIO_AF(GPIOA_UART1_TX, 1) |\
                                     PIN_AFIO_AF(GPIOA_UART1_RX, 1) |\
                                     PIN_AFIO_AF(GPIOA_USB_DM, 0) |\
                                     PIN_AFIO_AF(GPIOA_USB_DP, 0) |\
                                     PIN_AFIO_AF(GPIOA_SWDIO, 0) |\
                                     PIN_AFIO_AF(GPIOA_SWCLK, 0) |\
                                     PIN_AFIO_AF(15, 0))

/*
 * GPIOB
 */
#define VAL_GPIOB_MODER             (PIN_MODE_INPUT(0) |\
                                     PIN_MODE_OUTPUT(GPIOB_CAN_POWER_EN) |\
                                     PIN_MODE_OUTPUT(GPIOB_CAN_TERMINATOR_EN) |\
                                     PIN_MODE_INPUT(3) |\
                                     PIN_MODE_INPUT(4) |\
                                     PIN_MODE_INPUT(5) |\
                                     PIN_MODE_INPUT(6) |\
                                     PIN_MODE_INPUT(7) |\
                                     PIN_MODE_ALTERNATE(GPIOB_CAN_RX) |\
                                     PIN_MODE_ALTERNATE(GPIOB_CAN_TX) |\
                                     PIN_MODE_OUTPUT(GPIOB_LED_STATUS) |\
                                     PIN_MODE_OUTPUT(GPIOB_LED_TRAFFIC) |\
                                     PIN_MODE_INPUT(12) |\
                                     PIN_MODE_INPUT(13) |\
                                     PIN_MODE_INPUT(14) |\
                                     PIN_MODE_INPUT(15))

#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(0) |\
                                     PIN_OTYPE_OPENDRAIN(GPIOB_CAN_POWER_EN) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_CAN_TERMINATOR_EN) |\
                                     PIN_OTYPE_PUSHPULL(3) |\
                                     PIN_OTYPE_PUSHPULL(4) |\
                                     PIN_OTYPE_PUSHPULL(5) |\
                                     PIN_OTYPE_PUSHPULL(6) |\
                                     PIN_OTYPE_PUSHPULL(7) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_CAN_RX) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_CAN_TX) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_LED_STATUS) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_LED_TRAFFIC) |\
                                     PIN_OTYPE_PUSHPULL(12) |\
                                     PIN_OTYPE_PUSHPULL(13) |\
                                     PIN_OTYPE_PUSHPULL(14) |\
                                     PIN_OTYPE_PUSHPULL(15))

#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_2M(0) |\
                                     PIN_OSPEED_2M(GPIOB_CAN_POWER_EN) |\
                                     PIN_OSPEED_2M(GPIOB_CAN_TERMINATOR_EN) |\
                                     PIN_OSPEED_2M(3) |\
                                     PIN_OSPEED_2M(4) |\
                                     PIN_OSPEED_2M(5) |\
                                     PIN_OSPEED_2M(6) |\
                                     PIN_OSPEED_2M(7) |\
                                     PIN_OSPEED_2M(GPIOB_CAN_RX) |\
                                     PIN_OSPEED_40M(GPIOB_CAN_TX) |\
                                     PIN_OSPEED_2M(GPIOB_LED_STATUS) |\
                                     PIN_OSPEED_2M(GPIOB_LED_TRAFFIC) |\
                                     PIN_OSPEED_2M(12) |\
                                     PIN_OSPEED_2M(13) |\
                                     PIN_OSPEED_2M(14) |\
                                     PIN_OSPEED_2M(15))

#define VAL_GPIOB_PUPDR             (PIN_PUPDR_PULLDOWN(0) |\
                                     PIN_PUPDR_FLOATING(GPIOB_CAN_POWER_EN) |\
                                     PIN_PUPDR_FLOATING(GPIOB_CAN_TERMINATOR_EN) |\
                                     PIN_PUPDR_PULLDOWN(3) |\
                                     PIN_PUPDR_PULLDOWN(4) |\
                                     PIN_PUPDR_PULLDOWN(5) |\
                                     PIN_PUPDR_PULLDOWN(6) |\
                                     PIN_PUPDR_PULLDOWN(7) |\
                                     PIN_PUPDR_PULLUP(GPIOB_CAN_RX) |\
                                     PIN_PUPDR_PULLUP(GPIOB_CAN_TX) |\
                                     PIN_PUPDR_FLOATING(GPIOB_LED_STATUS) |\
                                     PIN_PUPDR_FLOATING(GPIOB_LED_TRAFFIC) |\
                                     PIN_PUPDR_PULLDOWN(12) |\
                                     PIN_PUPDR_PULLDOWN(13) |\
                                     PIN_PUPDR_PULLDOWN(14) |\
                                     PIN_PUPDR_PULLDOWN(15))

#define VAL_GPIOB_ODR               (PIN_ODR_LOW(0) |\
                                     PIN_ODR_LOW(GPIOB_CAN_POWER_EN) |\
                                     PIN_ODR_LOW(GPIOB_CAN_TERMINATOR_EN) |\
                                     PIN_ODR_LOW(3) |\
                                     PIN_ODR_LOW(4) |\
                                     PIN_ODR_LOW(5) |\
                                     PIN_ODR_LOW(6) |\
                                     PIN_ODR_LOW(7) |\
                                     PIN_ODR_HIGH(GPIOB_CAN_RX) |\
                                     PIN_ODR_HIGH(GPIOB_CAN_TX) |\
                                     PIN_ODR_HIGH(GPIOB_LED_STATUS) |\
                                     PIN_ODR_HIGH(GPIOB_LED_TRAFFIC) |\
                                     PIN_ODR_LOW(12) |\
                                     PIN_ODR_LOW(13) |\
                                     PIN_ODR_LOW(14) |\
                                     PIN_ODR_LOW(15))

#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(0,  0) |\
                                     PIN_AFIO_AF(GPIOB_CAN_POWER_EN,  0) |\
                                     PIN_AFIO_AF(GPIOB_CAN_TERMINATOR_EN,  0) |\
                                     PIN_AFIO_AF(3,  0) |\
                                     PIN_AFIO_AF(4,  0) |\
                                     PIN_AFIO_AF(5,  0) |\
                                     PIN_AFIO_AF(6,  0) |\
                                     PIN_AFIO_AF(7,  0) )
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_CAN_RX,  4) |\
                                     PIN_AFIO_AF(GPIOB_CAN_TX,  4) |\
                                     PIN_AFIO_AF(GPIOB_LED_STATUS, 0) |\
                                     PIN_AFIO_AF(GPIOB_LED_TRAFFIC, 0) |\
                                     PIN_AFIO_AF(12, 0) |\
                                     PIN_AFIO_AF(13, 0) |\
                                     PIN_AFIO_AF(14, 0) |\
                                     PIN_AFIO_AF(15, 0))

/*
 * GPIOC
 */
#define VAL_GPIOC_MODER             (PIN_MODE_INPUT(0) |\
                                     PIN_MODE_INPUT(1) |\
                                     PIN_MODE_INPUT(2) |\
                                     PIN_MODE_INPUT(3) |\
                                     PIN_MODE_INPUT(4) |\
                                     PIN_MODE_INPUT(5) |\
                                     PIN_MODE_INPUT(6) |\
                                     PIN_MODE_INPUT(7) |\
                                     PIN_MODE_INPUT(8) |\
                                     PIN_MODE_INPUT(9) |\
                                     PIN_MODE_INPUT(10) |\
                                     PIN_MODE_INPUT(11) |\
                                     PIN_MODE_INPUT(12) |\
                                     PIN_MODE_INPUT(13) |\
                                     PIN_MODE_INPUT(14) |\
                                     PIN_MODE_INPUT(15))

#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(0) |\
                                     PIN_OTYPE_PUSHPULL(1) |\
                                     PIN_OTYPE_PUSHPULL(2) |\
                                     PIN_OTYPE_PUSHPULL(3) |\
                                     PIN_OTYPE_PUSHPULL(4) |\
                                     PIN_OTYPE_PUSHPULL(5) |\
                                     PIN_OTYPE_PUSHPULL(6) |\
                                     PIN_OTYPE_PUSHPULL(7) |\
                                     PIN_OTYPE_PUSHPULL(8) |\
                                     PIN_OTYPE_PUSHPULL(9) |\
                                     PIN_OTYPE_PUSHPULL(10) |\
                                     PIN_OTYPE_PUSHPULL(11) |\
                                     PIN_OTYPE_PUSHPULL(12) |\
                                     PIN_OTYPE_PUSHPULL(13) |\
                                     PIN_OTYPE_PUSHPULL(14) |\
                                     PIN_OTYPE_PUSHPULL(15))

#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_2M(0) |\
                                     PIN_OSPEED_2M(1) |\
                                     PIN_OSPEED_2M(2) |\
                                     PIN_OSPEED_2M(3) |\
                                     PIN_OSPEED_2M(4) |\
                                     PIN_OSPEED_2M(5) |\
                                     PIN_OSPEED_2M(6) |\
                                     PIN_OSPEED_2M(7) |\
                                     PIN_OSPEED_2M(8) |\
                                     PIN_OSPEED_2M(9) |\
                                     PIN_OSPEED_2M(10) |\
                                     PIN_OSPEED_2M(11) |\
                                     PIN_OSPEED_2M(12) |\
                                     PIN_OSPEED_2M(13) |\
                                     PIN_OSPEED_2M(14) |\
                                     PIN_OSPEED_2M(15))

#define VAL_GPIOC_PUPDR             (PIN_PUPDR_PULLDOWN(0) |\
                                     PIN_PUPDR_PULLDOWN(1) |\
                                     PIN_PUPDR_PULLDOWN(2) |\
                                     PIN_PUPDR_PULLDOWN(3) |\
                                     PIN_PUPDR_PULLDOWN(4) |\
                                     PIN_PUPDR_PULLDOWN(5) |\
                                     PIN_PUPDR_PULLDOWN(6) |\
                                     PIN_PUPDR_PULLDOWN(7) |\
                                     PIN_PUPDR_PULLDOWN(8) |\
                                     PIN_PUPDR_PULLDOWN(9) |\
                                     PIN_PUPDR_PULLDOWN(10) |\
                                     PIN_PUPDR_PULLDOWN(11) |\
                                     PIN_PUPDR_PULLDOWN(12) |\
                                     PIN_PUPDR_PULLDOWN(13) |\
                                     PIN_PUPDR_PULLDOWN(14) |\
                                     PIN_PUPDR_PULLDOWN(15))

#define VAL_GPIOC_ODR               (PIN_ODR_LOW(0) |\
                                     PIN_ODR_LOW(1) |\
                                     PIN_ODR_LOW(2) |\
                                     PIN_ODR_LOW(3) |\
                                     PIN_ODR_LOW(4) |\
                                     PIN_ODR_LOW(5) |\
                                     PIN_ODR_LOW(6) |\
                                     PIN_ODR_LOW(7) |\
                                     PIN_ODR_LOW(8) |\
                                     PIN_ODR_LOW(9) |\
                                     PIN_ODR_LOW(10) |\
                                     PIN_ODR_LOW(11) |\
                                     PIN_ODR_LOW(12) |\
                                     PIN_ODR_LOW(13) |\
                                     PIN_ODR_LOW(14) |\
                                     PIN_ODR_LOW(15))

#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(0,  0) |\
                                     PIN_AFIO_AF(1,  0) |\
                                     PIN_AFIO_AF(2,  0) |\
                                     PIN_AFIO_AF(3,  0) |\
                                     PIN_AFIO_AF(4,  0) |\
                                     PIN_AFIO_AF(5,  0) |\
                                     PIN_AFIO_AF(6,  0) |\
                                     PIN_AFIO_AF(7,  0) )
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(8,  0) |\
                                     PIN_AFIO_AF(9,  0) |\
                                     PIN_AFIO_AF(10, 0) |\
                                     PIN_AFIO_AF(11, 0) |\
                                     PIN_AFIO_AF(12, 0) |\
                                     PIN_AFIO_AF(13, 0) |\
                                     PIN_AFIO_AF(14, 0) |\
                                     PIN_AFIO_AF(15, 0))

/*
 * GPIOF
 */
#define VAL_GPIOF_MODER             (PIN_MODE_INPUT(GPIOF_OSC_IN) |\
                                     PIN_MODE_INPUT(GPIOF_OSC_OUT) |\
                                     PIN_MODE_INPUT(2) |\
                                     PIN_MODE_INPUT(3) |\
                                     PIN_MODE_INPUT(4) |\
                                     PIN_MODE_INPUT(5) |\
                                     PIN_MODE_INPUT(6) |\
                                     PIN_MODE_INPUT(7) |\
                                     PIN_MODE_INPUT(8) |\
                                     PIN_MODE_INPUT(9) |\
                                     PIN_MODE_INPUT(10) |\
                                     PIN_MODE_INPUT(11) |\
                                     PIN_MODE_INPUT(12) |\
                                     PIN_MODE_INPUT(13) |\
                                     PIN_MODE_INPUT(14) |\
                                     PIN_MODE_INPUT(15))

#define VAL_GPIOF_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOF_OSC_IN) |\
                                     PIN_OTYPE_PUSHPULL(GPIOF_OSC_OUT) |\
                                     PIN_OTYPE_PUSHPULL(2) |\
                                     PIN_OTYPE_PUSHPULL(3) |\
                                     PIN_OTYPE_PUSHPULL(4) |\
                                     PIN_OTYPE_PUSHPULL(5) |\
                                     PIN_OTYPE_PUSHPULL(6) |\
                                     PIN_OTYPE_PUSHPULL(7) |\
                                     PIN_OTYPE_PUSHPULL(8) |\
                                     PIN_OTYPE_PUSHPULL(9) |\
                                     PIN_OTYPE_PUSHPULL(10) |\
                                     PIN_OTYPE_PUSHPULL(11) |\
                                     PIN_OTYPE_PUSHPULL(12) |\
                                     PIN_OTYPE_PUSHPULL(13) |\
                                     PIN_OTYPE_PUSHPULL(14) |\
                                     PIN_OTYPE_PUSHPULL(15))

#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_2M(GPIOF_OSC_IN) |\
                                     PIN_OSPEED_2M(GPIOF_OSC_OUT) |\
                                     PIN_OSPEED_2M(2) |\
                                     PIN_OSPEED_2M(3) |\
                                     PIN_OSPEED_2M(4) |\
                                     PIN_OSPEED_2M(5) |\
                                     PIN_OSPEED_2M(6) |\
                                     PIN_OSPEED_2M(7) |\
                                     PIN_OSPEED_2M(8) |\
                                     PIN_OSPEED_2M(9) |\
                                     PIN_OSPEED_2M(10) |\
                                     PIN_OSPEED_2M(11) |\
                                     PIN_OSPEED_2M(12) |\
                                     PIN_OSPEED_2M(13) |\
                                     PIN_OSPEED_2M(14) |\
                                     PIN_OSPEED_2M(15))

#define VAL_GPIOF_PUPDR             (PIN_PUPDR_FLOATING(GPIOF_OSC_IN) |\
                                     PIN_PUPDR_FLOATING(GPIOF_OSC_OUT) |\
                                     PIN_PUPDR_PULLDOWN(2) |\
                                     PIN_PUPDR_PULLDOWN(3) |\
                                     PIN_PUPDR_PULLDOWN(4) |\
                                     PIN_PUPDR_PULLDOWN(5) |\
                                     PIN_PUPDR_PULLDOWN(6) |\
                                     PIN_PUPDR_PULLDOWN(7) |\
                                     PIN_PUPDR_PULLDOWN(8) |\
                                     PIN_PUPDR_PULLDOWN(9) |\
                                     PIN_PUPDR_PULLDOWN(10) |\
                                     PIN_PUPDR_PULLDOWN(11) |\
                                     PIN_PUPDR_PULLDOWN(12) |\
                                     PIN_PUPDR_PULLDOWN(13) |\
                                     PIN_PUPDR_PULLDOWN(14) |\
                                     PIN_PUPDR_PULLDOWN(15))

#define VAL_GPIOF_ODR               (PIN_ODR_LOW(GPIOF_OSC_IN) |\
                                     PIN_ODR_LOW(GPIOF_OSC_OUT) |\
                                     PIN_ODR_LOW(2) |\
                                     PIN_ODR_LOW(3) |\
                                     PIN_ODR_LOW(4) |\
                                     PIN_ODR_LOW(5) |\
                                     PIN_ODR_LOW(6) |\
                                     PIN_ODR_LOW(7) |\
                                     PIN_ODR_LOW(8) |\
                                     PIN_ODR_LOW(9) |\
                                     PIN_ODR_LOW(10) |\
                                     PIN_ODR_LOW(11) |\
                                     PIN_ODR_LOW(12) |\
                                     PIN_ODR_LOW(13) |\
                                     PIN_ODR_LOW(14) |\
                                     PIN_ODR_LOW(15))

#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(GPIOF_OSC_IN,  0) |\
                                     PIN_AFIO_AF(GPIOF_OSC_OUT, 0) |\
                                     PIN_AFIO_AF(2,  0) |\
                                     PIN_AFIO_AF(3,  0) |\
                                     PIN_AFIO_AF(4,  0) |\
                                     PIN_AFIO_AF(5,  0) |\
                                     PIN_AFIO_AF(6,  0) |\
                                     PIN_AFIO_AF(7,  0) )
#define VAL_GPIOF_AFRH              (PIN_AFIO_AF(8,  0) |\
                                     PIN_AFIO_AF(9,  0) |\
                                     PIN_AFIO_AF(10, 0) |\
                                     PIN_AFIO_AF(11, 0) |\
                                     PIN_AFIO_AF(12, 0) |\
                                     PIN_AFIO_AF(13, 0) |\
                                     PIN_AFIO_AF(14, 0) |\
                                     PIN_AFIO_AF(15, 0))


#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
    void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */
