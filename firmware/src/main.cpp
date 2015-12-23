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

#include <ch.hpp>
#include <hal.h>
#include <unistd.h>
#include <zubax_chibios/os.hpp>

#include "board/board.hpp"


int main()
{
    board::init();

    board::enableCANPower(true);
    board::enableCANTerminator(true);

    while (true)
    {
        board::setStatusLED(true);
        board::setTrafficLED(true);

        ::usleep(100000);

        board::setStatusLED(false);
        board::setTrafficLED(false);

        ::usleep(900000);
    }
}
