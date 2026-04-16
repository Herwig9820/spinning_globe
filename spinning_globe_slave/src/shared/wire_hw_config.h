/*
==================================================================================================
Floating and spinning earth globe
---------------------------------
Copyright 2019, 2026 Herwig Taveirne

Program written and tested for classic (8-bit) Arduino Nano.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

See GitHub for more information and documentation: https://github.com/Herwig9820/spinning_globe

A complete description of this project can be found here:
https://www.instructables.com/Floating-and-Spinning-Earth-Globe/

===============================================================================================
Spinning globe extension: using the Wire interface to exchange messages with an Arduino nano esp32.
---------------------------------------------------------------------------------------------------
An Arduino nano esp32, acting as a bridge, will control the spinning globe (changing settings, checking states)
over WiFi, e.g. using MQTT.

Note that, if the program is compiled with this option enabled, hardware buttons and LCD (connector SV2)...
...will be inoperable (switches are still functioning). USB terminal is not used except for a welcome message.

===============================================================================================
*/

#ifndef FG_WIRE_HW_CONFIG_h
#define FG_WIRE_HW_CONFIG_h

// this file contains spinning globe Arduino nano HW related constants that must be known at the slave slide 

constexpr long AVR_classicNano_CPU_clockF{ 16000000 };                              // do NOT use F_CPU: we need the nano clock f at the slave side as well !
constexpr long timer1PreScaler{ 8 };                                                // 8 (as set in setup())
constexpr long timer1ClockFreq{ AVR_classicNano_CPU_clockF / timer1PreScaler };     // 2 MHz
constexpr long timer1PWMfreq{ 1000L };                                              // 1 KHz
constexpr long timer1Top{ timer1ClockFreq / timer1PWMfreq / 2 };                    // timer counts up and down : 2000 steps, TOP =1000


constexpr float samplingPeriod{ 1. / (float)timer1PWMfreq };                        // 1 millisecond sampling period, in seconds



constexpr float ADCvolt{ 5000. };
constexpr long ADCsteps{ 1024L };                                                   // globe vertical position sensor: resolution (10 bit ADC)

constexpr long fastDataRateSamplingPeriods{ 1 << 7 };                               // in sampling periods (milliseconds, power of 2)

#endif