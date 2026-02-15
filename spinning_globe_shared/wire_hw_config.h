#ifndef FG_WIRE_HW_CONFIG_h
#define FG_WIRE_HW_CONFIG_h

// this file contains spinning globe Arduino nano HW related constants that must be known at the slave slide 

constexpr long AVR_classicNano_CPU_clockF{ 16000000 };                              // do NOT use F_CPU: we need the nano clock f at the slave side as well !
constexpr long timer1PreScaler{ 8 };                                                // 8 (as set in setup())
constexpr long timer1ClockFreq{ AVR_classicNano_CPU_clockF / timer1PreScaler };     // 2 MHz
constexpr long timer1PWMfreq{ 1000L };                                              // 1 KHz
constexpr long timer1Top{ timer1ClockFreq / timer1PWMfreq / 2 };                    // timer counts up and down : 2000 steps, TOP =1000

constexpr float ADCvolt{ 5000. };
constexpr long ADCsteps{ 1024L };                                                   // globe vertical position sensor: resolution (10 bit ADC)

constexpr long fastDataRateSamplingPeriods{ 1 << 7 };                               // in sampling periods (milliseconds, power of 2)

#endif