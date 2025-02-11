/*
    Name:       spinning_globe.ino
    Created:    10/08/2019 - 11/02/2025
    Author:     Herwig Taveirne
    Version:    1.2.2

    Program written and tested for Arduino Nano
    Timer 1 reading in class MyTime, in procedure idleLoop and in ISR assumes clock speed is 16Mhz

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

*/

#include <LiquidCrystal.h>                                  // LCD
#include <util/atomic.h>                                    // atomic operations
#include <avr/wdt.h>                                        // watchdog timer
#include <limits.h>                                         // specific constants
#include <Arduino.h>
#include <stdlib.h>

#define boardVersion 101                                    // board version: 100 = hardware v1, 101 = v1 rev A and B
#define highAnalogGain 1                                    // 0: analog gain is 10, 1: analog gain is 15 (defined by resistors R9 to R12)

#define test_showEventStats 0                               // only for testing (event message mechanism)


// *** enumerations ***

enum userCmds :int {
    uNoCmd = -1,
    uPrevious, uNext, uDown, uUp, uEdit, uReset, uCancel, uShowAll, uLive, uTimeStamp, uHelp,                           // cmds without parameters: 0 - 99
    uMeasure = 100,                                                                                                     // cmds with 1 parameter: 100-199
    uLedstripSettings = 200,                                                                                            // cmds with 2 parameters: 200-299
    uUnknownCmd = 999                                                                                                   // unknown command receives code 999
};

enum rotStatus :uint8_t { rotNoPosSync, rotFreeRunning, rotMeasuring, rotUnlocked, rotLocked };                         // rotNoPosSync: also if rotation OFF or not floating   
enum errStatus :uint8_t { errNoError = 0, errDroppedGlobe, errStickyGlobe, errMagnetLoad, errTemp };
// eBlink, eSpareNoDataEvent1: cue only (no data) events. additional time cues can be added
enum events :uint8_t { eNoEvent = 0, eGreenwich, eStatusChange, eFastRateData, eLedstripData, eStepResponseData, eSecond, eBlink, eSpareNoDataEvent1 };
enum colorCycles :uint8_t { cLedstripOff = 0, cCstBrightWhite, cCstBrightMagenta, cCstBrightBlue, cWhiteBlue, cRedGreenBlue };      // led strip color cycle 
enum colorTiming :uint8_t { cLedstripVeryFast = 0, cLedstripFast, cLedstripSlow, cLedStripVerySlow };                               // led strip color cycle 


// *** I/O ***

// port A 
constexpr uint8_t A0_liftHallPin{ A0 };                         // port A analog input pin A0: vertical position sensor
constexpr uint8_t A1_temperaturePin{ A1 };                      // port A analog input pin A1: temperature sensor


// port B                                                       
constexpr uint8_t B1_OC1Apin{ 9 };                              // port B bit 1 (Nano pin D9): output pin for 16-bit timer 1 channel A (drives magnet)  

#if (boardVersion == 100)                                       
constexpr uint8_t B2_LCDenablePin{ 13 };                        // port B bit 5 (Nano pin D13): LCD enable  

constexpr uint8_t portB_IOchannelSelectBitMask{ B00011100 };    // port B bits 432: I/O channel select (74HCT138 decoder)

constexpr uint8_t portB_coilFlipFlopSelect{ 0 << 2 };           // decoder select lines: bits 432 = 000, 001, 010, 011              
constexpr uint8_t portB_auxFlipFlopSelect{ 1 << 2 };
constexpr uint8_t portB_switchesBufferSelect{ 2 << 2 };
constexpr uint8_t portB_ledstripSelect{ 3 << 2 };
#else                                                           
constexpr  uint8_t B2_LCDenablePin{ 10 };                       // port B bit 2 (Nano pin D10): LCD enable

constexpr uint8_t portB_IOchannelSelectBitMask{ B00110001 };    // port B bits 540: I/O channel select (74HCT138 decoder)

constexpr uint8_t portB_coilFlipFlopSelect{ B00000000 };        // decoder select lines: bits 540 = 000, 001, 010, 011              
constexpr uint8_t portB_auxFlipFlopSelect{ B00000001 };
constexpr uint8_t portB_switchesBufferSelect{ B00010000 };
constexpr uint8_t portB_ledstripSelect{ B00010001 };
#endif                                                          


// port C                                                       
constexpr uint8_t A2_IONotEnablePin{ A2 };                      // port C bit 2 (Nano pin A2): I/O channel not enable (74HCT138 decoder)
constexpr uint8_t portC_IOdisableBit{ B00000100 };              // port C bit 2


// port D                                                       
constexpr uint8_t D3_LCDregSelPin{ 3 };                         // port D bit 3 (Nano pin D3): LCD register select

constexpr uint8_t portD_redStatusLedbit{ B00001000 };           // port D bit 3: red status led bit mask
constexpr uint8_t portD_greenStatusLedBit{ B00010000 };         // port D bit 4: green status led bit mask
#if (boardVersion == 100)                                       
constexpr uint8_t portD_interruptInProgressBit{ B00100000 };    // port D bit 5: interrupt in progress bit mask
constexpr uint8_t portD_blueStatusLedBit{ B01000000 };          // port D bit 6: blue status led bit mask
#else                                                           
constexpr uint8_t portD_blueStatusLedBit{ B00100000 };          // port D bit 5: blue status led bit mask
constexpr uint8_t portD_interruptInProgressBit{ B01000000 };    // port D bit 6: interrupt in progress bit mask
#endif                                                          
constexpr uint8_t portD_enableMotorBit{ B10000000 };            // port D bit 7: enable motor bit mask

constexpr uint8_t pinD_firstKeyBit{ B00000100 };                // port D bit 2: first key bit
constexpr uint8_t pinD_switchStateBits{ B01111100 };            // port D bits 65432: all switches AND keys (producing switch states)
constexpr uint8_t pinD_keyBits{ B00111100 };                    // port D bits 5432: keys ONLY (producing key codes when pressed)
constexpr uint8_t pinD_greenwichBit{ B10000000 };               // port D bit 7: Greenwich sync bit mask

uint8_t portDbuffer{ 0 }, dataInBuffer{ 0 };


// port depending on board version
#if (boardVersion == 100)
constexpr uint8_t A3_ledstripDataPin{ A3 };                     // port C bit 3 (Nano pin A3): led strip data
constexpr uint8_t portC_ledstripDataBit{ B00001000 };           // port C bit 3 

#else
constexpr uint8_t ledstripDataBits{ B11000000 };                // port D bits 7 and 6
#endif 


// *** flash memory constants ***

const char str_build[] PROGMEM = "***spinning globe v1.2.1 ***\n";

const char str_empty16[] PROGMEM = "                ";
const char str_rotationOff[] PROGMEM = "rotation off";
const char str_freeRunning[] PROGMEM = "free running";
const char str_noPosSync[] PROGMEM = "wait for pos sync";
const char str_measuring[] PROGMEM = "measuring";
const char str_notLocked[] PROGMEM = "not locked";
const char str_locked[] PROGMEM = "locked";
const char str_notFloating[] PROGMEM = "not floating";
const char str_ErrDroppedGlobe[] PROGMEM = "E! dropped globe";
const char str_ErrStickyGlobe[] PROGMEM = "E! sticky globe";
const char str_ErrOverload[] PROGMEM = "E! overload";
const char str_ErrTemp[] PROGMEM = "E! temp too high";

const char str_rotTimeSet[] PROGMEM = "rot time>";
const char str_rotTimeAct[] PROGMEM = "rot t act";
const char str_syncError[] PROGMEM = "sync err ";
const char str_tLocked[] PROGMEM = "t locked ";
const char str_tFloat[] PROGMEM = "t float  ";
const char str_tempAct[] PROGMEM = "temp     ";
const char str_avgDutyC[] PROGMEM = "avg duty c";
const char str_vertPos[] PROGMEM = "vert pos>";
const char str_errSigVar[] PROGMEM = "vp avg err";
const char str_intTerm[] PROGMEM = "integ term";
const char str_avgPhase[] PROGMEM = "rot lag ";                 // with respect to (stable) magnetic field rotation
const char str_isrTime[] PROGMEM = "adc isr t";
const char str_procLoad[] PROGMEM = "proc load";
const char str_gain[] PROGMEM = "gain!";
const char str_intTimeCst[] PROGMEM = "int t c!";
const char str_difTimeCst[] PROGMEM = "dif t c!";
const char str_phaseAdj[] PROGMEM = "phas adj!";

const char str_editValue[] PROGMEM = "  << +, - to change value, E to end edit, C to cancel";
const char str_editValueWithDefault[] PROGMEM = "  << +, - to change value, R for reset value, E to end edit, C to cancel";     // during edit
const char str_help1[] PROGMEM = "Type + or - to show next/previous, E to edit value, S to show or stop live values, A to show all values, T for time stamp,";
const char str_help2[] PROGMEM = "LC0..5 to change led strip cycle (0 = off), LT1..4 to change led strip cycle time (1 = fastest), M0..1 to measure (step) response, ? for help";
const char str_cmdError[] PROGMEM = "== Not a valid command or parameter";
const char str_showLive[] PROGMEM = "== Show Live";
const char str_stopLive[] PROGMEM = "== Stop Live";
const char str_colorCycle[] PROGMEM = "== led strip color cycle ";
const char str_timeStamp[] PROGMEM = "== Time stamp ";
const char str_stepResponse[] PROGMEM = "== (Step) response (ms;hall;ctr)";
const char str_stepResponseEnd[] PROGMEM = "== (Step) response end";
const char str_eventsMissed[] PROGMEM = "event(s) missed !";
const char str_programMode[] PROGMEM = "PROGRAM MODE";

const char str_fmtTime[] PROGMEM = "%ldd %02ld:%02ld:%02ld %03ld";
const char str_fmt3unsignedInteger[] PROGMEM = "%u;%u;%u";
const char str_fmtDayHour[] PROGMEM = "%3ldd%2ldh";
const char str_fmtHourMinute[] PROGMEM = "%3ldh%2ldm";

#if test_showEventStats 
const char str_eventMaxStats[] PROGMEM = "event max stats: events pending %d, mem size %d";
#endif

const char* const paramLabels[] PROGMEM = { str_rotTimeSet, str_rotTimeAct, str_syncError, str_tLocked, str_tFloat, str_tempAct, str_avgDutyC,
    str_vertPos, str_errSigVar, str_intTerm, str_avgPhase, str_isrTime, str_procLoad, str_gain, str_intTimeCst, str_difTimeCst, str_phaseAdj };


// *** user selectable parameter values ***

constexpr int paramNo_rotTimes{ 0 }, paramNo_hallmVoltRefs{ 7 };
constexpr int paramNo_gainAdjust{ 13 }, paramNo_intTimeConstAdjust{ 14 }, paramNo_difTimeConstAdjust{ 15 }, paramNo_phaseAdjust{ 16 };      // order in sequence of parameters

/*v1.0.1 high speed rotation times adapted or created new*/
long rotationTimes[] = { 0, 900, 1500, 3000, 4500, 6000, 7500, 9000, 12000 };           // must be divisible by 12 (steps), 0 means OFF
#if highAnalogGain                                                                      // TWO limits: voltage before opamp >= 100 mV, voltage after opamp <= 2700 mV (prevent output saturation)
long hallMilliVolts[] = { 1500, 1800, 2100, 2400, 2700 };                               // ADC setpoint expressed in mV (hall output after 15 x amplification by opamp, converted to mVolt)
#else
long hallMilliVolts[] = { 1000, 1200, 1400, 1600, 1800 };                               // ADC setpoint expressed in mV (hall output after 10 x amplification by opamp, converted to mVolt)
#endif

constexpr int paramValueCounts[] = { sizeof(rotationTimes) / sizeof(rotationTimes[0]), 0, 0, 0, 0, 0, 0,
    sizeof(hallMilliVolts) / sizeof(hallMilliVolts[0]), 0, 0, 0, 0, 0, 0,0,0,0 };       // 0 if no value list for parameter
constexpr long* paramValueSets[] = { rotationTimes, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
    hallMilliVolts, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr };  // nullptr if no value list for parameter
constexpr long parameterEditable{ 0b11110000010000001 };                                                // LSB: first parameter in list

// initialize array with selected values
int ParamsSelectedValuesOrIndexes[] = { 2, -1, -1, -1, -1, -1, -1, 0, -1, -1, -1, -1, -1, 0,0,0,0 };    // -1 if display only (no changeable parameter); otherwise default value - needed in case the eeprom is not used to store spinning globe presets 

constexpr int paramCount = sizeof(paramLabels) / sizeof(paramLabels[0]);

bool paramChangeMode{ false };
long paramNo{ 0 };
int paramValueOrIndex{ 0 };


// *** strings ***

constexpr char degreesSymbol[] = { ' ', 'C', 0 };                                   // character '°' not in LCD character set
constexpr char milliSecSymbol[] = { 'm', 's', 0 };                                  // character 'µ' not in LCD character set: use 'u'
constexpr char microSecSymbol[] = { 'u', 's', 0 };                                  // character 'µ' not in LCD character set: use 'u'

char s150[150], s30[30];                                                            // general purpose long and short character strings


#if (F_CPU != 16000000L)
#error code expects CPU frequency 16 MHz
#endif


// *** time and other measurements *** 

constexpr uint8_t keyBufferLength{ 3 };
constexpr long timer1PWMfreq{ 1000L };                                              // 1 KHz
constexpr long timer1PreScaler{ 8 };                                                // as set in setup();
constexpr long timer1ClockFreq{ F_CPU / timer1PreScaler };                          // 2 MHz
constexpr long timer1Top{ timer1ClockFreq / timer1PWMfreq / 2 };                    // timer counts up and down : 2000 steps, TOP =1000
constexpr long fastDataRateSamplingPeriods{ 1 << 7 };                               // in sampling periods (milliseconds, power of 2)
constexpr float samplingPeriod{ 1. / (float)timer1PWMfreq };                        // 1 millisecond sampling period, in seconds
constexpr long oneSecondCount{ 1000L }, blinkTimeCount{ 800 };                      // milliseconds
constexpr long spareTimeCount{ 500 };                                               // milliseconds

bool showLiveValues{ true };
bool forceWriteLedstripSpecs{ false };
uint8_t currentSwitchStates{};
uint8_t ISRevent{ eNoEvent };                                                       // current ISR event retrieved for processing in main loop
int userCommand{ uNoCmd }, commandParam1, commandParam2;
float idleLoopMicrosSmooth{ 0 }, magnetOnCyclesSmooth{ 0 }, ISRdurationSmooth{ 0 }; // values smoothed by first order filter
float errSignalMagnitudeSmooth{ 0 };

// interface between ISR and main
volatile bool resetHWwatchDog{ false };                                             // periodic reset of hardware watchdog (disabling magnets)
volatile bool forceStatusEvent{ false };
volatile bool ISRoccurred{ false };                                                 // for idle time counting
volatile bool highLoad{ false };
volatile bool ADCisTemp{ false };                                                   // comm. between timer 1 overflow ISR and ADC conversion complete ISR
volatile bool useButtons{ false };                                                  // signals SW3 to SW0: interpret as buttons ?
volatile bool switchesSetRotationTime{ false }, switchesSetLedstrip{ false };       // signals SW3 to SW0: interpret as switches for specific settings ?
volatile bool switchesSetHallmVoltRef{ false };
volatile int8_t keyBuffer[keyBufferLength]{ 0 };                                    // can hold negative key codes
volatile uint8_t rotationStatus{ rotNoPosSync }, errorCondition{ errNoError };
volatile uint8_t switchStates{ 0 }, prevSwitchStates{ 0 };                          // current and previous state of the board switches / buttons
volatile uint8_t keysAvailable{ 0 };                                                // No of keys available in board key buffer
volatile unsigned int idleLoopNanos500{ 0 };                                        // at least reset every mS = 1000 micro seconds: will not overflow
volatile unsigned int millis16bits{ 0 };
volatile long milliSecond{ 0 }, second{ 0 };
volatile long tempSmooth{ 0 };                                                      // temperature smoothed by first order filter; long for speed, used by ISR


// *** PID controller ***

#if highAnalogGain
constexpr float analogGain{ 15. };
constexpr float presetGain{ 0.70 * 10. / analogGain };                              // PID: gain (total gain: gain x 1023 ADC steps / 5000 millivolt x analog gain)
constexpr float presetIntTimeCst{ 10.0 };                                           // PID: integrator  time constant (seconds) 
constexpr float presetDifTimeCst{ 0.023 };                                          // PID: differentiator time constant (seconds) 

constexpr long initialTTTintTerm{ (800 * analogGain) / 10 };                        // PID: initial value integrator term (for easier globe handling) --> depends on gain !
#else
constexpr float analogGain{ 10. };                                                  // 10. is the analog gain on the first board version
constexpr float presetGain{ 0.70 };                                                 // PID: gain (total gain: gain x 1023 ADC steps / 5000 millivolt x analog gain)
constexpr float presetIntTimeCst{ 10.0 };                                           // PID: integrator  time constant (seconds)
constexpr float presetDifTimeCst{ 0.023 };                                          // PID: differentiator time constant (seconds)

constexpr long initialTTTintTerm{ 800 };                                            // PID: initial value integrator term (for easier globe handling) --> depends on gain !
#endif

// step sizes for user adjustments
constexpr float gainStepSize{ 0.02 };                                               // PID: gain step size
constexpr float intTimeCstStepSize{ 0.5 };                                          // PID: integrator time constant step size
constexpr float difTimeStepSize{ 0.0001 };                                          // PID: differentiator time constant step size

constexpr long maxTTTintTerm{ (long)(initialTTTintTerm * 1.5) };                    // PID: max. value integrator term

constexpr int gain_BinaryFractionDigits{ 8 };                                       // added TTTgain accuracy (binary fraction digits) because of small TTTgain
constexpr int TTTintFactor_BinaryFractionDigits{ 18 };                              // added TTTintFactor accuracy (binary fraction digits) because of small TTTintFactor                           
constexpr int TTTdifFactor_BinaryFractionDigits{ 3 };                               // added TTTdifFactor accuracy (binary fraction digits) because of small TTTdifFactor

constexpr long ADCsteps{ 1024L };                                                   // globe vertical position sensor: resolution (10 bit ADC)
constexpr uint16_t printPIDperiod{ 20000 }, PIDstepTime{ 1000 };                    // for step response measurement

// interface between ISR and main
volatile int gainAdjustSteps{ 0 };                                                  // as stored in eeprom
volatile int intTimeCstAdjustSteps{ 0 };
volatile int difTimeCstAdjustSteps{ 0 };

volatile float gain{ };
volatile float intTimeCst{  };
volatile float difTimeCst{  };

volatile long TTTgain{};                                                            // true three term controller
volatile long TTTintFactor{};
volatile long TTTdifFactor{};
volatile float TTTintTimeCst{};                                                     // TTT integrator time constant
volatile float TTTdifTimeCst{};                                                     // TTT differentiator time constant

volatile long maxTTTallTerms{};


volatile long targetHallRef_ADCsteps{};                                             // globe vertical position ref (controller reference input) to reach after changing ref, in ADC steps 
volatile long hallRef_ADCsteps{};                                                   // current globe vertical position ref (controller reference input) in ADC steps 
volatile uint32_t firstFullAccIntTerm{};                                            // for printing step response (allows PC simulations)
volatile uint16_t printPIDtimeCounter{ printPIDperiod + 1 };                        // for step response measurement; printPIDperiod + 1 : 'printing stopped'   
volatile bool applyStep{ false };                                                   // apply step when measuring (step) response


// *** globe rotation controller ***

// unlocked globe rotation: rotation time / target rotation time ratio ranges
// --------------------------------------------------------------------------

// 1. slow down/speed up ranges. If measured globe rotation time / target rotation time ratio is...:
//    - less than lower limit: adjust magnetic field rotation time with a calculated ratio (slow down)
//    - greater than upper limit: adjust magnetic field rotation time with a calculated ratio (speed up)
//    - within limits: adjustment magnetic field rotation time to target rotation time

// array values are specified for high (T<3s) and low (T>=3s) target rotation speeds, respectively 
constexpr float speedAdjustCenterRange_low[2]{ 0.95, 0.8 };
constexpr float speedAdjustCenterRange_high[2]{ 1.05, 1.2 };

// 2. autolock ranges
//    when measured globe rotation time / target rotation time ratio is INSIDE the limits specified here during a number of successive rotations, then change rotation status to 'locked'
//    these limits must be set NARROWER than the slow down/speed up ranges !

// array values are specified for high (T<3s) and low (T>=3s) target rotation speeds, respectively 
constexpr float autoLockRange_low[2]{ 0.98, 0.9 };
constexpr float autoLockRange_high[2]{ 1.02, 1.1 };


// unlocked globe rotation: rotation speed ratio's used to calculate new rotation time
// -----------------------------------------------------------------------------------

// first the speed ratio to use is calculated by interpolating the speed ratio's set for fast (T=1s) and slow (T=12S) rotation times, using the measured rotation time 
// new rotation time to set is then calculated based on the (measured) rotation time and the calculated speed ratio
// NOTE: when rotation time / target rotation time ratio is in the center range (see above), magnetic field range is adjusted to target rotation time

// array values are specified for slowing down and speeding up, respectively
constexpr float speedRatioFastTurns[2]{ 0.8, 1.2 };                                 // rotation time 1 second (fast)  
constexpr float speedRatioSlowTurns[2]{ 0.9, 1.1 };                                 // rotation time 12 seconds (slow)


// unlocked globe rotation: calculation of globe rotation lag to set for next rotation
// -----------------------------------------------------------------------------------

// the globe rotation is lag is calculated using the lag for a rotation time of 1 second and a set slope   

// array values are specified for slowing down and speeding up, respectively
constexpr long globeRotationLag_1s[2]{ 100, 105 };                                  // rotation time 1 second (fast): rotation lag (degrees) 
constexpr long globeRotationLag_slope[2]{ 20, 55 };                                 // slope 


// -------------------------------------------------------------------------------------

constexpr long defaultStepTime{ 750L };                                             // one turn = stepTime * # steps, milliseconds;
constexpr long stepCount{ 12L };                                                    // #steps must be equivalent to 360 degrees AND 1 step must be equivalent to WHOLE number of degrees

// interface between ISR and main
volatile int rotationTimerSamplePeriod{};
volatile int phaseAdjustSteps{ 0 };                                                 // adjustment to cater for hall detector position changes (stored in eeprom, 0 to 179 2-degree steps)
volatile long targetStepTime{ defaultStepTime }, targetGlobeRotationTime{ targetStepTime * stepCount };
volatile long slowDown_maxGlobeRotationTime{};                                      // if rotation time lower than limit, then slow down 
volatile long speedUp_minGlobeRotationTime{};                                       // if rotation time higher than limit, then speed up
volatile long autoLock_minGlobeRotationTime{};                                      // if rotation time lower than limit, then set phase 
volatile long autoLock_maxGlobeRotationTime{};                                      // if rotation time lower than limit, then set phase 
volatile long stepTimeNewRotation{ targetStepTime };

// *** led strip dimming ***

constexpr uint8_t LSbrightnessItemCount{ 3 };                                       // max no of brightness values available for color led dimming 
constexpr uint8_t LSminBrightnessLevel{ 0x00 };                                     // min: 0 (LS off) 
constexpr uint8_t LSmaxBrightnessLevel{ 0xff };                                     // 0x7f or 0xff; ((value + 1)^2 / 256) - 1 = gamma corrected value = 0x3f or 0xff)     

// interface between ISR and main
volatile bool LSlongTimeUnit{ false };                                              // 128 ms instead of 1 ms
volatile uint8_t LScolorCycle, LScolorTiming;                                       // color cycle
volatile uint8_t LScolor[LSbrightnessItemCount];                                    // brightness levels (not yet assigned to specific colors or leds)
volatile uint8_t colorGammaCorrected[LSbrightnessItemCount + 1]{ 0xFF, 0x00, 0x00, 0x00 };    // ledstrip: for each led color 4 bytes: 0xFF and 3 8-bit RGB values, gamma corrected. Main led: first byte is not used 

volatile uint8_t LStransitionStops;                                                 // MUST be > 0 => set LSbrightnessFreezeTime = 0 if no transition stops desired

volatile long LSbrightnessTransitionTime;                                           // total brightness transition time (in a complete color cycle), in milliseconds - excludes 'frozen brightness' times                                           
volatile long LSbrightnessFreezeTime;                                               // total 'frozen brightness' time (summed up constant brightness time between all transitions), in milliseconds (minimum = 0)
volatile long LSbrightnessCycleTime;                                                // COMPLETE color cycle in milliseconds

volatile long LSminBrightnessTime;                                                  // if 3 colors: 1/3 of cycle time (only two primary colors at the same time - no white)
volatile long LSmaxBrightnessTime;                                                  // min 0: primary stronger than CMY), max 1/(no of colors) of cycle time (CMY stronger than primary) 
volatile long LSbrightnessDelay;                                                    // delay between brightness cycles (here: between to or three brightness values)

// number of brightness steps, including steps of min / max brightness (but excluding 'frozen brightness' time)
volatile long LSbrightnessUpDownSteps;
volatile long LSbrightnessMinLvlSteps;
volatile long LSbrightnessMaxLvlSteps;
volatile long LSbrightnessTransitionSteps;

volatile long LSscaledDelay;                                                        // delay between brightness cycles, scaled by total no of brightness steps

// initial values per brightness change counter: step size and delay between brightness cycle, scaled by total no of brightness steps (step size x no of brightness steps = total transition time)
volatile long LSbrightnessStepTimer[LSbrightnessItemCount];
volatile long LSbrightnessFreezeTimer;                                              // optional: skip first 'frozen brightness' step because a brightness is still missing (is starting from 'all brightness values OFF')
volatile long LSminBrightnessStepNo[LSbrightnessItemCount], LSmaxBrightnessStepNo[LSbrightnessItemCount];   // for counting min / max level steps

volatile uint8_t LSupdate;                                                          // update flag for leds per led group 
volatile uint8_t LSup;                                                              // initial dimming direction up for all brightness items (true if initially counting up OR in a max. brightness period)
volatile uint8_t LSminReached, LSmaxReached;


// *** for testing purposes ***

constexpr bool enableSafety{ true };                                                // enable safety checks lifting magnet ? (Note: temperature check lifting magnet is always ON)


// *** structures: communication between ISR and main ***

struct GreenwichData {                                                              // initialize members, awaiting first event
    long eventMilliSecond{ 0 }, eventSecond{ 0 };
    long globeRotationTime{ 0 };
    long rotationOutOfSyncTime{ 0 };
    long greenwichLag{ 0 };                                                         // versus (steady) magnetic field rotation (coils), when globe rotation is locked to it
    long lockedRotations{ 0 };
};

struct StatusData {                                                                 // initialize members, awaiting first event
    long eventMilliSecond{ 0 }, eventSecond{ 0 };
    uint8_t rotationStatus{ rotNoPosSync }, errorCondition{ errNoError };
    bool isGreenwich{ false };
    bool isFloating{ false };
};

struct SecondData {                                                                 // initialize members, awaiting first event
    long eventSecond{ 0 };
    long liftingSecond{ 0 }, lockedSecond{ 0 };
    long realTTTintTerm{ 0 };
};

struct FastRateData {
    long sumIdleLoopMicros;
    long sumISRdurations;
    long sumMagnetOnCycles;
    long sumADCtemp;
    long sumErrSignalMagnitude;
};

struct LedstripData {
    uint8_t LSupdate, LSmaxReached, LSminReached;
    uint8_t LScolor[LSbrightnessItemCount];
};

struct StepResponseData {
    uint16_t count, hallReading_ADCsteps, TTTcontrOut;
};

struct EventStats {
    uint8_t* activeMsgPtr{ nullptr };
    uint8_t activeEventType{ eNoEvent };
    uint8_t eventsPending{ 0 }, largestEventsPending{ 0 };                          // No of ISR events currently logged for processing in main loop 
    unsigned int eventBufferBytesUsed{ 0 }, largestEventBufferBytesUsed{ 0 };       // No of ISR events logged for processing in main loop: all-time max
    long eventsMissed{ 0 };                                                         // keeps track of events missed (not used at this stage)
};


// *** class: millis and micros function based on timer1 and own time counting logic (millis and micros < one second, count seconds) ***

class MyTime {
private:
    long m_milliSecond, microSecond;
    unsigned int volatile m_nanoSecond500, m_nextNanoSecond500;

public:
    long millis(long* secondsPtr = nullptr) {                           // return millis in current second & seconds as well (run time in micros = seconds * 1E6 + micros)
        uint8_t oldSREG = SREG;
        cli();                                                          // retrieve seconds and mS while interrupts disabled 
        if (secondsPtr != nullptr) { *secondsPtr = second; }            // total
        SREG = oldSREG;
        return milliSecond;                                             // 0 to 999
    }

    // execution time is very close to 20 microSeconds (16MHz clock)
    long micros(long* secondsPtr = nullptr) {                           // return micros in current second & seconds as well (run time in micros = seconds * 1E6 + micros) 
        uint8_t oldSREG = SREG;
        cli();                                                          // retrieve seconds and mS, read timer while interrupts disabled 
        if (secondsPtr != nullptr) { *secondsPtr = second; }            // total running
        m_milliSecond = milliSecond;                                    // 0 to 999

        bool oldTOV1 = (TIFR1 & _BV(TOV1));                             // timer 1 interrupt pending ? 

        m_nanoSecond500 = TCNT1;                                        // one read takes 8 clock cycles (2 LDS and 2 STS instructions -> 4 * 2 = 8 clock cycles = 500 nS with 16 MHz clock) 
        m_nextNanoSecond500 = TCNT1;                                    // take 2 counter readings (500 nS steps) to determine slope (difference between 2 readings is exactly -1 or +1)

        if (m_nanoSecond500 > m_nextNanoSecond500) { m_nanoSecond500 = 2 * timer1Top - m_nextNanoSecond500; }   // counting down (500 nS per cycle)

        if (TIFR1 & _BV(TOV1)) {                                        // timer 1 interrupt pending: milliSecond is not yet updated
            m_milliSecond++;                                            // milliSecond will be updated when ISR runs
            if (!oldTOV1) { m_nanoSecond500 = m_nextNanoSecond500; }    // two microsecond readings around overflow (zero) point: prevent "2 * timer1Top - m_nanoSecond500" calculation)
        }

        SREG = oldSREG;

        // calculate micro seconds in current second: 0 to 999999 (1 period = 500 nS) 
        microSecond = ((long)(m_nanoSecond500 >> 1)) + (m_milliSecond << 10) - (m_milliSecond << 4) - (m_milliSecond << 3);
        if (microSecond >= 1000000L) {
            microSecond = microSecond - 1000000L;
            if (secondsPtr != nullptr) { (*secondsPtr)++; }
        }
        return microSecond;
    }
};


// *** class: MyEvents ***

class MyEvents {
private:
    const uint8_t eventBufferSize{ 255 };                                                   // max 255

    uint8_t* eventBuffer{ nullptr };                                                        // dynamic memory to store event messages until they are processed
    uint8_t* oldestMessageStartPtr{ nullptr }, * newestMessageStartPtr{ nullptr };
    EventStats eventStats;

public:
    // interface between ISR and main

    MyEvents();                                                                             // constructor
    bool addChunk(uint8_t eventType, uint8_t newChunkSize, uint8_t** messagePtrPtr);
    bool removeOldestChunk(bool remove);
    bool isEventsWaiting();
    void snapshot(EventStats* eventSnapshotPtr);
};


MyEvents::MyEvents() {  // constructor
    eventBuffer = (uint8_t*)malloc(eventBufferSize);
}


// *** reserve space in the event message buffer for a new event message ***

bool MyEvents::addChunk(uint8_t eventType, uint8_t newChunkSize, uint8_t** messagePtrPtr) {         // prevent interference with another call to addChunk() - which may be called from ISR;
    bool OK{ false };
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        uint8_t* newestEndPtr{ nullptr };
        uint8_t** holdLastStartPtrPtr;

        newChunkSize += 4;                                                                          // add 4 bytes for event type, length and pointer to next event message (ok if event is cue only and carries no data)

        bool isEmpty = (oldestMessageStartPtr == nullptr);                                          // currently any event message logged ?
        if (!isEmpty) { newestEndPtr = newestMessageStartPtr + *(newestMessageStartPtr + 1) - 1; }  // pointer to last byte of current event message
        bool wrappedAround = (!isEmpty) && (oldestMessageStartPtr > newestEndPtr);

        uint8_t freeChunkSize = isEmpty ? eventBufferSize : (wrappedAround ? (oldestMessageStartPtr - newestEndPtr) - 1 : eventBuffer + eventBufferSize - newestEndPtr - 1);
        OK = (freeChunkSize >= newChunkSize);
        bool wrap{ false };
        if ((!OK) && (!wrappedAround) && (!isEmpty)) {                                              // append at end not possible: check if wraparound possible
            uint8_t freeChunkSize = oldestMessageStartPtr - eventBuffer;
            OK = (freeChunkSize >= newChunkSize);
            wrap = OK;
        }

        if (OK) {                                                                                   // room available to store next message
            if (isEmpty) { oldestMessageStartPtr = eventBuffer; newestMessageStartPtr = eventBuffer; }

            else {
                holdLastStartPtrPtr = (uint8_t**)(newestMessageStartPtr + 2);
                newestMessageStartPtr = wrap ? eventBuffer : newestMessageStartPtr + *(newestMessageStartPtr + 1);
                *holdLastStartPtrPtr = newestMessageStartPtr;                                       // pointer from (now) previously created event to newly created event
            }

            *newestMessageStartPtr = eventType;                                                     // newly created event: set event type
            *(newestMessageStartPtr + 1) = newChunkSize;                                            // newly created event: set message length (including 4-byte header)
            holdLastStartPtrPtr = (uint8_t**)(newestMessageStartPtr + 2);
            *holdLastStartPtrPtr = nullptr;                                                         // pointer to next event: set to nullptr (there is no next event)

            *messagePtrPtr = newestMessageStartPtr + 4;                                             // oldest event (next event to process): pointer to optional event message (NOT to the 4-byte header)

            eventStats.activeMsgPtr = oldestMessageStartPtr + 4;
            eventStats.activeEventType = *oldestMessageStartPtr;                                    // oldest event: event type
            eventStats.eventsPending++;
            // update statistics
            eventStats.largestEventsPending = max(eventStats.largestEventsPending, eventStats.eventsPending);
            eventStats.eventBufferBytesUsed += newChunkSize;
            eventStats.largestEventBufferBytesUsed = max(eventStats.largestEventBufferBytesUsed, eventStats.eventBufferBytesUsed);

        #if test_showEventStats 
            Serial.println(); Serial.print("+ ;");  Serial.println((int)newestMessageStartPtr);
        #endif
        }

        else {
            eventStats.eventsMissed++;
        #if test_showEventStats 
            Serial.println(); Serial.println("missed");
        #endif
        }
    }
    return OK;
}


// *** release space occupied by the oldest message in the event message buffer ***

bool MyEvents::removeOldestChunk(bool remove) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {                                                                                             // prevent interference with addChunk() - which may be called from ISR;
        if ((oldestMessageStartPtr == nullptr) || (!remove)) { return false; }                                                      // nothing to remove: is empty

        eventStats.eventsPending--;
        eventStats.eventBufferBytesUsed -= *(oldestMessageStartPtr + 1);                                                            // before pointer update

    #if test_showEventStats 
        Serial.println(); Serial.print("- ;");  Serial.println((int)oldestMessageStartPtr);
    #endif

        if (oldestMessageStartPtr == newestMessageStartPtr) { oldestMessageStartPtr = nullptr; newestMessageStartPtr = nullptr; }   // remove last remaining
        else { oldestMessageStartPtr = *(uint8_t**)(oldestMessageStartPtr + 2); }                                                   // remove oldest (which is not the last remaining)

        eventStats.activeMsgPtr = (oldestMessageStartPtr == nullptr) ? nullptr : oldestMessageStartPtr + 4;                         // pointer to optional structure; after pointer update
        eventStats.activeEventType = (oldestMessageStartPtr == nullptr) ? eNoEvent : *oldestMessageStartPtr;

    }
    return true;
}


bool MyEvents::isEventsWaiting() {
    bool eventsArePending{ false };
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {                             // prevent interference with addChunk() - which may be called from ISR;
        eventsArePending = (eventStats.eventsPending > 0);
    }
    return eventsArePending;
}


void MyEvents::snapshot(EventStats* eventSnapshotPtr) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {                             // prevent interference with addChunk() - which may be called from ISR;
        *eventSnapshotPtr = eventStats;
    }
}


// *** objects ***

LiquidCrystal lcd(D3_LCDregSelPin, B2_LCDenablePin, 4, 5, 6, 7);    // define I/O pins (LCD RS, LCD enable, data = PORT D bits 4 to 7) 
MyTime myTime;
MyEvents myEvents;

EventStats eventSnapshot;

// retain values from event message buffer
StatusData statusData;
GreenwichData greenwichData;
SecondData secondData;

// pointers into event message buffer
FastRateData* fastRateDataPtr;
LedstripData* ledstripDataPtr;
StepResponseData* stepResponseDataPtr;


// *** forward declarations ***

void getEventOrUserCommand();                                   // retrieve an event or a user command - exit if nothing available
void getISRevent();                                             // copy one ISR event (Greenwich, status change, second cue, blink, fast rate data events, ...) for processing, if available
void getCommand();                                              // parse one user command - exit if no more characters available or command is complete 
void processEvent();                                            // process one event, if available
void processCommand();                                          // process one user command, if available
void checkSwitches(bool forceSwitchCheck = false);              // if SW3 to SW0 to be interpreted as switches only (instead of buttons)
void writeStatus();                                             // print on event or on command
void writeParamLabelAndValue();                                 // print on event or on command
void writeLedStrip();                                           // apply gamma correction and write led strip
void LSout(uint8_t* led, uint8_t* ledstripMasks);               // write led strip
void LSoneLedOut(uint8_t holdPortC, uint8_t* LedData, uint8_t ledMask = B111);      // write one led strip led
void idleLoop();

void formatTime(char* s, long totalSeconds, long totalMillis, long* days = nullptr, long* hours = nullptr, long* minutes = nullptr, long* seconds = nullptr);
void readKey(char* keyAscii);                                   // from Serial interface and on board keys
void saveAndUseParam();
void fetchParameterValue(char* s, long paramNo, int paramValueOrIndex);
void setPIDcontroller();
void setRotationTime(int paramValueOrIndex, bool init = false);
void setColorCycle(uint8_t newColorCycle, uint8_t newColorTiming, bool initColorCycle = false);



void setup()
{
    // *** disable watchdog, open serial port and LCD ***

    wdt_disable();
    Serial.begin(1000000);                                      // baud rate as entered
    lcd.begin(16, 2);                                           // 16 characters, 2 rows


    // *** hardware: initialize ports ***

    // PORT A & C (same Nano pins)
    pinMode(A0_liftHallPin, INPUT);                             // A0: hall sensor analog input pin
    pinMode(A1_temperaturePin, INPUT);                          // A1: temperature sensor analog input pin
    pinMode(A2_IONotEnablePin, OUTPUT);                         // port C bit 2 (Nano pin A2): I/O channel not enable (74HCT138 decoder)  

#if (boardVersion == 100)
    pinMode(A3_ledstripDataPin, OUTPUT);
#else
    pinMode(A3, INPUT_PULLUP);                                  // not used
#endif

    pinMode(A4, INPUT_PULLUP);                                  // not used
    pinMode(A5, INPUT_PULLUP);                                  // not used

    PORTC = (PORTC | portC_IOdisableBit);                       // disable I/O hardware

    // PORT B 
    pinMode(B1_OC1Apin, OUTPUT);                                // timer 1 channel A to output pin

    DDRB = DDRB | portB_IOchannelSelectBitMask;                 // set other port B output pins
    PORTB = PORTB | portB_IOchannelSelectBitMask;


    // *** read initial switch status and adapt settings accordingly ***

    PORTD = PORTD | B11111100;                                                          // PORT D pins 2 to 7: prepare to enable pull ups   
    DDRD = DDRD & B00000011;                                                            // PORT D pins 2 to 7: inputs (pins 0 and 1: serial I/O)
    PORTB = ((PORTB & ~portB_IOchannelSelectBitMask) | portB_switchesBufferSelect);     // PORT B: select switch buffers
    PORTC = (PORTC & ~portC_IOdisableBit);                                              // enable switch buffers

    switchStates = PIND;                                                                // read switches twice (stall) - allow for setup time (see Atmel data sheet)
    switchStates = PIND & pinD_switchStateBits;                                         // (read twice)

    PORTC = (PORTC | portC_IOdisableBit);                                               // disable switch buffers
    PORTB = PORTB | portB_IOchannelSelectBitMask;
    DDRD = DDRD | B11111100;                                                            // PORT D pins 2 to 7: outputs (pins 0 and 1: serial I/O)


    // *** retrieve settings from eeprom and switches ***
    uint8_t cnt{ 0 };
    uint8_t eepromValue{ 0 };
    uint8_t ledstripCycle{}, ledstripTiming{};

    cli();

    // read rotation time from eeprom and set
    eepromValue = eeprom_read_byte((uint8_t*)0);                                        // restore globe rotation time from eeprom
    cnt = paramValueCounts[paramNo_rotTimes];                                           // No of defined rotation times 
    eepromValue = ((eepromValue < 0) || (eepromValue >= cnt)) ? 0 : eepromValue;
    ParamsSelectedValuesOrIndexes[paramNo_rotTimes] = eepromValue;
    setRotationTime(eepromValue, true);                                                 // set rotation time and store in eeprom


    // read globe vertical position setpoint from eeprom 
    // NEW version 1.0.3: read gain, integration & differentiation time constants from eeprom
    // set PID controller
    eepromValue = eeprom_read_byte((uint8_t*)1);
    cnt = paramValueCounts[paramNo_hallmVoltRefs];                                      // No of defined hall setpoints in millivolt
    eepromValue = ((eepromValue < 0) || (eepromValue >= cnt)) ? 0 : eepromValue;
    ParamsSelectedValuesOrIndexes[paramNo_hallmVoltRefs] = eepromValue;
    long hallmVoltRef = hallMilliVolts[eepromValue];                                    // globe vertical position ref in mVolt (after analog amplification)
    targetHallRef_ADCsteps = (ADCsteps * hallmVoltRef) / 5000L;                         // globe vertical position ref in ADC steps
    hallRef_ADCsteps = targetHallRef_ADCsteps;

    eepromValue = eeprom_read_byte((uint8_t*)4);
    gainAdjustSteps = (eepromValue >= 31) ? 31 : eepromValue;                           // preset gain corresponds to gainAdjustSteps mid value (16)  
    ParamsSelectedValuesOrIndexes[paramNo_gainAdjust] = gainAdjustSteps;

    eepromValue = eeprom_read_byte((uint8_t*)5);
    intTimeCstAdjustSteps = (eepromValue >= 31) ? 31 : eepromValue;                     // preset time constant corresponds to gainAdjustSteps mid value (16)
    ParamsSelectedValuesOrIndexes[paramNo_intTimeConstAdjust] = intTimeCstAdjustSteps;

    eepromValue = eeprom_read_byte((uint8_t*)6);
    difTimeCstAdjustSteps = (eepromValue >= 31) ? 31 : eepromValue;                     // preset time constant corresponds to gainAdjustSteps mid value (16)
    ParamsSelectedValuesOrIndexes[paramNo_difTimeConstAdjust] = difTimeCstAdjustSteps;

    setPIDcontroller();


    // NEW version 1.0.3: read phase adjustment for coils rotation start delay (non-locked rotation) from eeprom and store in memory     
    eepromValue = eeprom_read_byte((uint8_t*)7);
    phaseAdjustSteps = (eepromValue >= 179) ? 179 : eepromValue;                        // phase adjustment in 2-degree increments (0 to 358 degrees)                                 
    ParamsSelectedValuesOrIndexes[paramNo_phaseAdjust] = phaseAdjustSteps;


    // read led strip cycle & timing from eeprom and set 
    eepromValue = eeprom_read_byte((uint8_t*)2);                                        // b7654 = led strip cycle time, b3210 = led strip cycle

    /* not used but could be used for a 3-second cue
    eepromValue = eepromValue + (eeprom_read_byte((uint8_t*)3) & (uint8_t)0x01);        // if running time after previous reset was small: switch to next led strip color cycle
    */

    ledstripCycle = eepromValue & (uint8_t)0x0F;
    ledstripTiming = eepromValue >> 4;
    ledstripCycle = ((ledstripCycle < cLedstripOff) || (ledstripCycle > cRedGreenBlue)) ? cLedstripOff : ledstripCycle;
    ledstripTiming = ((ledstripTiming < cLedstripVeryFast) || (ledstripTiming > cLedStripVerySlow)) ? cLedstripVeryFast : ledstripTiming;
    setColorCycle(ledstripCycle, ledstripTiming, true);                                 // set led strip cycle and timing and store in eeprom

    // DIP switches 2 to 5 (signals SW3 to SW0): interpret as buttons if all 4 switches OFF (= 'high') after reset. If NOT all OFF, then interpret as switches and enter program mode (do not connect buttons then)
    // if in program mode, a selected setting restored from eeprom will be overridden 
    switchesSetLedstrip = (switchStates & pinD_keyBits) == (uint8_t)0x00;               // signals SW3 to SW0: interpret as switches and use to program led strip
    switchesSetRotationTime = ((switchStates & pinD_keyBits) >> 2) == (uint8_t)0x01;    // signals SW3 to SW0: interpret as switches and use to program rotation time
    switchesSetHallmVoltRef = ((switchStates & pinD_keyBits) >> 2) == (uint8_t)0x02;    // signals SW3 to SW0: interpret as switches and use to program globe vertical position reference
    useButtons = (switchStates & pinD_keyBits) == pinD_keyBits;                         // signals SW3 to SW0: interpret as buttons if all corresponding 4 switches OFF (= 'high') after reset (if not all OFF, then do not connect buttons)
    checkSwitches(true);                                                                // adapt settings according to switch states - note that switch 1 (signal SW4) is currently not used

    /* not used but could be used for a 3-second cue
    eeprom_update_byte((uint8_t*)3, (uint8_t)1);                                        // flag that reset took place
    */
    sei();

    // initial setting to display: rotation time, except if currently in program mode
    paramNo = switchesSetHallmVoltRef ? paramNo_hallmVoltRefs : paramNo_rotTimes;
    paramValueOrIndex = ParamsSelectedValuesOrIndexes[paramNo];


    // *** do a first temp reading here and assign it to temperature filter output, to avoid slow temperature ramp up ***

    tempSmooth = (((long)analogRead(A1_temperaturePin) * 50000L - (5000L << 10)) >> 10);// convert to degrees Celsius x 100 (multiply or divide by 1024 = ADC resolution: shift 10 bits instead)


    // *** init serial and LCD ***

    while (!Serial);
    Serial.println();

    if (!useButtons) {
        Serial.println(strcpy_P(s150, str_programMode));
        Serial.println();
    }
    Serial.println(strcpy_P(s150, str_build));
    Serial.println(strcpy_P(s150, str_help1));
    Serial.println(strcpy_P(s150, str_help2));

    lcd.clear();
    lcd.noAutoscroll();


    // *** enable watchdog timer (2 seconds) ***

    wdt_enable(WDTO_2S);


    // *** setup timer 1 (16 bit) for phase correct PWM 1 Khz and enable overflow interrupt ***

    // timer 1 is used as timebase AND to generate PWM for lifting magnet
    // Prescaler 8 (16MHz / 8 = 2 MHz clock => T = 500 nanoS), 1 kHz = 2 Mhz / (2 * 1000 = 2 * TOP value) 
    TCCR1A = _BV(COM1A1) | _BV(WGM11);                      // COM1A1 set: clear OC1A pin on compare match when up-counting, set when down-counting
    TCCR1B = _BV(WGM13) | _BV(CS11);                        // WGM13 & WGM11 set: PWM, phase correct, TOP = ICR1 register; CS11: prescaler factor 8 
    ICR1 = timer1Top;                                       // counter TOP value 
    do {} while (TCNT1 < 100);                              // prevent first of two timer interrupts in succession after reset, with ADC re-trigger before ADC interrupt
    TIMSK1 = _BV(TOIE1);                                    // enable overflow interrupt for this timer 
}


void loop()
{
    getEventOrUserCommand();                                // get ONE event or assembled user command, exit anyway if none available
    processEvent();                                         // process event, if available
    processCommand();                                       // process command, if available
    checkSwitches();                                        // if SW3 to SW0 to be interpreted as switches only (instead of buttons)
    writeStatus();                                          // print status to Serial and LCD (if connected)
    writeParamLabelAndValue();                              // print parameter label and value to Serial and LCD (if connected)
    writeLedStrip();                                        // write led strip on event      
    myEvents.removeOldestChunk(ISRevent != eNoEvent);       // has an event been processed now ? remove from message queue

    wdt_reset();                                            // reset watchdog timer
    resetHWwatchDog = true;                                 // allow ISR to set ISR-IN-EXEC signal high then low (set low again in ISR). Tmax around 400mS                                                  
    idleLoop();                                             // only for idle time measuring. Results in slight jitter on start of ISR (+/- 10 micros)
}


// *** get either an event or a user command, but not both at the same time

void getEventOrUserCommand() {
    ISRevent = eNoEvent;
    userCommand = uNoCmd;
    getISRevent();                                          // retrieve an event for processing, if available
    if (ISRevent == eNoEvent) { getCommand(); }             // if no event waiting to be processed and character(s) available, parse until command is complete or out of characters 
}


// *** process a single ISR event ***

void getISRevent() {

    // select an ISR event (greenwich, status change, second cue, blink, fast rate data events, ...) for processing
    // note that pressing and releasing HW buttons results in updating a character buffer, NOT in setting an event

    myEvents.snapshot(&eventSnapshot);                                      // current event status (active event data & statistics)
    ISRevent = eventSnapshot.activeEventType;                               // event type to process now (oldest), if any

    if (ISRevent == eNoEvent) {                                             // do nothing 
    }
    else if (ISRevent == eStatusChange) {                                   // status change event ? copy event message (needed later)
        statusData = *(StatusData*)(eventSnapshot.activeMsgPtr);
    }
    else if (ISRevent == eGreenwich) {                                      // Greenwich position event ? copy event message (needed later)
        greenwichData = *(GreenwichData*)(eventSnapshot.activeMsgPtr);
    }
    else if (ISRevent == eSecond) {                                         // second tick event ? copy event message (needed later)
        secondData = *(SecondData*)(eventSnapshot.activeMsgPtr);
    }
    else if (ISRevent == eFastRateData) {                                   // 'fast rate data logging' event ? set pointer to message
        fastRateDataPtr = (FastRateData*)(eventSnapshot.activeMsgPtr);
    }
    else if (ISRevent == eLedstripData) {                                   // led strip event ? set pointer to message
        ledstripDataPtr = (LedstripData*)(eventSnapshot.activeMsgPtr);
    }
    else if (ISRevent == eStepResponseData) {                               // step response event ? set pointer to message
        stepResponseDataPtr = (StepResponseData*)(eventSnapshot.activeMsgPtr);
    }
    else if ((ISRevent == eBlink) || (ISRevent == eSpareNoDataEvent1)) {    //  time cue event ?
        // do nothing here (no data associated with event)
    }
}


// *** parse multiple characters until either a command is complete OR no characters available (even if command is not yet complete) ***

void getCommand() {

    // all user commands are 1 character, without the need for a CR or LF (both are ignored) because globe buttons do not have these keys
    // commands can take single-character parameters (currently, only one parameter, but this can be extended)
    // commands come either from key presses read from within the ISR or from the Serial interface
    // exit even if command is not complete, and  continue assembly in next main loop

    // command -1: no command to execute
    // commands 0 - 99: without parameters
    // command 100 - 199: with one parameter
    // command 200 - 299: with two parameters
    // command 999: command error

    // commandState: 0 = wait for new command, 1 = wait for command parameter 1, 9 = command complete, 10 = command error


    constexpr bool enterCmdUsingSeparators{ false };    // enter with separators: not useful because Globe keyboard has no keys to type in separators and CR
    char keyAscii{ 0 };

    static int commandBuffer{ 0 }, commandParam1Buffer{ 0 }, commandParam2Buffer{ 0 }, commandState{ 0 };

    userCommand = uNoCmd;                                                                                   // init: no assembled command yet

    do {
        readKey(&keyAscii);                                                                                 // from HW buttons AND from Serial

        if (keyAscii == 0) {
            return;                                                                                         // no character read
        }

        bool ignoreChar = ((keyAscii == (char)0x20) || (keyAscii == (char)0x0D) || (keyAscii == (char)0x0A)) && !enterCmdUsingSeparators; // space, CR, LF characters

        if ((keyAscii == (char)0x1B) || (keyAscii == '/')) {                                                // use ESC or '/' as abort characters
            commandState = 0;                                                                               // abort: wait for new command
        }

        else if (!ignoreChar) {
            switch (commandState) {
                case 0:                                                                                     // currently no command: read command
                    commandBuffer = uNoCmd;
                    if (keyAscii == '-') { commandBuffer = uPrevious; }                                     // previous
                    else if (keyAscii == '+') { commandBuffer = uNext; }                                    // next
                    else if ((keyAscii == 'e') || (keyAscii == 'E')) { commandBuffer = uEdit; }             // enter edit / end edit
                    else if ((keyAscii == 'c') || (keyAscii == 'C')) { commandBuffer = uCancel; }
                    else if ((keyAscii == 'u') || (keyAscii == 'U')) { commandBuffer = uUp; }
                    else if ((keyAscii == 'd') || (keyAscii == 'D')) { commandBuffer = uDown; }
                    else if ((keyAscii == 'r') || (keyAscii == 'R')) { commandBuffer = uReset; }
                    else if ((keyAscii == 'a') || (keyAscii == 'A')) { commandBuffer = uShowAll; }          // show all parameters (Serial output only)
                    else if ((keyAscii == 's') || (keyAscii == 'S')) { commandBuffer = uLive; }             // show / stop live values
                    else if ((keyAscii == 't') || (keyAscii == 'T')) { commandBuffer = uTimeStamp; }        // time stamp (Serial output only) 
                    else if (keyAscii == '?') { commandBuffer = uHelp; }
                    else if ((keyAscii == 'm') || (keyAscii == 'M')) { commandBuffer = uMeasure; }          // measure (with or w/o step response)
                    else if ((keyAscii == 'l') || (keyAscii == 'L')) { commandBuffer = uLedstripSettings; } // select led strip color cycle or cycle timing

                    if ((commandBuffer == uPrevious) || (commandBuffer == uNext) || (commandBuffer == uDown) || (commandBuffer == uUp)
                        || (commandBuffer == uReset) || (commandBuffer == uEdit) || (commandBuffer == uCancel)) {
                        commandBuffer = commandBuffer + (paramChangeMode ? 20 : 0);
                    }

                    if ((commandBuffer >= 0) && (commandBuffer <= 99)) { commandState = 9; }                // these command take no parameter: signal 'command complete' if command detected
                    else if (commandBuffer >= 100) { commandState = 1; }                                    // command takes at least 1 parameter: signal 'look for parameter 1'
                    else { commandState = 10; }                                                             // unknown command: error
                    break;

                case 1:                                                                                     // command needing 1 parameter has been read: read parameter
                    // limit allowable characters for parameters (additional parameter checks depending on command: enter in execution block)
                    commandParam1Buffer = -1;                                                               // no command parameter 1 yet: read 
                    if ((keyAscii >= '0') && (keyAscii <= '9')) { commandParam1Buffer = ((uint8_t)keyAscii & B00001111); }
                    else if (keyAscii == '-') { commandParam1Buffer = 20; }
                    else if (keyAscii == '+') { commandParam1Buffer = 21; }
                    else if (keyAscii == '=') { commandParam1Buffer = 22; }
                    else if (((keyAscii >= 'A') && (keyAscii <= 'Z')) || ((keyAscii >= 'a') && (keyAscii <= 'z'))) { commandParam1Buffer = keyAscii & ~0x20; }

                    if (commandParam1Buffer >= 0) {
                        if (commandBuffer <= 199) { commandState = 9; }                                     // command takes 1 parameter only: signal 'command complete' if command detected
                        else { commandState = 2; }                                                          // command takes at least one additional parameter: signal 'look for parameter'
                    }
                    else { commandState = 10; }                                                             // illegal parameter value: error
                    break;

                case 2:
                    commandParam2Buffer = -1;                                                               // no command parameter 1 yet: read 
                    if ((keyAscii >= '0') && (keyAscii <= '9')) { commandParam2Buffer = ((uint8_t)keyAscii & B00001111); }

                    if (commandParam2Buffer >= 0) { commandState = 9; }
                    else { commandState = 10; }                                                             // illegal parameter value: error
                    break;

            }
        }

        if (commandState == 9) {                                                                            // command assembled
            userCommand = commandBuffer;

            commandParam1 = commandParam1Buffer;
            commandParam2 = commandParam2Buffer;
            commandState = 0;
        }

        else if (commandState == 10) {                                                                      // command error
            userCommand = uUnknownCmd;
            commandState = 0;
        }

    } while (commandState != 0);
}


// *** process an event (except printing) ***

void processEvent() {
    constexpr int averagingPeriodsMagnetLoad{ 10 };
    constexpr long maxOnCycles = (long)((averagingPeriodsMagnetLoad * 256 * fastDataRateSamplingPeriods * timer1Top) * 0.8);               // can normally not occur, but as this concerns safety ...
    constexpr int tempTimeCst_BinaryFractionDigits{ 16 };
    constexpr long tempTimeCst1024 = (long)(samplingPeriod * fastDataRateSamplingPeriods * (1L << tempTimeCst_BinaryFractionDigits));      // temp time cst * 2^n for added accuracy (long integer)

    static uint8_t fastRateDataEventCounter{ 0 };                                           // overflows at 255
    static long partialSumMagnetOnCycles{ 0 };
    static long movingSumMagnetOnCycles{ 0 };
    static long sumMagnetOnCycles[averagingPeriodsMagnetLoad] = { 0 };

    if (ISRevent == eNoEvent) { return; }

    // first order filters below are implemented as integrator with negative feedback: 
    // y(k)   =   sampling period / time constant * sigma(x(k) - y(k-1))   =   y(k-1) + sampling period / time constant * (x(k) - y(k-1))
    // samping period expressed in seconds / time constant 1 second (5 seconds for vertical position error signal and temp. reading)

    switch (ISRevent) {
        case eStatusChange:
            if (!statusData.isFloating) { errSignalMagnitudeSmooth = 0; }                   // globe currently not floating ? reset smoothed error value immediately
            break;

        case eFastRateData: {                                                               // data provided at a high rate (every 128 milliseconds)
            // feed idle time, ISR duration, magnet ON cycles and error signal totaled in fastDataRateSamplingPeriods to smoothing filters
            // -> NOTE that at this stage, the smoothed values are NOT AVERAGES BUT SUMS 
            idleLoopMicrosSmooth += ((((float)fastRateDataPtr->sumIdleLoopMicros) - idleLoopMicrosSmooth) * (samplingPeriod * fastDataRateSamplingPeriods / 1.0F));
            ISRdurationSmooth += ((((float)fastRateDataPtr->sumISRdurations) - ISRdurationSmooth) * (samplingPeriod * fastDataRateSamplingPeriods / 1.0F));
            magnetOnCyclesSmooth += ((((float)fastRateDataPtr->sumMagnetOnCycles) - magnetOnCyclesSmooth) * (samplingPeriod * fastDataRateSamplingPeriods / 1.0F));
            errSignalMagnitudeSmooth += ((((float)fastRateDataPtr->sumErrSignalMagnitude) - errSignalMagnitudeSmooth) * (samplingPeriod * fastDataRateSamplingPeriods / 5.0F));

            // feed temp. sensor reading to smoothing filter
            // TMP36 sensor: 10 mV per °C, 750 mV at 25 °C : 1 ADC step * 5000 mV / 1024 steps *  1 °C / 10 mV = 0.488 °C which gives sufficient accuracy for safety purposes
            long temp = ((fastRateDataPtr->sumADCtemp * 50000L - (5000L << 10)) >> 10);     // convert to degrees Celsius x 100 (multiply or divide by 1024 = ADC resolution: shift 10 bits)
            cli();                                                                          // tempSmooth is passed back to ISR for safety check high temperature
            tempSmooth = tempSmooth + ((((temp - tempSmooth) * tempTimeCst1024) / 5L) >> tempTimeCst_BinaryFractionDigits);
            sei();

            partialSumMagnetOnCycles += fastRateDataPtr->sumMagnetOnCycles;
            fastRateDataEventCounter++;                                                     // overflows at 255 idle events = 255 * 128 mS, period = 32768 milliseconds
            if (fastRateDataEventCounter == 0) {
                movingSumMagnetOnCycles = movingSumMagnetOnCycles + partialSumMagnetOnCycles - sumMagnetOnCycles[averagingPeriodsMagnetLoad - 1];   // spans more than 5 minutes
                for (int i = averagingPeriodsMagnetLoad - 2; i >= 0; i--) { sumMagnetOnCycles[i + 1] = sumMagnetOnCycles[i]; }
                sumMagnetOnCycles[0] = partialSumMagnetOnCycles;
                partialSumMagnetOnCycles = 0;
                cli();                                                                      // highLoad is passed back to ISR for safety check high magnet load
                highLoad = (movingSumMagnetOnCycles >= maxOnCycles);
                sei();
            }

            break;
        }

        case eSecond:
            if (secondData.eventSecond == 3) {
                /* not used but could be used for a 3-second cue
                cli();                                                                      // interrupts off: interface with ISR and eeprom write
                eeprom_update_byte((uint8_t*)3, (uint8_t)0);                                // time after reset is longer than 3 seconds
                sei();
                */
            }
            break;
    }
}


// *** execute a user command (except printing) ***

void processCommand() {
    if (userCommand == uNoCmd) { return; }

    bool commandParamError{ false };
    bool doSaveParams{ false };
    int valueCount;

    switch (userCommand) {
        // zero-parameter commands
        case uEdit:                                         // edit item
            paramChangeMode = (parameterEditable & (1L << paramNo));
            if (!paramChangeMode) { break; }                // item is not editable

        case uPrevious:                                     // previous item
        case uNext:                                         // next item
            if (userCommand == uPrevious) { paramNo--; }
            else if (userCommand == uNext) { paramNo++; }
            paramNo = (paramNo + paramCount) % paramCount;

            switch (paramNo) {
                case paramNo_gainAdjust: paramValueOrIndex = gainAdjustSteps; break;
                case paramNo_intTimeConstAdjust: paramValueOrIndex = intTimeCstAdjustSteps; break;
                case paramNo_difTimeConstAdjust: paramValueOrIndex = difTimeCstAdjustSteps; break;
                case paramNo_phaseAdjust: paramValueOrIndex = phaseAdjustSteps; break;
                default: paramValueOrIndex = ParamsSelectedValuesOrIndexes[paramNo]; break;
            }
            break;

        case uCancel:                                       // cancel: no function if not in parameter change mode
            break;

        case uShowAll:                                      // show all values once: yes / no
            break;

        case uLive:                                         // show values continuously: yes / no
            showLiveValues = !showLiveValues;
            break;

        case uTimeStamp:                                    // time stamp
            break;

        case uHelp:                                         // show help
            break;

        case uReset + 20:                                   // select default value
        {
            switch (paramNo) {
                case paramNo_gainAdjust:
                case paramNo_intTimeConstAdjust:
                case paramNo_difTimeConstAdjust:
                    paramValueOrIndex = 16; break;
                case paramNo_phaseAdjust:
                    paramValueOrIndex = 0; break;
            }
        }
        break;

        case uDown:
        case uUp:

        case uDown + 20:
        case uPrevious + 20:                                // previous item value

        case uNext + 20:                                    // next item value
        case uUp + 20:
        {
            bool down = (userCommand == uDown) || (userCommand == (uDown + 20)) || (userCommand == (uPrevious + 20));
            bool immediateSave = ((userCommand == uDown) || (userCommand == uUp));

            if (!(parameterEditable & (1L << paramNo))) { break; }; // if item is not editable, break

           // only for PID & rotation controller settings
            switch (paramNo) {
                case paramNo_gainAdjust:
                case paramNo_intTimeConstAdjust:
                case paramNo_difTimeConstAdjust: {
                    if (down) { if (paramValueOrIndex > 0) { paramValueOrIndex--; } }
                    else { if (paramValueOrIndex < 32) { paramValueOrIndex++; } }   // max. is 32 
                } break;
                case paramNo_phaseAdjust: {
                    if (down) { if (paramValueOrIndex > 0) { paramValueOrIndex--; } else { paramValueOrIndex = 179; } }    // max. is 179 (x2 = 358 degrees)
                    else { if (paramValueOrIndex < 179) { paramValueOrIndex++; } else { paramValueOrIndex = 0; } }
                } break;

                default:
                    if (down) {
                        if (paramValueOrIndex > 0) { paramValueOrIndex--; }
                    }
                    else {
                        if (paramValueOrIndex < paramValueCounts[paramNo] - 1) { paramValueOrIndex++; }
                    }
                    break;
            }

            if (immediateSave) {
                switch (paramNo) {
                    case paramNo_gainAdjust:         gainAdjustSteps = paramValueOrIndex; break;
                    case paramNo_intTimeConstAdjust: intTimeCstAdjustSteps = paramValueOrIndex; break;
                    case paramNo_difTimeConstAdjust: difTimeCstAdjustSteps = paramValueOrIndex; break;
                    case paramNo_phaseAdjust:        phaseAdjustSteps = paramValueOrIndex; break;
                    default:                         ParamsSelectedValuesOrIndexes[paramNo] = paramValueOrIndex; break;
                }
                doSaveParams = true;
            }
        }
        break;

        case uEdit + 20:                                    // exit edit mode (and save)
            switch (paramNo) {
                case paramNo_gainAdjust:         gainAdjustSteps = paramValueOrIndex; break;
                case paramNo_intTimeConstAdjust: intTimeCstAdjustSteps = paramValueOrIndex; break;
                case paramNo_difTimeConstAdjust: difTimeCstAdjustSteps = paramValueOrIndex; break;
                case paramNo_phaseAdjust:        phaseAdjustSteps = paramValueOrIndex; break;
                default:                         ParamsSelectedValuesOrIndexes[paramNo] = paramValueOrIndex; break;
            }
            paramChangeMode = false;
            doSaveParams = true;
            break;

        case uCancel + 20:                                  // cancel edit mode 
            switch (paramNo) {
                case paramNo_gainAdjust: paramValueOrIndex = gainAdjustSteps; break;
                case paramNo_intTimeConstAdjust: paramValueOrIndex = intTimeCstAdjustSteps; break;
                case paramNo_difTimeConstAdjust: paramValueOrIndex = difTimeCstAdjustSteps; break;
                case paramNo_phaseAdjust: paramValueOrIndex = phaseAdjustSteps; break;
                default: paramValueOrIndex = ParamsSelectedValuesOrIndexes[paramNo]; break;
            }
            paramChangeMode = false;
            break;

        // one-parameter commands
        case uMeasure:
            commandParamError = !((commandParam1 == 0) || (commandParam1 == 1));
            if (!commandParamError) {
                ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                    applyStep = (commandParam1 == 1);
                    if (printPIDtimeCounter > printPIDperiod) { printPIDtimeCounter = 0; }  // start printing: only when currently stopped
                }
            }
            break;

        // two-parameter commands
        case uLedstripSettings:
            commandParamError = !((commandParam1 == 'C') && (commandParam2 >= cLedstripOff) && (commandParam2 <= cRedGreenBlue));
            if (!commandParamError) { setColorCycle((uint8_t)commandParam2, LScolorTiming); }
            else {
                commandParamError = !((commandParam1 == 'T') && (commandParam2 >= cLedstripVeryFast + 1) && (commandParam2 <= cLedStripVerySlow + 1));
                if (!commandParamError) { setColorCycle((uint8_t)LScolorCycle, commandParam2 - 1); }
            }
            break;

            // command error
        case uUnknownCmd:                                               // signal unknown command
            break;
    } // switch

    if (doSaveParams) { saveAndUseParam(); }

    if (commandParamError) { userCommand = uUnknownCmd; }               // command parameter error
}


// *** check switch settings ***

void checkSwitches(bool forceSwitchCheck /* = false */) {               // if SW3 to SW0 to be interpreted as switches only (instead of buttons)

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {                                 // interrupts off to read switch setting: interface with ISR and eeprom write
        currentSwitchStates = switchStates;
    }

    if (forceSwitchCheck || (currentSwitchStates != prevSwitchStates)) {
        prevSwitchStates = currentSwitchStates;

        // here comes code for signal SW4, if used (is never interpreted as button)
        // ...

        if (switchesSetLedstrip) {                                      // set led strip cycle and timing                                
            // signal SW3 to SW0: set led strip cycle and timing
            // bit 3210: 
            // 00cc -> color cycle 0 to 3 (OFF or cst color), do not change led strip timing
            // 01tt -> color cycle 4 (white blue), timing 1 to 4
            // 10tt -> color cycle 5 (red green blue), timing 1 to 4
            // 11xx -> do not change led strip cycle and timing

            uint8_t sw = (currentSwitchStates >> 2) & (uint8_t)0x0F;
            if (sw <= 3) {                                              // led strip OFF or cst brightness: set cycle only, keep current timing
                setColorCycle(sw, LScolorTiming);                       // see enum: cLedstripOff = 0, cCstBrightWhite = 1, cCstBrightMagenta = 2, cCstBrightBlue = 3
            }
            else if (sw <= B00001011) {                                 // led strip sequence white blue or red green blue : set cycle and timing
                uint8_t colorCycle = (sw >> 2) + cWhiteBlue - 1;
                uint8_t colorTiming = (sw & B00000011);
                setColorCycle(colorCycle, colorTiming);
            }
            forceWriteLedstripSpecs = true;
        }

        else if (switchesSetRotationTime || switchesSetHallmVoltRef) {                      // set rotation time or vertical position setpoint
            // signal SW3 to SW0: set rotation time according to values stored
            uint8_t sw = (currentSwitchStates >> 2) & (uint8_t)0x0F;
            paramNo = switchesSetRotationTime ? paramNo_rotTimes : paramNo_hallmVoltRefs;   // parameter = rotation time or hall mV ref ?
            int cnt = paramValueCounts[paramNo];                                            // No of defined rotation times 
            paramValueOrIndex = (sw >= cnt) ? 0 : sw;                                       // if not in valid range, take first in list
            saveAndUseParam();
        }
    }
}


// *** write status and other info to LCD and Serial ***

void writeStatus() {
    if ((ISRevent == eNoEvent) && (userCommand == uNoCmd)) { return; }

    if ((userCommand == uShowAll) || (userCommand == uTimeStamp)) {                         // time stamp (actual time) needed ?
        long sec{ 0 };
        long mS = myTime.millis(&sec);
        formatTime(s30, sec, mS);
        strcat(strcpy_P(s150, str_timeStamp), s30);
        Serial.println();
        Serial.println(s150);                                                               // time stamp line
    }

    bool refreshStatus = ((ISRevent == eStatusChange) || (userCommand == uShowAll));        // user command Show all: print status as well  
    if (refreshStatus) {
        lcd.setCursor(0, 0);
        lcd.print(strcpy_P(s30, str_empty16));

        if (statusData.errorCondition == errNoError) {
            switch (statusData.rotationStatus) {
                case rotNoPosSync: strcpy_P(s30, statusData.isFloating ? ((targetGlobeRotationTime == 0) ? str_rotationOff : str_noPosSync) : str_notFloating); break;
                case rotFreeRunning: strcpy_P(s30, str_freeRunning); break;
                case rotMeasuring: strcpy_P(s30, str_measuring); break;
                case rotUnlocked: strcpy_P(s30, str_notLocked); break;
                case rotLocked: strcpy_P(s30, str_locked); break;
            }
        }

        else {
            switch (statusData.errorCondition) {
                case errDroppedGlobe: strcpy_P(s30, str_ErrDroppedGlobe); break;
                case errStickyGlobe: strcpy_P(s30, str_ErrStickyGlobe); break;
                case errMagnetLoad: strcpy_P(s30, str_ErrOverload); break;
                case errTemp: strcpy_P(s30, str_ErrTemp); break;
            }
        }

        lcd.setCursor(0, 0);
        lcd.print(s30);

        strcat(strcat(strcpy(s150, "++ "), s30), ((userCommand == uShowAll) ? " ++" : " ("));
        if (ISRevent == eStatusChange) {
            Serial.println();
            formatTime(s30, statusData.eventSecond, statusData.eventMilliSecond);           // time of this status change (which is a little earlier than myTime.millis(), giving the current time)
            strcat(strcat(s150, s30), ") ++");
        }
        Serial.println(s150);
    }

    if (userCommand == uShowAll) {                                                          // Print all parameters to Serial
        for (long paramNo = 0; paramNo < paramCount; paramNo++) {
            int paramValueOrIndex = ParamsSelectedValuesOrIndexes[paramNo];
            strcpy_P(s150, (char*)pgm_read_word(&(paramLabels[paramNo])));                  // parameter label
            fetchParameterValue(s30, paramNo, paramValueOrIndex);
            strcat(s150, s30);
            Serial.println(s150);
        }
        Serial.println();
    }

    if ((userCommand == uShowAll) || (userCommand == uLedstripSettings) || forceWriteLedstripSpecs) {    // change led strip cycle
        if ((userCommand == uLedstripSettings) || (forceWriteLedstripSpecs && (ISRevent != eStatusChange))) { Serial.println(); }
        forceWriteLedstripSpecs = false;
        sprintf(s30, "%u", LScolorCycle);
        Serial.print(strcat(strcpy_P(s150, str_colorCycle), (LScolorCycle == cLedstripOff) ? "Off" : s30));
        if (LScolorCycle >= cWhiteBlue) {
            sprintf(s30, "%u", LScolorTiming + 1);
            Serial.println(strcat(strcpy(s150, ", timing "), s30));
        }
        else { Serial.println(); }
    }

    else if (userCommand == uLive) {                                                        // show or stop printing live values
        Serial.println();
        Serial.println(strcpy_P(s150, showLiveValues ? str_showLive : str_stopLive));
    }

    else if (userCommand == uTimeStamp) {                                                   // write time stamp (actual time)
        // nothing else to do
    }

    else if (userCommand == uHelp) {                                                        // print help string
        Serial.println();
        Serial.println(strcpy_P(s150, str_help1));
        Serial.println(strcpy_P(s150, str_help2));
        Serial.println();
    }

    else if (userCommand == uMeasure) {
        Serial.println();
        Serial.println(strcpy_P(s150, str_stepResponse));
    }

    else if (userCommand == uUnknownCmd) {                                                  // signal unrecognized command
        /*
        Serial.println();
        Serial.println(strcpy_P(s150, str_cmdError));
        */
    }

    // step response test (note that if printing a lot of data every millisecond, and combining with other commands: risk of missing events)
    else if (ISRevent == eStepResponseData) {
        if (stepResponseDataPtr->count <= printPIDperiod) {
            sprintf_P(s150, str_fmt3unsignedInteger, stepResponseDataPtr->count, stepResponseDataPtr->hallReading_ADCsteps, (stepResponseDataPtr->count == 1) ? 0 : stepResponseDataPtr->TTTcontrOut);
            if (stepResponseDataPtr->count == 1) {
                sprintf(s30, ";%lu", firstFullAccIntTerm);
                strcat(s150, s30);
            }
            Serial.println(s150);
        }
        else {
            Serial.println();
            Serial.println(strcpy_P(s150, str_stepResponseEnd));
        }
    }
}


// *** write a parameter and its value to LCD and Serial ***

void writeParamLabelAndValue() {
    static bool blinkingTextNowOn{ true }, blinkEnabled{ false };                           // blinking values on LCD

    if (ISRevent == eBlink) { blinkingTextNowOn = false; }
    else if (ISRevent == eSecond) { blinkingTextNowOn = true; }
    blinkEnabled = paramChangeMode;                                                         // blink if user is changing values

    // parameter value type ?
    bool isSetValue = (parameterEditable & (1L << paramNo));                                // setting that can be changed by user  
    bool isRotationValue = ((paramNo == 1) || (paramNo == 2) || (paramNo == 10));           // value to print is provided by last Greenwich event
    bool isLiveValue = !(isSetValue || isRotationValue);                                    // all other values

    // refresh LCD ?
    bool LCDeraseValue = ((ISRevent == eBlink) && (!blinkingTextNowOn) && blinkEnabled);

    bool LCDwriteValue = (ISRevent == eStatusChange)
        || ((ISRevent == eGreenwich) && isRotationValue)
        || ((ISRevent == eSecond) && (isLiveValue || blinkEnabled))                         // if blink is enabled, value needs to be rewritten now as well
        || ((userCommand >= 0) && (blinkingTextNowOn || !blinkEnabled));                    // if blink is enabled, do not write value while screen is erased

    // refresh Serial ?
    bool SerialWriteValue = ((ISRevent == eStatusChange) && !(isRotationValue && statusData.isGreenwich))   // if linked Greenwich event, do not write rotation value now
        || (showLiveValues && ((ISRevent == eGreenwich) && isRotationValue))
        || (showLiveValues && ((ISRevent == eSecond) && isLiveValue))
        || ((userCommand >= 0) && (userCommand != uMeasure));

    if (ISRevent == eStepResponseData) { SerialWriteValue = SerialWriteValue || (stepResponseDataPtr->count > printPIDperiod); }  // pointer is only defined if step response event

    strcpy_P(s150, (char*)pgm_read_word(&(paramLabels[paramNo])));                           // parameter label

    fetchParameterValue(s30, paramNo, paramValueOrIndex);                                    // parameter value
    strcat(s150, LCDeraseValue ? "       " : s30);                                           // blink: spaces instead of value

    if (LCDeraseValue) {                                                                     // blink
        lcd.setCursor(0, 1);
        lcd.print(s150);
    }
    if (LCDwriteValue) {
        lcd.setCursor(0, 1);
        lcd.print(s150);
    }

    if (SerialWriteValue) {
        Serial.print(s150);
        strcpy_P(s150, (paramNo > paramNo_gainAdjust) ? str_editValueWithDefault : str_editValue);
        Serial.println(paramChangeMode ? s150 : "");

        if (eventSnapshot.eventsMissed > 0) {
            sprintf(s150, "%ld ", eventSnapshot.eventsMissed);
            strcat_P(s150, str_eventsMissed);
            Serial.println(s150);
        }

    #if test_showEventStats
        sprintf_P(s150, str_eventMaxStats, eventSnapshot.largestEventsPending, eventSnapshot.largestEventBufferBytesUsed);
        Serial.println(s150);
    #endif
    }
}


// *** write to led strip ***

void writeLedStrip() {
    const uint8_t minBrightnessGamma = (((((uint32_t)LSminBrightnessLevel) + 1UL) * (((uint32_t)LSminBrightnessLevel) + 1UL)) - 1UL) >> 8;
    const uint8_t maxBrightnessGamma = (((((uint32_t)LSmaxBrightnessLevel) + 1UL) * (((uint32_t)LSmaxBrightnessLevel) + 1UL)) - 1UL) >> 8;

    // include the line below to alternate between blue with another color if LScolorCycle == cWhiteBlue
    // static uint8_t LSColorSequence{ 1 }; 

    if (ISRevent != eLedstripData) { return; }                                                  // no change in brightness values 

    /*
    // only 4 leds (above and underneath) used
    uint8_t ledstripMasks[3]{ B10100101, B10100101, B10100101 };                                // RGB led strip mask (8 leds) for red, green, blue colors, in this order
    */
    uint8_t ledstripMasks[3]{ B11111111, B11111111, B11111111 };                                // RGB led strip mask (8 leds) for red, green, blue colors, in this order 

    if (ledstripDataPtr->LSupdate) {                                                            // brightness updated ?
        for (uint8_t i = 0; i < LSbrightnessItemCount; i++) {
            // assign calculated brightness values to Blue, Green and Red, respectively (order defined by led strip hardware)
            uint32_t temp = ((uint32_t)ledstripDataPtr->LScolor[i]) + 1UL;
            uint32_t brGamma = ((temp * temp) - 1UL) >> 8;                                      // gamma correction: use quadratic function (approximation for ^2.2)
            colorGammaCorrected[i + 1] = (uint8_t)brGamma;                                      // byte 0 = led brightness (fixed), bytes 123 = blue-green-red, in this order (defined by led strip hardware)                                        
        }

        if ((LScolorCycle >= cCstBrightWhite) && (LScolorCycle <= cCstBrightBlue)) {            // constant color (white, magenta, blue) 
            colorGammaCorrected[1] = maxBrightnessGamma;                                                    // blue: maximum brightness
            colorGammaCorrected[2] = (LScolorCycle == cCstBrightWhite) ? maxBrightnessGamma : minBrightnessGamma;       // green: minimum brightness except if led color is white
            colorGammaCorrected[3] = (LScolorCycle != cCstBrightBlue) ? maxBrightnessGamma : minBrightnessGamma;        // red: minimum brightness except if led color is white or magenta
        }

        else if (LScolorCycle == cWhiteBlue) {                                                  // white > blue
            colorGammaCorrected[2] = colorGammaCorrected[1];                                    // set green equal to blue
            colorGammaCorrected[3] = colorGammaCorrected[1];                                    // set red equal to blue
            colorGammaCorrected[1] = maxBrightnessGamma;                                        // set blue to max brightness
            /* include this code to alternate blue with (in this sample code) magenta: white > blue > white > magenta
            if (LSColorSequence == 2) { colorGammaCorrected[3] = maxBrightnessGamma; }          // every two cycles, set red to max brightness
            if (ledstripDataPtr->LSmaxReached) {
                if (++LSColorSequence == 3) { LSColorSequence = 1; }
            }
            */
        }

        LSout((uint8_t*)colorGammaCorrected, ledstripMasks);
    }
}


// *** send led strip data to hardware ***

void LSout(uint8_t* led, uint8_t* ledstripMasks) {                                              // output data to led strip 
    uint8_t startFrame[4]{ 0, 0, 0, 0 }, colorOffAndEndFrame[4]{ 0xFF, 0, 0, 0 };               // brightness, blue, green, red

    // NOTE: only Port C 'IO disable' and, if applicable, 'led strip data' bits, should be written to (altered) by led strip routines
    uint8_t holdPortC = PORTC;                                                                  // read current PORTC bits only once (speed)
    PORTB = ((PORTB & ~portB_IOchannelSelectBitMask) | portB_ledstripSelect);                   // PORT B: select led strip

    LSoneLedOut(holdPortC, startFrame);                                                         // Start-frame marker
    for (uint8_t ledNo = 0; ledNo <= 7; ledNo++, ledstripMasks[0] >>= 1, ledstripMasks[1] >>= 1, ledstripMasks[2] >>= 1) {      // For each led
        LSoneLedOut(holdPortC, led, (ledstripMasks[2] & B1) | ((ledstripMasks[1] & B1) << 1) | ((ledstripMasks[0] & B1) << 2));
    }
    LSoneLedOut(holdPortC, colorOffAndEndFrame);
}


// *** send led strip data for one led to hardware ***

void LSoneLedOut(uint8_t holdPortC, uint8_t* ledStrip4Bytes, uint8_t ledMask /* = B111 */) {    // output data for 1 led strip led
    uint8_t b8;      // preserve original value
    ledMask = (ledMask << 1) | B1;                                                              // add '1' because brightness byte is always sent to led as received

    for (uint8_t n = 0; n <= 3; n++, ledStrip4Bytes++, ledMask >>= 1) {                         // 4 bytes per pointer (brightness byte, blue value, green value, red value)
        b8 = (ledMask & B1) ? *ledStrip4Bytes : (uint8_t)0x00;                                  // 1 byte
        for (int8_t i = 7; i >= 0; i--, b8 <<= 1) {                                             // shift out 8 bits

        #if (boardVersion == 100)
            if (b8 & (uint8_t)0x80) { holdPortC |= portC_ledstripDataBit; }
            else { holdPortC &= ~portC_ledstripDataBit; }
        #else
            // NOTE: original PORT D data does NOT need to be held and restored (but interrupts should hold and restore PORT D data)  
            PORTD = (b8 & (uint8_t)0x80) ? ledstripDataBits : (uint8_t)0x00;                    // currently, two ledstrips receive same data                       
        #endif
            cli();                                                                              // do not interrupt clocking led strip
            // speed: no port reads, only two port writes
            PORTC = holdPortC & ~portC_IOdisableBit;                                            // setup led strip data at clock falling edge                                                        
            PORTC = holdPortC | portC_IOdisableBit;                                             // clock in led strip data at clock rising edge
            sei();
        }
    }
}


// *** measure idle time (time spent in this loop) - optimized for speed ( < 7 microseconds per single do ... while loop) ***

void idleLoop() {
    bool stopCountingTime{ false }, isFirstLoop{ true };
    unsigned int prevNanoSecond500{ 0 };

    do {                                                                        // relies on (delta T between two loops << 1000 micro seconds), no need to check for timer 1 overflow (TOV1) as in class MyTime
        uint8_t oldSREG = SREG;
        cli();                                                                  // do not interrupt time counting loop
        bool workPending = ((myEvents.isEventsWaiting()) || (keysAvailable > 0) // need to exit idle loop because ... (NOTE: use LIVE eventStats here in order to detect newly occuring events)
            || (Serial.available() > 0));                                       // (1) there is something to do (possibly as a result from previous interrupts): 
        stopCountingTime = (workPending || ISRoccurred);                        // (2) interrupt occurred (flag set in ISR) and the ISR duration may not be added to the idle time 
        // this only works for 'my' interrupts, other interrupts (e.g. timer 0) are missed as they don't set ISRoccurred, resulting in a small error    
        ISRoccurred = false;                                                    // reset flag
        if (!stopCountingTime) {
            unsigned int volatile nanoSecond500{}, nextNanoSecond500{};
            nanoSecond500 = TCNT1;                                              // one read takes 8 clock cycles (2 LDS and 2 STS instructions -> 4 * 2 = 8 clock cycles = 500 nS with 16 MHz clock) 
            nextNanoSecond500 = TCNT1;                                          // take 2 counter readings (500 nS steps) to determine slope (difference between 2 readings is exactly -1 or +1)
            if (nanoSecond500 > nextNanoSecond500) { nanoSecond500 = 2 * timer1Top - nextNanoSecond500; }   // counting down (500 nS per cycle) 
            if (!isFirstLoop) { idleLoopNanos500 = (idleLoopNanos500 + ((nanoSecond500 < prevNanoSecond500) ? 2 * timer1Top : 0) + nanoSecond500) - prevNanoSecond500; }
            prevNanoSecond500 = nanoSecond500;
            isFirstLoop = false;
        }
        SREG = oldSREG;
    } while (!stopCountingTime);
}


// *** utilities ***

// *** format time, given as a number of seconds and milliseconds within a second as a string and optionally return total days, hours, minutes and seconds ***

void formatTime(char* s, long totalSeconds, long totalMillis, long* days /* = nullptr */, long* hours /* = nullptr */, long* minutes /* = nullptr */, long* seconds /* = nullptr */) {

    long sec, min, hr, d;

    sec = totalSeconds;
    min = totalSeconds / 60L;
    hr = min / 60L;
    d = hr / 24L;
    sec = totalSeconds - min * 60;
    min = min - hr * 60;
    hr = hr - d * 24;

    if (days != nullptr) { *days = d; }
    if (hours != nullptr) { *hours = hr; }
    if (minutes != nullptr) { *minutes = min; }
    if (seconds != nullptr) { *seconds = sec; }

    sprintf_P(s, str_fmtTime, d, hr, min, sec, totalMillis);
}


// *** read a character from the hardware buttons buffer or the Serial interface ***

void readKey(char* keyAscii) {                                  // from Serial interface and Globe keys
    constexpr char keypressChars[6] = "-+EC~";                  // note that max 5 keys are available; ~ means no code assigned

    *keyAscii = 0;
    int8_t key{ 0 };                                            // can hold negative key codes

    if (keysAvailable > 0) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            key = keyBuffer[0];
            for (size_t i = 0; i < keyBufferLength - 1; i++) { keyBuffer[i] = keyBuffer[i + 1]; }
            keysAvailable--;
        }

        if (key > 0) {                                          // key press: convert to ASCII for common treatment with characters read from Serial
            *keyAscii = keypressChars[key - 1];
            if (*keyAscii == '~') { *keyAscii = 0; }
        }
        else {}                                                 // key release (normal, long or extra long key press): currently not used (throw away) 
    }

    if ((*keyAscii == 0) && (Serial.available() > 0)) {         // no character from board ? read one character from serial buffer, if available
        *keyAscii = Serial.read();
    }
}


// *** fetch a parameter value ***

void fetchParameterValue(char* s, long paramNo, int paramValueOrIndex) {
    long paramValue{ 0 };                                       // !!! do not define variables within a case clause unless you put the case clause in curly brackets
    long days, hours, min, sec;

    switch (paramNo) {
        case 0: {                                               // set rotation time
            paramValue = *(paramValueSets[paramNo] + paramValueOrIndex);
            strcpy(s, "    off");
            if (paramValue != 0) {
                dtostrf(((float)paramValue) / 1000., 6, 2, s);
                strcat(s, "s");
            }
            break;
        }

        case 1: {                                               // actual rotation time
            strcpy(s, " --.--s");
            if (statusData.rotationStatus >= rotUnlocked) {     // from latest status change event before write
                dtostrf(((float)greenwichData.globeRotationTime) / 1000., 6, 2, s);
                strcat(s, "s");
            }
            break;
        }

        case 2: {                                               // sync error
            strcpy(s, "  -.--s");
            if (statusData.rotationStatus == rotLocked) {       // from latest status change event before write
                dtostrf(((float)greenwichData.rotationOutOfSyncTime) / 1000., 6, 2, s);
                strcat(s, "s");
            }
            break;
        }

        case 3: {                                                // time globe rotation is locked to rotating magnetic field
            formatTime(s, (statusData.rotationStatus == rotLocked ? secondData.lockedSecond : 0), 0, &days, &hours, &min, &sec);
            if (days > 0) { sprintf_P(s, str_fmtDayHour, days, hours); }
            else { sprintf_P(s, str_fmtHourMinute, hours, min); }
            break;
        }

        case 4: {                                                // time globe is floating
            formatTime(s, (statusData.isFloating ? secondData.liftingSecond : 0), 0, &days, &hours, &min, &sec);
            if (days > 0) { sprintf_P(s, str_fmtDayHour, days, hours); }
            else { sprintf_P(s, str_fmtHourMinute, hours, min); }
            break;
        }

        case 5: {                                                 // temperature
            dtostrf(((float)tempSmooth) / 100., 5, 1, s);
            strcat(s, degreesSymbol);
            break;
        }

        case 6: {                                                // avg duty cycle
            // divide by sampling periods to get average ON cycles, divide by magnet cycles available to get ratio, x 100%
            dtostrf((magnetOnCyclesSmooth / fastDataRateSamplingPeriods) * 100. / ((float)timer1Top), 5, 1, s); // in tenths of percent
            strcat(s, "%");
            break;
        }

        case 7: {                                                // globe lifting: reference for hall detector in milliVolt as read by Arduino (sensor value x analog gain)
            paramValue = *(paramValueSets[paramNo] + paramValueOrIndex);
            sprintf(s, "%5ldmV", paramValue);
            break;
        }

        case 8: {                                                // error signal variance in milliVolt as read by Arduino (sensor value x analog gain)
            strcpy(s, " ---mV");
            if (statusData.isFloating) {                         // from latest fast rate update event before write
                // divide by sampling periods to get avg error ADC steps, divide by 1024 steps, multiply by 5000 milliVolt
                dtostrf((errSignalMagnitudeSmooth / fastDataRateSamplingPeriods) * 5000. / (float)ADCsteps, 4, 0, s);
                strcat(s, "mV");
            }
            break;
        }

        case 9: {                                                // integration term
            sprintf(s, "%6ld", secondData.realTTTintTerm);
            break;
        }

        case 10: {                                               // average phase measured while locked
            strcpy(s, "  ---deg");
            if (statusData.rotationStatus == rotLocked) {
                int angle = (greenwichData.greenwichLag * 360) / targetGlobeRotationTime;
                sprintf(s, "%+5ddeg", angle);                   // lag (>180 degrees: lead)
            }
            break;
        }


        case 11: {                                               // ADC conversion complete ISR routine duration measured
            dtostrf(ISRdurationSmooth / fastDataRateSamplingPeriods, 5, 0, s); // divide by sampling periods to get average
            strcat(s, microSecSymbol);
            break;
        }

        case 12: {                                               // processor load
            // idleLoopMicrosSmooth: smoothed total idle micro seconds in 128 sample periods
            // 1000 * fastDataRateSamplingPeriods: total microseconds available in 128 sample periods
            dtostrf(((1E3F * (float)fastDataRateSamplingPeriods - idleLoopMicrosSmooth) * 100.F / (1E3F * (float)fastDataRateSamplingPeriods)), 6, 1, s); // as a percentage (whole number)
            strcat(s, "%");
            break;
        }

        case 13: {
            dtostrf(presetGain * analogGain / 10. + gainStepSize * (paramValueOrIndex - 16), 11, 3, s);         // 16: center point (preset)
            break;
        }
        case 14: {
            dtostrf(presetIntTimeCst + intTimeCstStepSize * (paramValueOrIndex - 16), 7, 1, s);
            strcat(s, "s");
            break;
        }
        case 15: {
            dtostrf((presetDifTimeCst + difTimeStepSize * (paramValueOrIndex - 16)) * 1000., 6, 1, s);
            strcat(s, milliSecSymbol);
            break;
        }
        case 16: {

            sprintf(s, "%+4ddeg", (paramValueOrIndex < 90) ? (paramValueOrIndex << 1) : (paramValueOrIndex << 1) - 360);
            break;
        }
    }
}


// *** save a parameter value changed by user ***

void saveAndUseParam()
{
    ParamsSelectedValuesOrIndexes[paramNo] = paramValueOrIndex;
    // only items that can be changed need to have an entry here 
    int byteNumber{ 4 };

    switch (paramNo) {
        case paramNo_rotTimes: {
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {                                     // interrupts off: interface with ISR and eeprom write
                setRotationTime(paramValueOrIndex);
                forceStatusEvent = true;                                            // 'not floating', 'no position sync' and 'rotation OFF' share same status, so force status re-write
                eeprom_update_byte((uint8_t*)0, (uint8_t)paramValueOrIndex);        // eeprom write can take longer than 1 mS (with no interrupts), but lifting magnet will hold
            }
            break;
        }

        case paramNo_hallmVoltRefs: {
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {                                     // interrupts off: interface with ISR and eeprom write
                long hallmVoltRef = *(paramValueSets[paramNo] + paramValueOrIndex); // globe vertical position ref in mVolt read by Arduino ADC
                targetHallRef_ADCsteps = (ADCsteps * hallmVoltRef) / 5000L;
                forceStatusEvent = true;                                            // will force rewriting serial and LCD
                eeprom_update_byte((uint8_t*)1, (uint8_t)paramValueOrIndex);        // eeprom write can take longer than 1 mS (with no interrupts), but lifting magnet will hold
            }
            break;
        }

        case paramNo_phaseAdjust: byteNumber++;
        case paramNo_difTimeConstAdjust: byteNumber++;
        case paramNo_intTimeConstAdjust: byteNumber++;
        case paramNo_gainAdjust:
        {
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {                                     // interrupts off: interface with ISR and eeprom write
                setPIDcontroller();
                forceStatusEvent = true;                                            // will force rewriting serial and LCD
                eeprom_update_byte((uint8_t*)byteNumber, (uint8_t)paramValueOrIndex);    // eeprom write can take longer than 1 mS (with no interrupts), but lifting magnet will hold
            }
            break;
        }
    }
}


// ***  ***

void setPIDcontroller()
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {                                             // interrupts off: interface with ISR and eeprom write
        // multiplication factors must be chosen in relation to order of magnitude of the presets
        gain = presetGain + gainStepSize * (gainAdjustSteps - 16);                  // 16: center point (preset)
        intTimeCst = presetIntTimeCst + intTimeCstStepSize * (intTimeCstAdjustSteps - 16);
        difTimeCst = presetDifTimeCst + difTimeStepSize * (difTimeCstAdjustSteps - 16);

        TTTintTimeCst = (intTimeCst * (1 + difTimeCst / intTimeCst));               // TTT integrator time constant
        TTTdifTimeCst = (difTimeCst / (1 + difTimeCst / intTimeCst));               // TTT differentiator time constant

        TTTgain = (long)(gain * (1. + difTimeCst / intTimeCst) * (1L << gain_BinaryFractionDigits));     // TTT gain
        TTTintFactor = (long)(samplingPeriod / TTTintTimeCst * (1L << TTTintFactor_BinaryFractionDigits));
        TTTdifFactor = (long)(TTTdifTimeCst / samplingPeriod * (1L << TTTdifFactor_BinaryFractionDigits));

        maxTTTallTerms = LONG_MAX / 2 / TTTgain;                                     // for check to prevent overflow after multiplication (factor 1/2: keep 1 extra bit for safety)
    }
}


// *** set a specific rotation time ***

void setRotationTime(int paramValueOrIndex, bool init /* = false */)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {                                             // interrupts off: interface with ISR and eeprom write
        targetGlobeRotationTime = *(paramValueSets[paramNo_rotTimes] + paramValueOrIndex);
            // adapt magnetic field rotation immediately
        targetStepTime = targetGlobeRotationTime / stepCount;

        const int speedIndex = (targetGlobeRotationTime >= 3000) ? 1 : 0;

        // slow down and speed up: OUTSIDE a range of rotation times -> max, min globe rotation times, respectively
        slowDown_maxGlobeRotationTime = targetGlobeRotationTime * speedAdjustCenterRange_low[speedIndex];         // if current globe rotation time is smaller than this value (speed is higher), slow down
        speedUp_minGlobeRotationTime = targetGlobeRotationTime * speedAdjustCenterRange_high[speedIndex];

        // autolock: INSIDE a range of rotation times -> min, max globe rotation times, respectively
        autoLock_minGlobeRotationTime = targetGlobeRotationTime * autoLockRange_low[speedIndex];        // if globe rotation time is within this range, rotation will try to lock on to rotating magnetic field
        autoLock_maxGlobeRotationTime = targetGlobeRotationTime * autoLockRange_high[speedIndex];

        rotationStatus = rotNoPosSync;                                              // change of rotation speed: no position sync yet; waiting to start measuring

        stepTimeNewRotation = targetStepTime;
        rotationTimerSamplePeriod = stepTimeNewRotation;
    }
}


// *** set a specific color cycle ***

void setColorCycle(uint8_t newColorCycle, uint8_t newColorTiming, bool initColorCycle /* = false */)
{
    // led strip timing is only relevant for non-cst led strip cycles
    // very fast: every 2.5 minute resp. 3 minutes; slow: every 10 resp. 15 minutes (depends on led strip cycle); 
    // slow and very slow cycles: choose cycle times such that cycle will be moved 1/2 cycle (two-color cycles) or 1/6 cycle (three-color cycles) every 24 hours 
    // slow: number of complete cycles: 48 two-color cycles in 23 hours 45 minutes, or 24 three-color cycles in 23 hours 50 minutes, respectively, giving a time shift of either 15 or 10 minutes per day
    // very slow: number of complete cycles: 12 two-color cycles in 23 hours, or 6 three-color cycles in 23 hours 20 minutes, respectively, giving a time shift of either 30 or 40 minutes per day
    constexpr long ledstripTimings[2][4] = { { 150 * 1000L, 600 * 1000L, 1781 * 1000L + 250L, 6900 * 1000L }, { 180 * 1000L, 900 * 1000L, 3575 * 1000L, 14000 * 1000L } }; // in seconds
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {                                                                   // interrupts off: interface with ISR and eeprom write
        if (initColorCycle || (newColorCycle != LScolorCycle) || (newColorTiming != LScolorTiming)) {
            LScolorCycle = newColorCycle;                                                                   // color cycle
            LScolorTiming = newColorTiming;                                                                 // color cycle timing
            LStransitionStops = (LScolorCycle <= cWhiteBlue) ? 2L : 6L;                                     // no of 'frozen brightness' stops within a brightness cycle: MUST be > 0 => set LSbrightnessFreezeTime = 0 if no transition stops desired
            int8_t cycleType = (LScolorCycle <= cWhiteBlue) ? 0 : 1;
            LSbrightnessTransitionTime = ledstripTimings[cycleType][newColorTiming];                        // total brightness transition time (in a complete color cycle), in milliseconds - excludes 'frozen brightness' times                                           
            LSlongTimeUnit = LSbrightnessTransitionTime >= 3600 * 1000L;                                    // prevent overflow LSscaledDelay variable for long cycle times: time unit is now 128 milliseconds instead of 1 ms
            if (LSlongTimeUnit) { LSbrightnessTransitionTime >>= 7; }                                       // will introduce small cumulative timing error if led strip cycle time not a multiple of 128 ms

            LSbrightnessFreezeTime = ((LScolorCycle <= cWhiteBlue) ? 0L : 0L) * 1000L;                      // total 'frozen brightness' time (summed up constant brightness time between all transitions), in milliseconds (minimum = 0)
            LSbrightnessCycleTime = LSbrightnessFreezeTime + LSbrightnessTransitionTime;                    // COMPLETE color cycle in milliseconds

            LSminBrightnessTime = (LScolorCycle <= cWhiteBlue) ? 0 : LSbrightnessTransitionTime / 3;        // if 3 colors: 1/3 of cycle time (only two primary colors at the same time - no white)
            LSmaxBrightnessTime = 0.6 * LSbrightnessTransitionTime / ((LScolorCycle <= cWhiteBlue) ? 2 : 3);// min 0: primary stronger than CMY), max 1/(no of colors) of cycle time (CMY stronger than primary) 
            LSbrightnessDelay = LSbrightnessTransitionTime / ((LScolorCycle <= cWhiteBlue) ? 2 : 3);        // delay between brightness cycles (here: between to or three brightness values)

            // number of brightness steps, including steps of min / max brightness (but excluding 'frozen brightness' time)
            LSbrightnessUpDownSteps = (LSmaxBrightnessLevel - LSminBrightnessLevel) * 2;
            LSbrightnessMinLvlSteps = (LSbrightnessUpDownSteps * LSminBrightnessTime) / (LSbrightnessTransitionTime - LSminBrightnessTime - LSmaxBrightnessTime);
            LSbrightnessMaxLvlSteps = (LSbrightnessUpDownSteps * LSmaxBrightnessTime) / (LSbrightnessTransitionTime - LSminBrightnessTime - LSmaxBrightnessTime);
            LSbrightnessTransitionSteps = (LSbrightnessUpDownSteps + LSbrightnessMinLvlSteps + LSbrightnessMaxLvlSteps);

            LSscaledDelay = LSbrightnessDelay * LSbrightnessTransitionSteps;                                // delay between brightness cycles, scaled by total no of brightness steps

            LSbrightnessFreezeTimer = -LSbrightnessCycleTime - 0 * LSbrightnessTransitionTime;              // optional: skip first 'frozen brightness' step if a brightness is still missing (if starting from 'all brightness values OFF')

            for (uint8_t i = 0; i < LSbrightnessItemCount; i++) {
                LScolor[i] = ((i == 0) && (LScolorCycle > cLedstripOff)) ? LSmaxBrightnessLevel : LSminBrightnessLevel;    // initial brightness levels (not yet assigned to specific colors or leds)
                LSminBrightnessStepNo[i] = 0;
                LSmaxBrightnessStepNo[i] = (i == 0) ? LSbrightnessMaxLvlSteps >> 1 : 0;
                // initial values per brightness step timer: step size and delay between brightness cycle, scaled by total no of brightness steps (step size x no of brightness steps = total transition time)
                LSbrightnessStepTimer[i] = -LSbrightnessTransitionTime - ((i == 2) ? LSscaledDelay : 0);
            }

            LSupdate = B111;                                                                                // update flags per brightness 
            LSup = B111;                                                                                    // initial dimming direction up for all brightness items (true if initially counting up OR in a max. brightness period)
            LSminReached = B000;
            LSmaxReached = B000;

            eeprom_update_byte((uint8_t*)2, (uint8_t)(LScolorCycle | (LScolorTiming << 4)));                // eeprom write can take longer than 1 mS (with no interrupts), but lifting magnet will hold
        }
    }
}


// *** timer 1 overflow interrupt service routine (ISR) ***

// reset hardware watchdog if requested by main loop
// initiate ADC conversion hall sensor value (vertical position measurement) -> will be read in 'ADC conversion complete' ISR
// read and debounce switches & buttons, read Greenwich position sensor
// count time

SIGNAL(TIMER1_OVF_vect) {

    uint8_t holdPortBduringInt = PORTB;                                                     // hold current PORT B value (led strip could have changed PORT B I/O selection bits at the time this ISR occurs) 
    uint8_t holdPortDduringInt = PORTD;                                                     // hold current PORT D value (LCD driver can be updating PORT D in main loop at the time this ISR occurs) 

    static uint8_t prevButtonStates{ pinD_keyBits };
    static uint16_t keyDownTimer{ 0 };                                                      // milliseconds - on-board cancel key only

    // if instructed by main loop, reset hardware watchdog
    // for testing purposes, this indicates the occurrence of the timer 1 overflow event as well

    if (resetHWwatchDog) {                                                                  // HW watchdog is reset by square wave generated
        portDbuffer = portDbuffer | portD_interruptInProgressBit;                           // this signals occurrence of the timer 1 overflow event as well (for testing purposes)
    }
    else { portDbuffer = portDbuffer & ~portD_interruptInProgressBit; }                     // watchdog not reset (indicates software or hardware issue)
    resetHWwatchDog = false;
    PORTD = portDbuffer;

    PORTB = ((PORTB & ~portB_IOchannelSelectBitMask) | portB_auxFlipFlopSelect);            // PORT B: select aux flip flops
    PORTC = (PORTC & ~portC_IOdisableBit);                                                  // clock signal for aux flip flops LOW then HIGH
    PORTC = (PORTC | portC_IOdisableBit);


    // flag that an ISR has run: only needed for idle time counting (signal that ISR duration must be deducted from idle time)
    ISRoccurred = true;

    // measure vertical position: initiate ADC conversion hall sensor value (always at the same time relative to timer 1 overflow: do it now)
    // NOTE: ISR should start at timer 1 overflow, but can be slightly delayed (< 10 microseconds) because of idle loop ...
    // ... and other places where interrupts are briefly disabled
    ADMUX = (B01 << REFS0) | (B0000 << MUX0);                                               // internal Nano 5 Volt reference, port A0
    ADCSRA |= (1 << ADEN) | (1 << ADSC) | (1 << ADIE) | (B111 << ADPS0);                    // enable ADC, start conversion, enable interrupt at completion, prescaler factor 128
    ADCisTemp = false;                                                                      // indicate this is a hall sensor measurement (not a temperature measurement)


    // read switches and Greenwich position sensor
    PORTD = PORTD | B11111100;                                                              // PORT D pins 2 to 7: prepare to enable pull ups   
    DDRD = DDRD & B00000011;                                                                // PORT D pins 2 to 7: inputs (pins 0 and 1: serial I/O)

    PORTB = ((PORTB & ~portB_IOchannelSelectBitMask) | portB_switchesBufferSelect);         // PORT B: select switch buffers
    PORTC = (PORTC & ~portC_IOdisableBit);                                                  // enable switch buffers

    dataInBuffer = PIND;                                                                    // read switches twice (stall) - allow for setup time (see Atmel data sheet)
    dataInBuffer = PIND; // (read twice)

    PORTC = (PORTC | portC_IOdisableBit);                                                   // disable switch buffers

    DDRD = DDRD | B11111100;                                                                // PORT D pins 2 to 7: outputs (pins 0 and 1: serial I/O)

    PORTB = holdPortBduringInt;                                                             // restore port B contents
    PORTD = holdPortDduringInt;                                                             // restore port D contents


    // debounce switches / keys and produce (1) switch states and (2) key codes for pressed / released keys
    if ((millis16bits & B1111) == 0) {                                                      // 16 mS debounce time
        switchStates = dataInBuffer & pinD_switchStateBits;                                 // debounced

        if (useButtons) {                                                                   // interpret signals SW3 to SW0 as buttons ? (corresponding 4 switches should all remain in the OFF (= high) position)
            // produce key code for last pushbutton pressed (+) or released (-)
            uint8_t keyNumber = 0;
            uint8_t buttonsActioned = ((switchStates ^ prevButtonStates) & pinD_keyBits);   // new button press / release detected ?
            uint8_t buttonsPressed = ((~switchStates) & prevButtonStates & pinD_keyBits);   // button press only (no release)
            while (buttonsActioned) {                                                       // press or release detected - safety (should always break)
                keyNumber++;
                if (buttonsActioned & pinD_firstKeyBit) {
                    int8_t keyPressed = (buttonsPressed & pinD_firstKeyBit) ? keyNumber : -keyNumber;
                    if (keyPressed > 0) { keyDownTimer = 1; }                               // key press: (re-)start counting (even if currently other key down)
                    else {                                                                  // key release: determine key press duration and stop counting (even if currently other key down)
                        if (keyDownTimer > 700) { keyPressed -= (int8_t)0x40; }             // clear bit 6
                        if (keyDownTimer > 2000) { keyPressed -= (int8_t)0x20; }            // clear bit 5 as well
                        keyDownTimer = 0;                                                   // disable counting
                    }
                    if (keysAvailable < keyBufferLength) {                                  // key press ignored if buffer full
                        keyBuffer[keysAvailable] = keyPressed;
                        keysAvailable++;                                                    // available for read
                    }
                    break;
                }
                buttonsActioned = buttonsActioned >> 1;                                     // buttons pressed or released
                buttonsPressed = buttonsPressed >> 1;                                       // buttons pressed
            }
        }
        prevButtonStates = switchStates;
    }

    if ((keyDownTimer > 0) && (keyDownTimer <= 2000)) { keyDownTimer++; }

    // time counting (we don't use Arduino time functions based on timer 0)
    milliSecond++;
    if (milliSecond == oneSecondCount) {
        milliSecond = 0;
        second++;
    }

    millis16bits++;                                                                         // let it overflow
}


// *** ADC conversion complete interrupt (ISR) ***

// Normally, the 'ADC complete' ISR runs every millisecond, as soon as the hall sensor ADC conversion initiated by the timer 1 overflow event (which occurs every mS) is available
// Every 128 mS, this 'ADC complete' event itself initiates a second ADC conversion (temperature), which triggers an 'extra' ADC complete event in the same millisecond ...
// ... which will only store this temperature measurement and exit immediately
// the next 'regular' 'ADC complete' event (after the next timer 1 overflow event) will then pick up that temp. value to pass it on to main, together with other values

// 'ADC complete' ISR flow:
// read previously converted ADC value (can be vertical position hall sensor or temperature sensor) 
// -> if it was a temp. measurement, store it and RETURN
// -> if it was a hall sensor measurement, store it and CONTINUE :
// control globe vertical position (PID controller)
// control rotation
// handle safety (dropped globe, sticky globe, high temp, high magnet duty cycle)
// every 128 mS: initiate ADC conversion temp. sensor
// communicate with main loop: events
// measure ISR duration (also check for passing 0.5 mS)

SIGNAL(ADC_vect) {

    // for ISR speed, use long variables instead of floats where possible -> add accuracy by adding extra digits as binary fraction
    // divisions take much longer than multiplications and need to be avoided (e.g. division by cst 5 equals multiplication with cst (1/5 * 2^8) and then shifting bits right 8 bits   
    // extra digits must be carefully chosen because this decreases most significant bits leading to overflow
    // example: TTTgain is less than 1, which is too small for integer * and /, so we add 8 binary digits, representing binary fraction

    constexpr int PIDcalculation_BinaryFractionDigits{ 14 };                                    // added accuracy (binary fraction digits) in PID controller calculations
    constexpr int PIDcalc_preliminaryDivisionDigits{ 4 };                                       // to prevent overflow after multiplication (factor 1/2: keep 1 extra bit for safety)


    // PID controller
    // --------------

#if highAnalogGain                                                                              // compensate for higher analog gain
    constexpr long hallRange_ADCsteps{ (300 * analogGain) / 10 };                               // maximum deviation from hall reference (set point) used in calculations to prevent integer variable overflow, in ADC steps
    constexpr long floatingGlobeHallRange_ADCsteps{ (100 * analogGain) / 10 };                  // maximum deviation from hall reference (set point) to check for 'non-floating' condition, in ADC steps
#else
    constexpr long hallRange_ADCsteps{ 300 };                                                   // maximum deviation from hall reference (set point) used in calculations to prevent integer variable overflow, in ADC steps
    constexpr long floatingGlobeHallRange_ADCsteps{ 100 };                                      // maximum deviation from hall reference (set point) to check for 'non-floating' condition, in ADC steps
#endif

    constexpr int contrOutSteps{ timer1Top };                                                   // 16 bit timer1: value for 1 millisecond
    constexpr int disabledMagnetOnCycles{ 0 };                                                  // magnet disabled during error condition
    constexpr int minMagnetOnCycles{ 7 };                                                       // ensure electromagnet is always on (stable) during hall sensor reading
    constexpr int maxMagnetOnCycles{ contrOutSteps - 8 };                                       // leave a small spike for test purposes (visible on scope)

    static long hallReading_ADCsteps{};
    static long errorSignal{ 0 }, errorSignalPrev{ 0 };                                         // error signal, previous error signal 
    static long TTTintTerm{ 0 }, TTTdifTerm{ 0 };                                               // true three term PID controller integrator and differentiator term
    static long sumErrSignalMagnitude{ 0 };                                                     // measure for stability vertical position

    bool isGreenwich{ false };
    int TTTcontrOut{ 0 };                                                                       // controller out (value between minMagnetOnCycles and minMagnetOnCycles)


    // globe rotation controller
    // -------------------------

    constexpr long defaultGreenwichDebounceTime{ 800L };                                        // debounce time for globe rotation position sensor, milliseconds
    constexpr long maxGlobeRotationTime{ 20000 };                                               // in milliseconds

    static bool GreenwichPositionSync{ false };
    static uint8_t lastTurnsInAutoLockRangeCount{ 0 };
    static uint8_t holdRotationStatus{ rotNoPosSync }, holdErrorCondition{ errNoError };
    static int stepNo{ 0 };                                                                     // current period of software (slow) timer (in sample periods)
    static int greenwichBounceTimer{ 0 };                                                       // position sensor reading (true = sync)
    static long globeRotationTimeCount{ 0 };
    static long lockedRotations{ 0 }, lockedSecond{ 0 }, liftingSecond{ 0 };
    static long greenwichLag{ 0 };
    static long globeRotationTime{ 0 }, rotationOutOfSyncTime{ 0 };

    // safety and associated readings
    // ------------------------------

    constexpr long maxTemp = { 6500L };                                                         // SAFETY: maximum allowed lifting magnet temperature (degrees x 100)
    constexpr long allowedDroppedOrStickyGlobePeriod{ 5000L };                                  // SAFETY: maximum allowed dropped or sticky globe time, in milliseconds
    constexpr long allowedNonFloatingGlobePeriod{ 250L };                                       // maximum non-floating globe time before setting 'non-floating' status, in milliseconds
    constexpr long enforcedMinDutyCyclePeriod{ 300L };                                          // SAFETY: minimum period with enforced minimum lifting magnet duty cycle, in milliseconds

    static bool liftingMagnetEnabled{ true }, requestEnableMagnet{ false };                     // SAFETY: lifting magnet enabled or disabled
    static bool isFloating{ false }, holdIsFloating{ false };
    static long sumADCtemp{ 0 };
    static long minMagnetEnablingDelay{ -2 };                                                   // -2: wait for upward globe movement in lifting magnet re-enabling sequence
    static long droppedGlobePeriodCount{ 0 };                                                   // SAFETY: measured dropped globe time, in milliseconds 
    static long stickyGlobePeriodCount{ 0 };                                                    // SAFETY: measured sticky globe time, in milliseconds 
    static long lockedMilliSecond{ 0 }, liftingMilliSecond{ 0 }, errorLedSequence{ 0 };
    static long sumIdleLoopMicros{ 0 }, sumMagnetOnCycles{ 0 }, sumISRdurations{ 0 };


    // on board led and dimming 
    // ------------------------

    bool blueLedOn{ false }, greenLedOn{ false }, redLedOn{ false };


    // *** start execution ***
    // =======================

    // measure time between T1 clock overflow and ISR start 
    // only relevant if ISR in response to 'hall sensor conversion complete'
    int ISRstart = TCNT1;                                                                       // safe to assume that timer 1 is counting up and not yet counting down (just started counting)                                                                                              

    // only for idle time counting (signal that ISR duration must be deducted from idle time)
    ISRoccurred = true;

    // read previously converted ADC value (can be vertical position hall sensor or temperature sensor)
    uint8_t ADlow = ADCL;
    uint8_t ADChigh = ADCH;
    unsigned int ADCfull = (ADChigh << 8) | ADlow;

    // second ADC conversion complete interrupt after timer 1 interrupt: store temperature value for further processing

    long prevHallReading_ADCsteps;

    if (ADCisTemp) {                                                                            // converted ADC value is temperature
        // ISR time is not measured here
        sumADCtemp = ADCfull;
        return;                                                                                 // nothing more to do
    }
    else {                                                                                      // converted ADC value is lifting hall-sensor value
        prevHallReading_ADCsteps = hallReading_ADCsteps;
        hallReading_ADCsteps = ADCfull;
    }


    // *** hall sensor value read: continue with normal flow (lift & rotation control, safety, events, ...) ***
    // ========================================================================================================

    uint8_t holdPortBduringInt = PORTB;                                                         // hold current PORT B value (led strip could have changed PORT B I/O selection bits at the time this ISR occurs) 
    uint8_t holdPortDduringInt = PORTD;                                                         // hold current PORT D value (LCD driver can be updating PORT D in main loop at the time this ISR occurs) 

    if (printPIDtimeCounter > printPIDperiod) {                                                 // no step response test being conducted: normal behavior
        if ((hallRef_ADCsteps != targetHallRef_ADCsteps) && ((millis16bits & B111111) == 0)) {  // vertical position: slowly adapt controller reference to a changed target reference (one step every 64 mS)  
            (hallRef_ADCsteps < targetHallRef_ADCsteps) ? hallRef_ADCsteps++ : hallRef_ADCsteps--;
        }
    }
    else if (printPIDtimeCounter == PIDstepTime) {                                              // counted from data logging start 
        if (applyStep) { hallRef_ADCsteps = targetHallRef_ADCsteps + 41; }                      // +200 mV
    }

    if (hallReading_ADCsteps < hallRef_ADCsteps - hallRange_ADCsteps) { hallReading_ADCsteps = hallRef_ADCsteps - hallRange_ADCsteps; } // bring measured position within range, because integer calc. with limited accuracy
    else if (hallReading_ADCsteps > hallRef_ADCsteps + hallRange_ADCsteps) { hallReading_ADCsteps = hallRef_ADCsteps + hallRange_ADCsteps; }


    // *** control globe vertical position and rotation ***
    // ====================================================

    // lifting magnet status is enabled ? control globe levitation and rotation
    if (liftingMagnetEnabled) {

        // (1): Control system for lifting magnet (PID)
        // --------------------------------------------
        errorSignalPrev = errorSignal;                                                                          // remember previous value of error signal 
        errorSignal = -((hallRef_ADCsteps - hallReading_ADCsteps) << PIDcalculation_BinaryFractionDigits);      // new error signal in ADC steps

        TTTintTerm = TTTintTerm + ((TTTintFactor * errorSignal) >> TTTintFactor_BinaryFractionDigits);          // integrator term
        TTTintTerm = max(TTTintTerm, 0L);                                                                       // limit values: positive
        TTTintTerm = min(TTTintTerm, maxTTTintTerm << PIDcalculation_BinaryFractionDigits);                     // (disturbance = weight of object always acts in same direction = downwards)

        TTTdifTerm = (TTTdifFactor * (errorSignal - errorSignalPrev)) >> TTTdifFactor_BinaryFractionDigits;     // differentiator term

        // controller output becomes magnet duty cycle (0 = magnet completely OFF)
        long TTTallTerms = errorSignal + TTTintTerm + TTTdifTerm;                                               // after multiplying with TTTgain, result can be > LONG_MAX: 

        if (abs(TTTallTerms) > maxTTTallTerms) {                                                                // prevent overflow after multiplication (factor 1/2: keep 1 extra bit for safety)
            TTTallTerms = TTTallTerms >> PIDcalc_preliminaryDivisionDigits;                                     // get rid of accuracy in two steps 
            TTTcontrOut = (int)((TTTgain * TTTallTerms) >> (gain_BinaryFractionDigits + PIDcalculation_BinaryFractionDigits - PIDcalc_preliminaryDivisionDigits));      // TTT controller output
        }
        else {
            TTTcontrOut = (int)((TTTgain * TTTallTerms) >> (gain_BinaryFractionDigits + PIDcalculation_BinaryFractionDigits));  // TTT controller output
        }
        TTTcontrOut = max(TTTcontrOut, minMagnetOnCycles);                                                      // limit duty cycle to values within 0 - 100% 
        TTTcontrOut = min(TTTcontrOut, maxMagnetOnCycles);

        OCR1A = TTTcontrOut;

        // (2): control system for rotating magnetic field
        // -----------------------------------------------
        // - within 'auto locking' range(close to set rotation time) the system is SELF-controlling and LOCKING to the rotating magnetic field (NO slip)
        // - outside this range, adjust the phase of the rotating magnetic field to the phase (fixed value) appearing during auto-locking (ACTIVE controlling)
        // - outside an even wider range, adjust magnetic field phase AND rotation time. This changes rotation time faster to setpoint and ...
        // ... eliminates risk of locking to a rotation speed 3 times faster than the set rotation time (which is also a stable, but undesired, situation)

        if (targetGlobeRotationTime > 0) {

            bool fastSpeed = globeRotationTime < 2000L;                                         // 1 rotation faster than 2 seconds ?
            long greenwichDebounceTime = fastSpeed ? (defaultGreenwichDebounceTime >> 1) : defaultGreenwichDebounceTime;

            greenwichBounceTimer++;                                                             // measure globe rotation time
            if (greenwichBounceTimer > greenwichDebounceTime) { greenwichBounceTimer = greenwichDebounceTime; }

            bool g = (dataInBuffer & pinD_greenwichBit) == pinD_greenwichBit;                   // 1 if fixed globe position (magnet) is currently passing sensor 
            isGreenwich = g && (!GreenwichPositionSync) &&
                (greenwichBounceTimer >= greenwichDebounceTime);                                // fixed globe position (magnet) just STARTED passing sensor (position sync) ?
            if (g != GreenwichPositionSync) { greenwichBounceTimer = 0; }                       // debounce: reset timer on both edges (XOR)
            GreenwichPositionSync = g;                                                          // remember


            // is this a globe position sync ?
            // -------------------------------
            if (isGreenwich) {                                                                  // is a globe position sync (magnet just STARTED passing sensor)

                // switch: test status of completed turn and, if condition is met, set status for next turn
                switch (rotationStatus) {


                    case rotNoPosSync:                                                          // no position sync since a while
                    case rotFreeRunning: {                                                      // NOT USED                        
                        lastTurnsInAutoLockRangeCount = 0;                                      // reset counter
                        globeRotationTimeCount = 0;                                             // start counting time of current globe rotation
                        rotationStatus = rotMeasuring;                                          // measure time of first rotation
                    } break;

                    // last turn was only measuring time
                    case rotMeasuring: {
                        rotationStatus = rotUnlocked;                                           // unlock
                        // no break here: continue

                    // last turn was unlocked 
                    case rotUnlocked: {
                        globeRotationTime = globeRotationTimeCount + 1;                         // time of last full globe rotation, in sampling periods
                        globeRotationTimeCount = 0;                                             // start counting time of current globe rotation

                        // set magnetic field rotation time (coils)
                        // ----------------------------------------
                        // if globe rotation time is outside a calculated 'band', adapt magnetic field rotation time:
                        // if globe rotation time is much too slow ( time >  threshold > target rotation time), than set a higher magnetic field rotation time
                        // if globe rotation time is much too fast ( time <  threshold < target rotation time), than set a lower magnetic field rotation time
                        // if in center speed range, set target rotation time

                        const long stepTimeCurrentRotation = globeRotationTime / stepCount;                     // current globe rotation time, in milliseconds        
                        const bool forceSlowDown = (globeRotationTime <= slowDown_maxGlobeRotationTime);        // set a target rotation time slightly higher than measured (slowing down)
                        const bool forceSpeedUp = (globeRotationTime >= speedUp_minGlobeRotationTime);          // set a target rotation time slightly lower than measured (speeding up)

                        stepTimeNewRotation = targetStepTime;                                                   // init
                        if (forceSlowDown || forceSpeedUp) {
                            const int index{ forceSlowDown ? 0 : 1 };
                            float slope = (speedRatioSlowTurns[index] - speedRatioFastTurns[index]) / (12000. - 1000.);
                            float speedRatio = speedRatioFastTurns[index] + slope * (globeRotationTime - 1000.);
                            stepTimeNewRotation = stepTimeCurrentRotation / speedRatio;
                        }

                        // if globe rotation time is inside a calculated 'band' (narrower than the band used to change magnetic field rotation time), flag this rotation as 'in autolock range' (but not yet locked)
                        else {
                            bool thisTurnInAutoLockRange = (globeRotationTime > autoLock_minGlobeRotationTime) && (globeRotationTime < autoLock_maxGlobeRotationTime); // check if in auto locking range
                            uint8_t requiredTurnsInAutoLockRange{ (targetGlobeRotationTime >= 3000) ? 4 : 10 };
                            lastTurnsInAutoLockRangeCount = thisTurnInAutoLockRange ? min(lastTurnsInAutoLockRangeCount + 1, requiredTurnsInAutoLockRange) : 0;

                            // determine lock status and calculate sync error (when locked)
                            if (lastTurnsInAutoLockRangeCount >= requiredTurnsInAutoLockRange) {    // minimum number of globe rotations in autolock range ?
                                lockedRotations = 0;
                                rotationOutOfSyncTime = 0;
                                greenwichLag = 0;
                                rotationStatus = rotLocked;
                                break;
                            }
                        }

                        // -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
                        // adjust the phase angle between the rotating globe and the rotating magnetic field generated by the six coils on each rotation.
                        // the phase angle sets the starting angle of the rotating magnetic field (0 to 359 degrees), each time the reference meridian (the meridian where the 'Greenwich' magnet is located...
                        // ...on the rotating globe) is detected by the hall sensor - WHICH IS NOW.

                        // the phase angle is defined (in code, user adjustable) separately for lower and higher rotation speeds, and for speeding up / slowing down (4 defined values, in degrees).
                        // note: the phase angle to be set depends on the position of the hall sensor detecting the reference ('Greenwich') meridian.
                        //       if the hall sensor is moved in the direction of the rotation, the phase angle will decrease, and vice versa.
                        //       a phase adjustment setting is available for the user to correct for the position of the hall sensor.
                        //
                        // the set phase angle is converted to stepNo (0 to 11) and rotationTimerSamplePeriod (fraction of a step) and counting starts again from there.
                        // rotationTimerSamplePeriod value equals the required number of milliseconds (timer interrupts)
                        // the actual orientation of the coil pairs magnetic fields is only changed when a full 'next' step is reached, per implementation (at the latest after 1/12 of one rotation, or 30 degrees).

                        // NOTE: when unlocked, the phase lag is ALWAYS calculated and set at every turn of the globe, irrespective of the current globe rotation time.

                        // example: a phase angle lag of 140 degrees is calculated. This is translated to 4.67 steps (140 * 12 / 360: one rotation corresponds to 12 steps).
                        //          when the 'Greenwich' meridian passes the hall detector, the magnetic field should be set to 4.77 steps and counting up should be continue from there.
                        //          in reality, the orientation of the magnetic field is set at the next full step, as per implementation (step 5 in this example).
                        // -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

                        // calculated phase to set (degrees) = first degree term * rotation speed (rotations / second) + constant term (degrees)
                        const int index = (globeRotationTime < targetGlobeRotationTime) ? 0 : 1;
                        const long firstDegreeTerm = globeRotationLag_slope[index];      // slope    
                        const long constantTerm = globeRotationLag_1s[index];      // offset    

                        // phaseAdjustSteps: 0 to 179: angle in 2-degree units (1: 2 degrees, ... 179: 358 degrees = -2 degrees)
                        long phaseToSet = (firstDegreeTerm * 1000) / globeRotationTime + constantTerm + (((phaseAdjustSteps << 1)) % 360);      // degrees 
                        stepNo = phaseToSet * stepCount / 360;                                                                                  // integer division: 0 to step count - 1
                        rotationTimerSamplePeriod = (phaseToSet * stepTimeNewRotation * stepCount) / 360 - stepNo * stepTimeNewRotation;        // integer division: accuracy !
                    } break;


                    // last turn was locked: keep track of sync error and unlock if above limit
                    case rotLocked: {

                        lockedRotations++;
                        globeRotationTime = globeRotationTimeCount + 1;                         // time of last full globe rotation, in sampling periods
                        globeRotationTimeCount = 0;                                             // start counting time of current globe rotation

                        // ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
                        // when rotation is locked, the magnetic field generated by the 6 coils rotates at a fixed pace. This is the reference.
                        // The rotation time of each successive 360-degrees turn of the globe (displayed as 'actual rotation time') will slowly converge to this (coils) magnetic field rotation time,...
                        //... oscillating around it with a decreasing amplitude.
                        // This actual globe rotation time can be measured at each passing of the globe's 'Greenwich' magnet (signaling passing of the Greenwich meridian).

                        // Once the globe rotation time has converged to the set rotation time (coils magnetic field), ideally, the globe's two 'rotation' magnets will reside more or less in the center point...
                        // ...of the two poles of the rotating magnetic field. If the globe rotates a little too fast/too slow, the magnets will slow down/speed up globe rotation (and vice versa).
                        // But there's also the air resistance, which increases quadratically with the globe's rotation speed. The resulting force needs to be counteracted to get an equilibrium again:
                        // for higher speeds, the globe's rotation will lag a little more behind (resulting in a slightly higher force pushing the globe forward).

                        // The magnetic field angle is defined as zero when the step number and rotation timer sample period are set to 0.
                        // The globe rotation angle is set to zero when the 'Greenwich' magnet passes the detector.
                        // We define the magnetic field rotation as 'leading' be a certain angle if the magnetic field angle is less than 180 degrees when the glob's angle is zero.
                        // The globe rotation is then lagging behind.
                        // Fast (slow) rotation times will increase (decrease) the globe rotation lag (lead), because of increased (decreased) air resistance when the globe is turning faster (slower).

                        // Empirically the relationship between speed and globe rotation lag is found to be linear:
                        // globe rotation lag (degrees) = 51.167 x rotation speed (full rotations / second) + 93.078 degrees

                        // Just like the actual globe rotation time, the globe rotation lag is measured at each passing of the globe's 'Greenwich' magnet.
                        // rotation lag (degrees) = ((stepNo * stepTime) + rotationTimerSamplePeriod) * 360 / step count       => when the Greenwich magnet passes the detector

                        // There is a linear relationship between the globe rotation sync error (the result of adding up individual GLOBE rotation times) and the globe rotation lag.
                        // But in contrast to the globe rotation lag (degrees), which will always converge to the same angle (in similar circumstances, for a specific rotation speed), ...
                        // ...the sync error (time) is not, because it is set to zero without taking into account the starting globe rotation lag angle with respect to the magnetic field rotation...
                        // ...when the status switches to 'locked'.

                        // Globe rotation lag (in degrees) can be very useful as a starting point when defining the 'speed up' and 'slow down' phase adjust constants.
                        // ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

                        rotationOutOfSyncTime = rotationOutOfSyncTime + (targetGlobeRotationTime - globeRotationTime);
                        bool withinTolerance = (4L * abs(rotationOutOfSyncTime) <= (targetGlobeRotationTime));      // total deviation less than 1/4 rotation (targetStepTime * steps equals 1 rotation)
                        if (!withinTolerance) { rotationStatus = rotUnlocked; lastTurnsInAutoLockRangeCount = 0; break; }

                        greenwichLag = (stepNo * targetStepTime) + rotationTimerSamplePeriod;   // converted to, and displayed in degrees in main loop
                    } break;
                    }
                }
            }
            // is NOT a globe position sync 
            // ----------------------------
            else {
                if ((rotationStatus == rotMeasuring) || (rotationStatus == rotUnlocked) || (rotationStatus == rotLocked)) {
                    if (globeRotationTimeCount <= (maxGlobeRotationTime)) {                     // check for time out since last globe position sync
                        globeRotationTimeCount++;                                               // no time out yet: continue counting time of current globe rotation 

                        if (rotationStatus == rotLocked) {
                            long cumul = rotationOutOfSyncTime - globeRotationTimeCount;
                            bool withinTolerance = ((4L * cumul) > (-5L * targetGlobeRotationTime));    // time out: total deviation more than 1/4 rotation = 5/4 rotation since last position sync 

                            if (!withinTolerance) {                                             // only when moving from locked --> unlocked
                                lastTurnsInAutoLockRangeCount = 0;                              // for next globe position sync timer event
                                rotationStatus = rotUnlocked;                                   // rotation NOT in sync with clock any more
                            }
                        }
                    }

                    // no globe position sync since quite some time (time out) ? set free-running mode
                    else { rotationStatus = rotNoPosSync; }
                }
            }


            // set coil pair magnetic field states
            // -----------------------------------
            if (rotationTimerSamplePeriod >= stepTimeNewRotation) {                             // >= : safety (timer should never be greater than step time)
                rotationTimerSamplePeriod = 0;

                stepNo++;
                if (stepNo >= stepCount) { stepNo = 0; }                                        // >= : safety (step no should never be greater than stepCount)

                bool coilOnSouth, coilOnNorth;                                                  // 3 coil pairs can be set individually to N-S, S-N or not engaged
                uint8_t coils{ 0 };

                for (int coilNo = 0; coilNo < 3; coilNo++) {                                    // south means attracting / pointing to a magnet's north pole

                    uint8_t coilStep = (2 * (uint8_t)coilNo + (uint8_t)stepNo);
                    if (coilStep >= stepCount) { coilStep = coilStep - stepCount; }             // avoid modulo operator (needing division - speed)

                    // NNNSSS, rotating field, smooth transition for coils switching poles
                    coilOnNorth = (coilStep >= 6) && (coilStep <= 11);                          // last parameter 10 if intermediate stepNo with 2 coils off  
                    coilOnSouth = (coilStep >= 0) && (coilStep <= 5);                           // last parameter 4 if intermediate stepNo with 2 coils off

                    if (coilOnNorth) { coils = coils | _BV(3 + 2 * coilNo); }
                    if (coilOnSouth) { coils = coils | _BV(2 + 2 * coilNo); }
                }

                coils = ~coils;                                                                 // negative logic
                portDbuffer = (portDbuffer & B00000011) | (coils & B11111100);                  // prepare coil on / off information

                PORTD = portDbuffer;                                                            // port D bits 765432 = coil wires 2B (bit 7), 2A, 1B, 1A, 0B, 0A (bit 2) - see schematic 


                PORTB = ((PORTB & ~portB_IOchannelSelectBitMask) | portB_coilFlipFlopSelect);   // PORT B: select coils flip flops
                PORTC = (PORTC & ~portC_IOdisableBit);                                          // clock signal for coil flip flops LOW then HIGH
                PORTC = (PORTC | portC_IOdisableBit);
            }

            rotationTimerSamplePeriod++;
        }

        // determine RGB status led state while not in error mode
        // ------------------------------------------------------

        bool stepTick = ((rotationTimerSamplePeriod < 20) && !(stepNo & B1));                   // signals new step (rotating magnetic field)
        bool greenwichTick = (globeRotationTimeCount < 60L);                                    // magnet passes sensor
        bool dimmed = (millis16bits & B111) == B0000;                                           // dimmed, 1/8 on

        switch (rotationStatus) {
            case rotNoPosSync:                                                                  // no position sync since a while
                // green led is either ON or flashing (period = 256 mS, 1/4 on), depending floating status
                greenLedOn = isFloating ? (targetGlobeRotationTime != 0) || (liftingSecond <= 5) : (millis16bits & 0b11111111) < 0b111111;  // not floating: green led flashes, period = 256 mS, 1/4 on
                break;
            case rotFreeRunning:                                                                // NOT USED
                greenLedOn = !stepTick;
                break;
            case rotMeasuring:                                                                  // measuring
                // measuring: green led flickers, period = 64 mS, 3/4 on
                greenLedOn = (millis16bits & B111111) >= B010000;
                break;
            case rotUnlocked:                                                                   // not locked
                // blue led signals step ticks; green led signals magnet passing sensor ('Greenwich' event)
                blueLedOn = (dimmed || stepTick) && !greenwichTick;
                greenLedOn = greenwichTick;
                break;
            case rotLocked:                                                                     // locked
                // LED OFF
                break;
        }
    }


    // lifting magnet status is disabled: process error condition

    else {                                                                                      // error condition: lifting magnet not enabled                                                              
        TTTcontrOut = disabledMagnetOnCycles;
        OCR1A = disabledMagnetOnCycles;                                                         // init: set minimum duty cycle for magnet

        blueLedOn = (milliSecond < 500);
        if (milliSecond == 0L) {
            errorLedSequence++;
            if (errorLedSequence >= errorCondition + 1) { errorLedSequence = 0; }               // pause blue led flash after No of flashes indicating error type
        }

        if (errorLedSequence == 0) { blueLedOn = false; }
        redLedOn = greenLedOn = false;

        // enable lifting magnet again, after a short delay, when hall detector has sensed main globe magnet (globe upward movement passing specific point, then downward movement passing other point further down)
        switch (minMagnetEnablingDelay) {
            case -2:
                if ((prevHallReading_ADCsteps >= hallRef_ADCsteps - floatingGlobeHallRange_ADCsteps) && (hallReading_ADCsteps < hallRef_ADCsteps - floatingGlobeHallRange_ADCsteps)) { minMagnetEnablingDelay = -1; }   // upward globe movement ?
                break;
            case -1:
                if ((prevHallReading_ADCsteps <= hallRef_ADCsteps + floatingGlobeHallRange_ADCsteps) && (hallReading_ADCsteps > hallRef_ADCsteps + floatingGlobeHallRange_ADCsteps)) { minMagnetEnablingDelay = 0; }    // downward globe movement ?
                break;
            default:
                minMagnetEnablingDelay++;
                break;
        }
        if (minMagnetEnablingDelay > enforcedMinDutyCyclePeriod) { minMagnetEnablingDelay = enforcedMinDutyCyclePeriod; }
        requestEnableMagnet = (minMagnetEnablingDelay == enforcedMinDutyCyclePeriod);
        if (requestEnableMagnet) { minMagnetEnablingDelay = -2; }                               // re-initialize
    }


    // *** led strip led dimming ***
    // ============================

    // check whether it's time to increase / decrease brightness values
    // brightness values are assigned to leds / colors in main loop, and led strip is written to in main loop as well (because time consuming)

    if ((LSminBrightnessLevel < LSmaxBrightnessLevel) && (LScolorCycle >= cWhiteBlue)) {        // brightness changing over time ?
        if ((!LSlongTimeUnit) || ((millis16bits & 0b1111111) == 0)) {
            // handle 'brightness freeze' time

            LSbrightnessFreezeTimer += LStransitionStops;                                               // count brightness freeze time, scaled by no of freeze intervals in a cycle 
            if (LSbrightnessFreezeTimer >= 0) { LSbrightnessFreezeTimer -= LSbrightnessCycleTime; }     // reset timer (LSbrightnessCycleTime = dim/freeze cycle time scaled by no of freeze intervals in a cycle) 
            bool LSdimStopped = (LSbrightnessFreezeTimer >= -LSbrightnessFreezeTime);                   // dimming currently stopped ? (brightness freeze)
            if (!LSdimStopped) {

                // brightness step timer triggers move to a next brightness step
                // brightness can increase, decrease or stay constant (min. and max. brightness levels) when moving to a new step

                // algorithm prevents divisions by scaling the time counted by the number of brightness steps 
                // Note - transition time and transiton steps : includes constant brightness time and steps as well

                // brightness step no = current time x total number of transition steps / total transition time (linear relation between step and time)
                // => brightness step x  total transition time = current time x total number of transition steps (up, down, cst brightness)
                // => we do not count the time, but the time scaled by total number of transition steps
                // if result > 1 x transition time: brightness step = 1
                // if result > n x transition time: brightness step = n
                // each time we move to the next brightness step, we reset the brightness step timer (subtract 1 x transition time, which is the step time x the number of transition steps)
                // => if result > 0 : move to the next brightness step

                for (uint8_t i = 0; i < LSbrightnessItemCount; i++) {
                    if ((LScolorCycle == cWhiteBlue) && (i > 0)) { continue; }                          // brightness values either fixed or adapted outside this loop ? skip 

                    LSbrightnessStepTimer[i] += LSbrightnessTransitionSteps;                            // count step time, scaled by no of brightness steps 
                    if (LSbrightnessStepTimer[i] > 0) {                                                 // time to start next brightness step ?
                        LSbrightnessStepTimer[i] -= LSbrightnessTransitionTime;                         // reset step timer (transition time = brightness step time scaled by total no of brightness steps)

                        if ((LScolor[i] == LSminBrightnessLevel) && !(LSup & (1 << i))) {          // min. brightness and counting down ?
                            if (LSminBrightnessStepNo[i]++ == LSbrightnessMinLvlSteps) {                // if specified, keep this brightness cst for a number of brightness steps
                                LSminBrightnessStepNo[i] = 0;                                           // min. brightness period reached: reset constant brightness step counter
                                LSup = LSup | (1 << i);                                                 // now counting up again
                                LSminReached = LSminReached | (1 >> i);
                            }
                        }
                        else if ((LScolor[i] == LSmaxBrightnessLevel) && (LSup & (1 << i))) {      // max. brightness and counting up ?
                            if (LSmaxBrightnessStepNo[i]++ == LSbrightnessMaxLvlSteps) {                // if specified, keep this brightness cst for a number of brightness steps
                                LSmaxBrightnessStepNo[i] = 0;                                           // max. brightness period reached: reset constant brightness step counter
                                LSup = LSup & ~(1 << i);                                                // now counting down again
                                LSmaxReached = LSmaxReached | (1 >> i);
                            }
                        }

                        if ((LSminBrightnessStepNo[i] == 0) && (LSmaxBrightnessStepNo[i] == 0)) {       // not in a period of constant (min or max) brightness
                            (LSup & (1 << i)) ? LScolor[i]++ : LScolor[i]--;                  // increase or decrease brightness ?
                            LSupdate = LSupdate | (1 << i);                                             // flag that a brightness is changed: leds need to be re-written
                        }
                    }
                }
            }
        }
    }

    // *** safety checks ***
    // =====================

    // sticky globe 
    if ((hallReading_ADCsteps <= hallRef_ADCsteps - floatingGlobeHallRange_ADCsteps)) {
        stickyGlobePeriodCount++;
        if (stickyGlobePeriodCount > allowedDroppedOrStickyGlobePeriod) { stickyGlobePeriodCount = allowedDroppedOrStickyGlobePeriod; }
    }
    else { stickyGlobePeriodCount = 0; }

    // check 'dropped globe' condition (hall detector reading = max, OR lifting magnet is max. duty cycle)
    if ((hallReading_ADCsteps >= hallRef_ADCsteps + floatingGlobeHallRange_ADCsteps)) {
        droppedGlobePeriodCount++;                                                              // allow a minimum period with maximum duty cycle to allow handling the globe
        if (droppedGlobePeriodCount > allowedDroppedOrStickyGlobePeriod) { droppedGlobePeriodCount = allowedDroppedOrStickyGlobePeriod; }
    }
    else { droppedGlobePeriodCount = 0; }

    // calculate current error condition
    uint8_t currentErrorCondition{ errNoError };
    if (droppedGlobePeriodCount == allowedDroppedOrStickyGlobePeriod) { currentErrorCondition = errDroppedGlobe; }
    else if (stickyGlobePeriodCount == allowedDroppedOrStickyGlobePeriod) { currentErrorCondition = errStickyGlobe; }
    else if (highLoad) { currentErrorCondition = errMagnetLoad; }
    else if (tempSmooth > (maxTemp - (liftingMagnetEnabled ? 0 : 200))) { currentErrorCondition = errTemp; }    // hysteresis 2 degrees

    // decide which action to take
    bool disableNow = (liftingMagnetEnabled && (currentErrorCondition != errNoError));
    bool enableNow = (!liftingMagnetEnabled && (currentErrorCondition == errNoError) && requestEnableMagnet);
    bool cannotEnable = (!liftingMagnetEnabled && (currentErrorCondition != errNoError) && requestEnableMagnet);
    requestEnableMagnet = false;                                                                // reset

    // take action to enter or exit error condition
    if (enableSafety) {
        if (disableNow) {
            rotationStatus = rotNoPosSync;                                                      // init (rotating field OFF) 
            liftingMagnetEnabled = false;                                                       // will take effect in next cycle   
            errorCondition = currentErrorCondition;
            errorLedSequence = 0;
        }

        else if (enableNow) {
            rotationStatus = rotNoPosSync;                                                      // no position sync yet (rotating field OFF)
            TTTintTerm = initialTTTintTerm << PIDcalculation_BinaryFractionDigits;              // reset integrator term to initial value for easier globe handling

            liftingMagnetEnabled = true;                                                        // if it was disabled by overload condition
            errorCondition = errNoError;                                                        // attempt to enable magnets again
        }

        else if (cannotEnable) {
            // nothing to do
        }
    }


    // *** do some housekeeping ***
    // ============================

    // is globe floating at this time ? 
    isFloating = ((droppedGlobePeriodCount < allowedNonFloatingGlobePeriod) && (stickyGlobePeriodCount < allowedNonFloatingGlobePeriod) && (errorCondition == errNoError));

    if (isFloating) {                                                                           // globe is currently floating ?
        liftingMilliSecond++;                                                                   // counts to 1 second
        if (liftingMilliSecond == oneSecondCount) {
            liftingMilliSecond = 0;                                                             // init only
            liftingSecond++;
        }
        sumErrSignalMagnitude += abs(errorSignal);                                              // add up current error signal magnitude to total (with standard accuracy = ADC reading)
    }
    else {
        liftingMilliSecond = 0;                                                                 // not floating: keep lifting time at zero
        liftingSecond = 0;
        sumErrSignalMagnitude = 0;                                                              // do not measure average vertical position error (keep at zero)
        TTTintTerm = initialTTTintTerm << PIDcalculation_BinaryFractionDigits;                  // reset integrator term to initial value for easier globe handling
        rotationStatus = rotNoPosSync;                                                          // init rotation status (rotating field OFF)
        hallRef_ADCsteps = targetHallRef_ADCsteps;                                              // no need for smoothening transition as magnet is not lifting)
        if (printPIDtimeCounter <= printPIDperiod) { printPIDtimeCounter = printPIDperiod; }    // prepare for last print (when printPIDtimeCounter = printPIDperiod + 1)                               
    }

    if (rotationStatus == rotLocked) {
        lockedMilliSecond++;                                                                    // counts to 1 second
        if (lockedMilliSecond == oneSecondCount) {
            lockedMilliSecond = 0;
            lockedSecond++;
        }
    }
    else {
        lockedMilliSecond = 0;
        lockedSecond = 0;
    }

    // keep track of magnet ON cycles
    sumMagnetOnCycles += TTTcontrOut;

    // measure processor idle time
    sumIdleLoopMicros += (long)(idleLoopNanos500 >> 1);
    idleLoopNanos500 = 0;                                                                       // integer for speed in idle loop: reset every millisecond to prevent overflow

    // remember previous status
    bool statusChanged = (rotationStatus != holdRotationStatus) || (errorCondition != holdErrorCondition) || (isFloating != holdIsFloating);
    holdRotationStatus = rotationStatus;                                                        // remember previous status
    holdErrorCondition = errorCondition;
    holdIsFloating = isFloating;


    // *** communicate with main: events ***
    // =====================================

    // several event types can be triggered 
    // events are maintained by a FIFO event message buffer 

    uint8_t* messagePtr{ nullptr };

    // events happening at regular intervals at a relatively fast rate (multiple times per second), NOT synchronized with fixed parts of a second  
    if ((millis16bits & (fastDataRateSamplingPeriods - 1)) == 0) {
        if (myEvents.addChunk(eFastRateData, sizeof(FastRateData), &messagePtr)) {
            ((FastRateData*)messagePtr)->sumIdleLoopMicros = sumIdleLoopMicros;
            ((FastRateData*)messagePtr)->sumISRdurations = sumISRdurations;
            ((FastRateData*)messagePtr)->sumMagnetOnCycles = sumMagnetOnCycles;
            ((FastRateData*)messagePtr)->sumADCtemp = sumADCtemp;
            ((FastRateData*)messagePtr)->sumErrSignalMagnitude = sumErrSignalMagnitude >> PIDcalculation_BinaryFractionDigits;  // remove extra digits again (coming from error signal) 
            sumIdleLoopMicros = 0;                                                              // reset (note that sumADCtemp is a single meas. every 128 mS and does not need to be reset)
            sumISRdurations = 0;
            sumMagnetOnCycles = 0;
            sumErrSignalMagnitude = 0;
        }
    }

    // second event
    if (milliSecond == 0L) {
        if (myEvents.addChunk(eSecond, sizeof(SecondData), &messagePtr)) {
            ((SecondData*)messagePtr)->eventSecond = second;
            ((SecondData*)messagePtr)->liftingSecond = liftingSecond;
            ((SecondData*)messagePtr)->lockedSecond = lockedSecond;
            ((SecondData*)messagePtr)->realTTTintTerm = (TTTintTerm >> PIDcalculation_BinaryFractionDigits);    // 1 sample every second (slow moving)
        }
    }

    // time cues: events synchronized with fixed parts of a second, without any data to be transferred
    bool blink = (milliSecond == blinkTimeCount);
    bool spareEvent = false;                                                                    // disabled -> if needed, you can create more time cues, for example: bool spareEvent = (milliSecond == spareTimeCount);
    if (blink || spareEvent) {                                                                  // only one event can occur at the same time
        myEvents.addChunk(blink ? eBlink : eSpareNoDataEvent1, 0, &messagePtr);                 // cue only, no data
    }

    // status change event 
    if (statusChanged || forceStatusEvent) {
        if (myEvents.addChunk(eStatusChange, sizeof(StatusData), &messagePtr)) {
            ((StatusData*)messagePtr)->eventMilliSecond = milliSecond;
            ((StatusData*)messagePtr)->eventSecond = second;
            ((StatusData*)messagePtr)->rotationStatus = rotationStatus;
            ((StatusData*)messagePtr)->errorCondition = errorCondition;
            ((StatusData*)messagePtr)->isFloating = isFloating;
            ((StatusData*)messagePtr)->isGreenwich = isGreenwich;

            forceStatusEvent = false;                                                           // 'not floating', 'no position sync' and 'rotation OFF' share same status, so force status re-write
        }
    }

    // globe position sync event: log AFTER status event
    if (isGreenwich) {
        if (myEvents.addChunk(eGreenwich, sizeof(GreenwichData), &messagePtr)) {
            ((GreenwichData*)messagePtr)->eventMilliSecond = milliSecond;
            ((GreenwichData*)messagePtr)->eventSecond = second;
            ((GreenwichData*)messagePtr)->globeRotationTime = globeRotationTime;
            ((GreenwichData*)messagePtr)->lockedRotations = lockedRotations;
            ((GreenwichData*)messagePtr)->rotationOutOfSyncTime = rotationOutOfSyncTime;
            ((GreenwichData*)messagePtr)->greenwichLag = greenwichLag;
        }
    }

    // led strip brightness change events
    if (LSupdate) {
        if (myEvents.addChunk(eLedstripData, sizeof(LedstripData), &messagePtr)) {
            ((LedstripData*)messagePtr)->LSupdate = LSupdate;
            ((LedstripData*)messagePtr)->LSminReached = LSminReached;
            ((LedstripData*)messagePtr)->LSmaxReached = LSmaxReached;
            for (uint8_t i = 0; i <= 2; i++) { ((LedstripData*)messagePtr)->LScolor[i] = LScolor[i]; }

            LSupdate = B000;                            // reset
            LSminReached = B000;
            LSmaxReached = B000;
        }
    }

    // measure (with or w/o step response)
    if (printPIDtimeCounter <= printPIDperiod) {
        printPIDtimeCounter++;
        if (myEvents.addChunk(eStepResponseData, sizeof(StepResponseData), &messagePtr)) {
            ((StepResponseData*)messagePtr)->count = (uint16_t)printPIDtimeCounter;
            ((StepResponseData*)messagePtr)->hallReading_ADCsteps = (uint16_t)hallReading_ADCsteps;
            ((StepResponseData*)messagePtr)->TTTcontrOut = (uint16_t)TTTcontrOut;
            if (printPIDtimeCounter == 1) { firstFullAccIntTerm = (uint32_t)TTTintTerm; }       // only first value needed: do not waste message buffer space
        }
    }


    // *** set auxiliary flip flops ***
    // ================================

    // set leds on/off, switch rotating magnetic field on/off, signal end of ISR (is signal was set at ISR start to reset hardware watchdog)
    bool enableMotor = !((rotationStatus == rotNoPosSync) || (rotationStatus == rotMeasuring) || (errorCondition != errNoError));

    // set auxiliary flip flop bits
    portDbuffer = portDbuffer & ~portD_interruptInProgressBit;                                  // signal end of interrupt routine (e.g. for test purposes)

    if (enableMotor) { portDbuffer = portDbuffer | portD_enableMotorBit; }
    else { portDbuffer = portDbuffer & ~portD_enableMotorBit; }

    if (blueLedOn) { portDbuffer = portDbuffer | portD_blueStatusLedBit; }                      // led on when voltage low
    else { portDbuffer = portDbuffer & ~portD_blueStatusLedBit; }

    if (greenLedOn) { portDbuffer = portDbuffer | portD_greenStatusLedBit; }
    else { portDbuffer = portDbuffer & ~portD_greenStatusLedBit; }

#if (boardVersion == 101)                                                                       // no red led prior to board version 101
    if (redLedOn) { portDbuffer = portDbuffer & ~portD_redStatusLedbit; }                       // red led: negative logic
    else { portDbuffer = portDbuffer | portD_redStatusLedbit; }
#endif

    PORTD = portDbuffer;

    PORTB = ((PORTB & ~portB_IOchannelSelectBitMask) | portB_auxFlipFlopSelect);                // PORT B: select aux flip flops
    PORTC = (PORTC & ~portC_IOdisableBit);                                                      // clock signal for aux flip flops LOW then HIGH
    PORTC = (PORTC | portC_IOdisableBit);

    PORTB = holdPortBduringInt;                                                                 // restore port B contents
    PORTD = holdPortDduringInt;                                                                 // restore port D contents


    // *** regularly initiate ADC conversion temp. sensor ***
    // ======================================================

    // one mS before fast rate data event, so that it will be available by then

    if ((millis16bits & (fastDataRateSamplingPeriods - 1)) == (fastDataRateSamplingPeriods - 1)) {    // initiate ADC conversion temp sensor value
        ADMUX = (B01 << REFS0) | (B0001 << MUX0);
        ADCSRA |= (1 << ADEN) | (1 << ADSC) | (1 << ADIE) | (B111 << ADPS0);
        ADCisTemp = true;                                                                       // indicate this is a temp. measurement (not a hall sensor measurment)
    }


    // *** measure ISR duration (also check for passing 0.5 mS) ***
    // ============================================================

    int intServDurationInitial = TCNT1, singleISRduration;
    do {
        singleISRduration = TCNT1;
    } while (intServDurationInitial == singleISRduration);
    if (intServDurationInitial > singleISRduration) { singleISRduration = 2 * timer1Top - singleISRduration; }  // 2000 steps in 1 mS
    singleISRduration = ((singleISRduration - ISRstart) >> 1);                                  // in microseconds
    sumISRdurations += (long)singleISRduration;                                                 // add  

}

