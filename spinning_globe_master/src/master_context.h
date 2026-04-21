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


/*
===============================================================================
Spinning globe declarations shared with messageHandling library
===============================================================================
*/

#ifndef FG_MASTER_STATE_h
#define FG_MASTER_STATE_h

#include <Arduino.h>
#include <stdlib.h>
#include <util/atomic.h>                                    // atomic operations

#define highAnalogGain 1                                    // 0: analog gain is 10, 1: analog gain is 15 (defined by resistors R9 to R12)

// rotNoPosSync: also if rotation OFF or not floating; WIRE ONLY: 2 extra states in these cases
enum rotStatus :uint8_t { rotNoPosSync, rotFreeRunning, rotMeasuring, rotUnlocked, rotLocked, wire_rotOff, wire_notFloating };
enum errStatus :uint8_t { errNoError = 0, errDroppedGlobe, errStickyGlobe, errMagnetLoad, errTemp };
// eBlink, eSpareNoDataEvent1: cue only (no data) events. additional time cues can be added
enum events :uint8_t { eNoEvent = 0, eGreenwich, eStatusChange, eFastRateData, eLedstripData, eStepResponseData, eSecond, eBlink, eSpareNoDataEvent1 };
enum colorCycles :uint8_t { cLedstripOff = 0, cCstBrightWhite, cCstBrightMagenta, cCstBrightBlue, cWhiteBlue, cRedGreenBlue };      // led strip color cycle 
enum colorTiming :uint8_t { cLedstripVeryFast = 0, cLedstripFast, cLedstripSlow, cLedStripVerySlow };                               // led strip color cycle 

constexpr uint32_t DASHBOARD_ALIVE_TIMEOUT{ 30 * 1000 };    // mqtt dashboard timeout = 30 s
constexpr uint8_t settingSteps{ 32 };                       // must be even; from 0 to settingSteps        
constexpr uint8_t centerPointStep{ settingSteps / 2 };

constexpr uint32_t oneSecondCount{ 1000L }, blinkTimeCount{ 800 };  // milliseconds
constexpr uint32_t spareTimeCount{ 500 };                           // milliseconds

// ========== indexes in globe attributes list: attributes with user selectable values ==========

constexpr int attributeIndex_rotTimes{ 0 };
constexpr int attributeIndex_hallmVoltRefs{ 7 };
constexpr int attributeIndex_gainAdjust{ 13 };
constexpr int  attributeIndex_intTimeConstAdjust{ 14 };
constexpr int  attributeIndex_difTimeConstAdjust{ 15 };
constexpr int  attributeIndex_coilPhaseAdjust{ 16 };

constexpr uint8_t LSbrightnessItemCount{ 3 };               // max no of brightness values available for color led dimming 
constexpr uint8_t LSminBrightnessLevel{ 0x00 };             // min: 0 (LS off) 
constexpr uint8_t LSmaxBrightnessLevel{ 0xff };             // 0x7f or 0xff; ((value + 1)^2 / 256) - 1 = gamma corrected value = 0x3f or 0xff) 


/*v1.0.1 high speed rotation times adapted or created new*/
constexpr long const rotationTimes[] = { 0, 900, 1500, 3000, 4500, 6000, 7500, 9000, 12000 };   // values must be divisible by 12 (steps), 0 means OFF
#if highAnalogGain                                                                              // TWO limits: voltage before opamp >= 100 mV, voltage after opamp <= 2700 mV (prevent output saturation)
constexpr long const hallMilliVolts[] = { 1500, 1800, 2100, 2400, 2700 };                       // ADC setpoint expressed in mV (hall output after 15 x amplification by opamp, converted to mVolt)
#else
constexpr long const hallMilliVolts[] = { 1000, 1200, 1400, 1600, 1800 };                           // ADC setpoint expressed in mV (hall output after 10 x amplification by opamp, converted to mVolt)
#endif


// globe attributes: a list of basic settings (e.g.: set rotation time) and calculated values (e.g.: current rotation time)
// array 'globeMetricsLabels' contains the corresponding labels 

// globe attributes: '1': editable setting / '0': calculated value
constexpr long const globeMetrics_editableFlags{ 0b11110000010000001 };                         // LSB: first globe selectedAttribute in list

// globe attributes: lengths of individual value lists (0 if no value list (value range) or not a setting
constexpr int const globeMetrics_listLengths[] = { sizeof(rotationTimes) / sizeof(rotationTimes[0]), 0, 0, 0, 0, 0, 0,
    sizeof(hallMilliVolts) / sizeof(hallMilliVolts[0]), 0, 0, 0, 0, 0, 0,0,0,0 };               // 0 if no value list for selectedAttribute

// globe selectedAttribute: pointers to value lists for individual attributes
const long* const globeMetrics_valueListsPointers[] = { rotationTimes, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
    hallMilliVolts, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr };  // nullptr if no value list for globe selectedAttribute


// ========== communication between (1) ISR and main, and (2) between main and message handling ==========

struct GreenwichData {                                                              // initialize members, awaiting first event
    uint32_t eventMilliSecond{ 0 }, eventSecond{ 0 };
    long globeRotationTime{ 0 };
    long rotationOutOfSyncTime{ 0 };
    long greenwichLag{ 0 };                                                         // versus (steady) magnetic field rotation (coils), when globe rotation is locked to it
    long lockedRotations{ 0 };
};

struct StatusData {                                                                 // initialize members, awaiting first event
    uint32_t eventMilliSecond{ 0 }, eventSecond{ 0 };
    uint8_t rotationStatus{ rotNoPosSync }, errorCondition{ errNoError };
    bool isGreenwich{ false };
    bool isFloating{ false };
};

struct SecondData {                                                                 // initialize members, awaiting first event
    uint32_t eventSecond{ 0 };
    long liftingSecond{ 0 }, lockedSecond{ 0 };
    long realTTTintegrationTerm{ 0 };
};

struct FastRateData {
    long sumIdleLoopMicros{ 0 };
    long sumISRdurations{ 0 };
    long sumMagnetOnCycles{ 0 };
    long sumADCtemp{ 0 };
    long sumErrSignalMagnitude{ 0 };
    unsigned int eventMillis{ 0 };
};

struct LedstripData {
    uint8_t LSupdate, LSmaxReached, LSminReached;                                   // operational data
    uint8_t LScolor[LSbrightnessItemCount];
};

struct EventData {
    uint8_t* activeMsgPtr{ nullptr };
    uint8_t activeEventType{ eNoEvent };
    uint8_t eventsPending{ 0 }, largestEventsPending{ 0 };                          // No of ISR events currently logged for processing in main loop 
    unsigned int eventBufferBytesUsed{ 0 }, largestEventBufferBytesUsed{ 0 };       // No of ISR events logged for processing in main loop: all-time max
    uint32_t eventsMissed{ 0 };                                                     // keeps track of events missed (not used at this stage)
};

struct LedStripSettings {
    volatile uint8_t ledEffect;                                                     // led strip settings
    volatile uint8_t ledCycleSpeed;
};

struct PIDsettings {
    // User‑adjustable parameters (from EEPROM or UI)
    uint8_t gainAdjustSteps = 0;
    uint8_t intTimeCstAdjustSteps = 0;
    uint8_t difTimeCstAdjustSteps = 0;

    // Derived PID parameters
    float gain = 0;
    float intTimeCst = 0;
    float difTimeCst = 0;

    float TTTintTimeCst = 0;
    float TTTdifTimeCst = 0;

    // True Three‑Term controller factors
    long TTTgain = 0;
    long TTTintFactor = 0;
    long TTTdifFactor = 0;

    long maxTTTallTerms = 0;
};

struct GlobeAttribute {
    bool attributeChangeMode{ false };
    long attributeIndex{ 0 };                                   // index of a particular selectedAttribute
    int attributeValue{ 0 };                                    // value of that same selectedAttribute
};

struct SmoothedMeasurements {
    int32_t tempSmooth{ 0 };
    float idleLoopMicrosSmooth{ 0 };
    float magnetOnCyclesSmooth{ 0 };
    float ISRdurationSmooth{ 0 };                               // values smoothed by first order filter
    float errSignalMagnitudeSmooth{ 0 };
};


struct StepResponseData {
    uint16_t count, hallReading_ADCsteps, TTTcontrOut;
};


// ========== class VisualRing: state machine creating a signal used to produce a 'ring' sequence  ==========
/*
The ring consists of 'm' identical sequences 's', separated by a pause.
A sequence consists of 'n' alternating states A and B (e.g., 3 states: A->B->A).
Each alternating state (A and B) has a common duration 'd', expressed in steps. The pause has a fixed duration 'p' (expressed in steps).

Use: control the ringing of a bell, the flashing of a led... based on the output signal produced by a VisualRing object.

Example: m = 2, n = 3, d = 4, p = 8. Sequential states:  AAAA BBBB AAAA pppppppp AAAA BBBB AAAA (end of ring)

A ring is started by calling startRing(...), specifying:
- the number of sequences (m). If m is 0, the ring will last until it is stopped by calling 'stopRing()'
- the number of alternating states in each sequence (n>0),
- the duration of each alternating state (d>0), counted in steps
- the duration of the pause (p>0), counted in steps (not relevant if only one sequence
- color scheme to use

This state machine counts *steps*, not time.
Each time advanceRingOneStep() is called, the state machine advances 1 step.
Example: if advanceRingOneStep() is called every 128 ms and d = 2 steps, then each state lasts 256 ms.
*/

class VisualRing {
public:
    enum RingState : uint8_t { ring_rest, ring_start, ring_state_A, ring_state_B, ring_pause };
    enum RingType : uint8_t { ring_off, ringing, alarm };

private:
    bool _changeStateNow{ false };
    RingState _ringState{ ring_rest };
    RingType _ringType{ ring_off };

    uint8_t _altColorChangesInSequence;         // number of color changes in 1 sequence (counted in steps)
    uint8_t _altColorSteps;                     // duration of an alternating color within a sequence (counted in steps)
    uint8_t _pauseSteps;                        // pause duration (counted in steps)

    uint8_t _altColorCounter;                   // remaining color changes in current sequence; counts down; b0: state A or B, b7: 'state changes NOW'
    uint8_t _pauseCounter;                      // remaining sequences in the ring; counts down
    uint8_t _stepCounter;                       // remaining steps to next color change (in steps); counts down

    uint8_t _colorScheme{ 0 };

public:
    // ========== stop a ring ==========
    bool inline stopRing() {
        if (_ringState == ring_rest) { return false; }          // not during an ongoing ring
        // immediately stop ring or alarm
        _ringState = ring_rest;                                 // will be stopped now in advanceRing()
        _ringType = ring_off;                                   // ring, alarm or off
        return true;
    }

    // ========== check the current ring state ==========
    VisualRing::RingState inline ringState(bool& changeNow, uint8_t& ringColorScheme) {
        // 'changeNow': state just changed: the calling program should take appropriate action BEFORE the next call to ringState()
        changeNow = _changeStateNow;
        ringColorScheme = _colorScheme;
        return _ringState;                                      // return 'ring state was changed'
    };

    VisualRing::RingType inline ringType() {
        return _ringType;                                       // return 'ring state was changed'
    };

    // public interface
    bool startRing(uint8_t sequences, uint8_t colorChangesInSequence, uint8_t altColorSteps, uint8_t pauseSteps, uint8_t ringAttribute);
    void advanceRingOneStep();
};


class GlobeTime {
// Note: timer 1 reading assumes clock speed is 16Mhz

private:
    uint32_t m_milliSecond, microSecond;
    unsigned int volatile m_nanoSecond500, m_nextNanoSecond500;
    volatile uint32_t milliSecond{ 0 }, second{ 0 };

public:

    uint32_t inline getMicros(uint32_t* secondsPtr = nullptr);

    uint32_t inline incrementMillis999() {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            milliSecond++;            // total
            if (milliSecond == oneSecondCount) { milliSecond = 0; second++; }
            return milliSecond;
        }
    }

    uint32_t inline getSecond() {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            return second;
        }
    }

    uint32_t inline getMillis(uint32_t* secondsPtr = nullptr) {             // return millis in current second & seconds as well (run time in micros = seconds * 1E6 + micros)
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            if (secondsPtr != nullptr) { *secondsPtr = second; }            // total
            return milliSecond;
        }
    }
};


class GlobeEvents {
private:
    static constexpr uint8_t eventBufferSize{ 127 };                        // max 255

    uint8_t* oldestMessageStartPtr{ nullptr }, * newestMessageStartPtr{ nullptr };
    EventData eventData;

public:
    // interface between ISR and main
    uint8_t eventBuffer[eventBufferSize];                                   // dynamic memory to store event messages until they are processed

    bool addChunk(uint8_t eventType, uint8_t newChunkSize, uint8_t** messagePtrPtr);
    bool removeOldestChunk(bool remove);
    bool isEventsWaiting();
    void takeSnapshot(EventData* eventSnapshotPtr);
};


#endif

