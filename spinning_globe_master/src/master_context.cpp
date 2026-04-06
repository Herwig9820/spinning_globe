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

#include <util/atomic.h>                                    
#include "master_context.h"
#include "shared/wire_hw_config.h"

/*
============================================================================
Class: GlobeEvents
============================================================================
*/


// ========== reserve space in the event message buffer for a new event message ==========

bool GlobeEvents::addChunk(uint8_t eventType, uint8_t newChunkSize, uint8_t** messagePtrPtr) {      // prevent interference with another call to addChunk() - which may be called from ISR;
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

    #if TRACK_FREE_MEM
        trackFree();
    #endif

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

            eventData.activeMsgPtr = oldestMessageStartPtr + 4;
            eventData.activeEventType = *oldestMessageStartPtr;                                     // oldest event: event type
            eventData.eventsPending++;
            // update statistics
            eventData.largestEventsPending = max(eventData.largestEventsPending, eventData.eventsPending);
            eventData.eventBufferBytesUsed += newChunkSize;
            eventData.largestEventBufferBytesUsed = max(eventData.largestEventBufferBytesUsed, eventData.eventBufferBytesUsed);

        #if TEST_SHOW_STATS 
            Serial.println(); Serial.print(F("+ ;"));  Serial.println((int)newestMessageStartPtr);
        #endif
        }

        else {
            eventData.eventsMissed++;
        #if TEST_SHOW_STATS 
            Serial.println(); Serial.println(F("missed"));
        #endif
        }
    }
    return OK;
}


// ========== release space occupied by the oldest message in the event message buffer ==========

bool GlobeEvents::removeOldestChunk(bool remove) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {                                                                                             // prevent interference with addChunk() - which may be called from ISR;
        if ((oldestMessageStartPtr == nullptr) || (!remove)) { return false; }                                                      // nothing to remove: is empty

        eventData.eventsPending--;
        eventData.eventBufferBytesUsed -= *(oldestMessageStartPtr + 1);                                                             // before pointer update

    #if TEST_SHOW_STATS 
        Serial.println(); Serial.print(F("- ;"));  Serial.println((int)oldestMessageStartPtr);
    #endif

        if (oldestMessageStartPtr == newestMessageStartPtr) { oldestMessageStartPtr = nullptr; newestMessageStartPtr = nullptr; }   // remove last remaining
        else { oldestMessageStartPtr = *(uint8_t**)(oldestMessageStartPtr + 2); }                                                   // remove oldest (which is not the last remaining)

        eventData.activeMsgPtr = (oldestMessageStartPtr == nullptr) ? nullptr : oldestMessageStartPtr + 4;                          // pointer to optional structure; after pointer update
        eventData.activeEventType = (oldestMessageStartPtr == nullptr) ? eNoEvent : *oldestMessageStartPtr;

    }
    return true;
}


// ========== check whether any globe events are waiting to be processed ==========

bool GlobeEvents::isEventsWaiting() {
    bool eventsArePending{ false };
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {                         // prevent interference with addChunk() - which may be called from ISR;
        eventsArePending = (eventData.eventsPending > 0);       // (8-bit variable: operation is already atomic)
    }
    return eventsArePending;
}


// ========== take a snapshot of the current globe event status ==========

void GlobeEvents::takeSnapshot(EventData* eventSnapshotPtr) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {                         // prevent interference with addChunk() - which may be called from ISR;
        *eventSnapshotPtr = eventData;
    }
}

/*
============================================================================
Class: getMillis and getMicros function based on timer1 and own time counting logic
(millis and micros < one second, count seconds) * **
============================================================================
*/



// execution time is very close to 20 microSeconds (16MHz clock)
uint32_t GlobeTime::getMicros(uint32_t* secondsPtr = nullptr) {         // return micros in current second & seconds as well (run time in micros = seconds * 1E6 + micros) 
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
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
    }

    // calculate micro seconds in current second: 0 to 999999 (1 period = 500 nS) 
    microSecond = ((uint32_t)(m_nanoSecond500 >> 1)) + (m_milliSecond << 10) - (m_milliSecond << 4) - (m_milliSecond << 3);
    if (microSecond >= 1000000L) {
        microSecond = microSecond - 1000000L;
        if (secondsPtr != nullptr) { (*secondsPtr)++; }
    }
    return microSecond;
}


 // ========== VisualRing: small state machine to create a timed 'ring' ==========

// ========== start a ring ==========
bool VisualRing::startRing(uint8_t sequences, uint8_t colorChangesInSequence, uint8_t altColorSteps, uint8_t pauseSteps) {

    // in rest, sequence and state counters are zero
    if (_ringState != ring_rest) { return false; }          // not during an ongoing ring
    _ringState = ring_start;                                // init ring

    // store color change count in a sequence and durations (in steps) of a color in a sequence and of a pause between sequences
    _altColorChangesInSequence = colorChangesInSequence;
    _altColorSteps = altColorSteps;                         // number of steps between two state changes within a ring sequence
    _pauseSteps = pauseSteps;                               // number of steps in a pause

    // init counters for pauses, alternating colors in a sequence, steps within a color   
    _altColorCounter = colorChangesInSequence;              // number of (alternating) colors within ring sequence
    _pauseCounter = sequences - 1;                          // number of ring sequences - 1 (no pause after last ring)
    _stepCounter = _altColorSteps;                          // start with an alternating color
    return true;
};


// ========== advance one step in the ring sequence ==========
void VisualRing::advanceRingOneStep() {
#if TRACK_FREE_MEM
    trackFree();
#endif

    _changeStateNow = false;           

    // a small state machine to control ring color timing
    switch (_ringState) {
        case ring_rest:
        {
            return;                                         // nothing to do
        }
        break;

        case ring_start:
        {
            _ringState = ring_state_A; 
            _changeStateNow = true;
        }
        break;

        case ring_state_A:
        case ring_state_B:
        {
            if (--_stepCounter == 0) {                      // last step within an alternating color (red or white) 
                if (--_altColorCounter == 0) {              // this was the last color (red or white) in a sequence: go to pause or end ?      
                    _stepCounter = _pauseSteps;             // in case a pause still follows
                    _ringState = ((_pauseCounter == 0) ? ring_rest : ring_pause);
                    _changeStateNow = true;
                }

                else {
                    _stepCounter = _altColorSteps;          // reload step counter
                    _ringState = ((_ringState == ring_state_A) ? ring_state_B : ring_state_A);
                    _changeStateNow = true;
                }
            }
        }
        break;

        case ring_pause:
        {
            if (--_stepCounter == 0) {                      // this was the last step within a pause: a next sequence will follow
                _altColorCounter = _altColorChangesInSequence;
                _pauseCounter--;
                _stepCounter = _altColorSteps;
                _ringState = ring_state_A;
                _changeStateNow = true;
            }
        }
        break;
    }

    Serial.print(_ringState, HEX); Serial.print(' ');

    if (_changeStateNow) {
        Serial.print(F("\r\nadvance to new state: ")); Serial.print(_ringState); Serial.print(F(", time ")); Serial.println(millis());
    }
};

// ========== check the current ring state ==========
VisualRing::RingState VisualRing::ringState(bool &changeNow) {
    changeNow = _changeStateNow;
    return _ringState;                                                    // return 'ring state was changed'
};

