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
Wire Master State Machine - transport layer
===============================================================================

Current State	    Event	                            Action	                Next State
-------------       -----                               ------                  ----------
IDLE	            TX buffer non - empty	            Copy TX → out buffer	SEND
SEND	            MIN_CYCLE_TIME not elapsed	        Wait	                SEND
SEND	            I2C write error	                    Retry(≤ MAX_RETRIES)	SEND
SEND	            I2C write error, retries exceeded	Drop message	        IDLE
SEND	            Write OK, no reply expected	        Finalize	            IDLE                NOT APPLICBLE (always reply expected)
SEND	            Write OK, reply expected	        Start delay	            WAIT_BEFORE_POLL
WAIT_BEFORE_POLL	MIN_CYCLE_TIME elapsed	            Start polling	        WAIT_FOR_READY
WAIT_FOR_READY	    Poll interval not elapsed	        Wait	                WAIT_FOR_READY
WAIT_FOR_READY	    READY received	                    Proceed	                RECEIVE
WAIT_FOR_READY	    MAX_REPLY_TIME exceeded	            Error	                IDLE
RECEIVE	            MIN_CYCLE_TIME  not elapsed	        Wait	                RECEIVE
RECEIVE	            Reply read OK	                    Deliver reply	        IDLE
RECEIVE	            Reply read error	                Error	                IDLE

===============================================================================
Master-side design invariants
===============================================================================

Single in-flight wire transaction
- The master state machine enforces that only one message may occupy
  SEND / WAIT / RECEIVE states at a time.

TX queue is local only
- The master TX queue exists solely to absorb application jitter.
- Queued messages are sent strictly one at a time in FIFO order.

Message lifetime rule
- A queued TX message is removed only after:
    - a valid reply is received, or
    - retry limit is exceeded.

Retry consistency
- Retries always resend the same message bytes.
- Retry counters are reset only when advancing to the next message.

Time arithmetic correctness
- All time comparisons use unsigned wrap-safe subtraction.
- No logic depends on absolute timestamps.
- The master controls all timings.

Key invariant:
- The master leaves IDLE only when it owns the bus, and returns to IDLE only
  when the transaction is fully resolved.

===============================================================================
Polling / readiness invariants (if polling is used)
===============================================================================

READY is authoritative
- READY means the reply is fully prepared and immutable.
BUSY is non-destructive
- BUSY does not change slave state and does not discard data.
Polling does not advance protocol state
- Only the actual reply read completes a transaction.

===============================================================================
Wait times
===============================================================================

Wait times are non-blocking with one exception: timeout while reading
incoming wire data in method 'i2cReadMessage()'.

===============================================================================
*/

#ifndef FG_MASTER_TRANSPORT.h
#define FG_MASTER_TRANSPORT.h

#include "arduino.h"
#include <util/atomic.h>                                            // atomic operations
#include "shared/wire_protocol.h"

class  WireMaster {

/* ================= CONSTANTS ================= */

private:
    static constexpr  uint8_t I2C_SLAVE_ADDR = 9;
    static constexpr uint32_t I2C_CLOCK = 100000UL;                 // 100 or 400 kHz

    // Ring queue
    static constexpr uint8_t TX_QUEUE_SIZE = 4;                     // queue depths     
    static constexpr uint8_t RX_QUEUE_SIZE = 4;

    // Timing/backoff
    static constexpr unsigned long SLAVE_POLL_INTERVAL_micros = 2000;
    static constexpr unsigned long MIN_CYCLE_PERIOD_MICROS = 10000; // minimum spacing between send or receive operations

    static constexpr unsigned long MAX_RECEIVE_CYCLE_MILLIS = 100;  // maximum time waiting for a slave reply (from send to receive)

    // Retries
    static constexpr uint8_t MAX_TRIES_PER_PACKET = 3;              // max. retries allowed per message (send)


/* ================= TX: RING BUFFER, RX: RING BUFFER ================= */

private:
    volatile uint8_t txQueue[TX_QUEUE_SIZE][HEADER_SIZE + MASTER_PAYLOAD_MAX + 1];
    volatile uint8_t txExpReplyMsgType[TX_QUEUE_SIZE];              // paired with txQueue slot index
    volatile uint8_t txExpReplyMsgSize[TX_QUEUE_SIZE];              // paired with txQueue slot index

    volatile uint8_t rxQueue[RX_QUEUE_SIZE][HEADER_SIZE + SLAVE_PAYLOAD_MAX + 1];  // message type + payload size + payload + checksum
    volatile uint8_t rxExpReplyMsgType[TX_QUEUE_SIZE];              // paired with txQueue slot index  

    // producer owned (SPSC)
    volatile uint8_t rxHead = 0;                                    // next free slot (main or ISR will write here)
    volatile uint8_t txHead = 0;                                    // next free slot (main or ISR will write here)

    // consumer owned (SPSC)
    volatile uint8_t txTail = 0;                                    // oldest queued (main reads here)
    volatile uint8_t rxTail = 0;                                    // oldest queued (main reads here)


    /* ================= STATE + WORKING COPIES ================= */

    enum MasterState :uint8_t { MS_IDLE, MS_SEND, MS_WAIT_BEFORE_POLLING, MS_WAIT_FOR_SLAVE_READY, MS_RECEIVE };
    MasterState state = MasterState::MS_IDLE;

    uint8_t rxInBuffer[HEADER_SIZE + SLAVE_PAYLOAD_MAX + 1];
    uint8_t txOutBuffer[HEADER_SIZE + MASTER_PAYLOAD_MAX + 1];


public:
    // state returned to the calling program
    enum class WireStatus : uint8_t {
        I_idle,
        I_waitForCue,                                               // counting time until next cue
        I_xmitOK,                                                   // sent/receive went OK
        E_tx_wireXmitError,
        E_rx_checksum,
        E_rx_timeOut,
        E_rx_full
    };

private:
    volatile bool& _triggerWireCommLed;

    I2C_m_masterSendStats masterSendStats;
    I2C_m_masterReceiveStats masterReceiveStats;
public:


    /* ========== METHODS ========== */

    WireMaster(volatile bool& triggerWireCommLed);
    ~WireMaster();

    // safe to call from ISR
    // enqueue with expReplyPayloadSize = 0xff: no return message requested, = 0x00: requested, without payload
    bool enqueueTx(uint8_t msgType, uint8_t payloadSize, const void* payload, uint8_t expReplyMsgType, uint8_t expReplyPayloadSize);
    bool dequeueRx(uint8_t& msgType, uint8_t& payloadSize, void* payload, uint8_t& expReplyMsgType);

    // should be called frequently from application main loop
    WireStatus sendAndReceiveMessage();

    void getSendStats(I2C_m_masterSendStats& sendStatSnapshot);
    void getReceiveStats(I2C_m_masterReceiveStats& receiveStatSnapshot);

private:
    // helpers: called from sendAndReceiveMessage()
    bool copyTXqueueTailToOut(uint8_t* const out, uint8_t& expReplyMsgType, uint8_t& expReplyMsgSize);
    bool i2cWriteMessage(const uint8_t* p);
    bool i2cReadMessage(uint8_t* p, uint8_t expReplyMsgType, uint8_t expReplyMsgSize);
    WireMaster::WireStatus copyInToRXqueueHead(uint8_t* const in, uint8_t expReplyMsgType, uint8_t expReplyMsgSize);
};

#endif

