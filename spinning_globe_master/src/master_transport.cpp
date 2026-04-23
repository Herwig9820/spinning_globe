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
    send and receive SPSC queues
===============================================================================

Send and receive queues are implemented as SPSC (single producer single consumer)
ring buffers:
the same queue element is never read and written concurrently => NO LOCKS
(no atomic blocks) while ISR safe
-> txHead, rxHead = indexes for WRITING to tx, rx queues: ONLY WRITTEN TO DURING ENQUEUING
-> txTail, rxTail = indexes for READING to tx, rx queues: ONLY READ FROM DURING ENQUEUING
-> if head and tail are equal, the queue is either full (write attempt will fail)
or empty (nothing to read)

Memory fences:
- writing the queue: DATA must be visible before updating the head index :
  use a memory fence to force ordered memory operations
- reading the queue: tail index must be visible before reading the DATA  : idem

Single-writer operations (e.g. on counters) do not need protection -
corresponding read operations that can be interrupted by a write operation
must be protected however.

Note on AVR MCU's
-----------------
- the Wire master library does not call any user functions (no callbacks) during
  the execution of Wire library ISRs (the Wire slave library does).
- AVR memory operations are strongly ordered: no hardware fences needed, but added
  to make code generic.

===============================================================================
*/

#include <Wire.h>
#include "master_transport.h"

#define release_barrier() asm volatile ("" ::: "memory")
#define acquire_barrier() asm volatile ("" ::: "memory")


WireMaster::WireMaster(volatile bool& triggerWireCommLed) : _triggerWireCommLed(triggerWireCommLed) {
    Wire.begin(); // join I2C bus (address optional for master)
    Wire.setClock(I2C_CLOCK);
}

WireMaster::~WireMaster() {
}


/*
===============================================================================
RING BUFFER IMPLEMENTATION
===============================================================================
*/

//  ========== enqueueTx: safe from ISR. Returns true if enqueued, false if queue full ==========

bool WireMaster::enqueueTx(uint8_t msgType, uint8_t payloadSize, const void* payload, uint8_t expReplyMsgType, uint8_t expReplyPayloadSize) {
    
    static uint8_t msgSeqNum{0};

#if TRACK_FREE_MEM
    trackFree();
#endif

    uint8_t next = (txHead + 1) % TX_QUEUE_SIZE;
    if (next == txTail) {                                           // queue full ?
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { masterSendStats.E_stats_tx_full++; }
        return false;
    }

    // seq num must be added here and stored in txQueue because it's included in the checksum which is calculated here
    txQueue[txHead][WireFrame::OFFSET_MSG_TYPE] = msgType;
    txQueue[txHead][WireFrame::OFFSET_PAYLOAD_SIZE] = payloadSize;
    txQueue[txHead][WireFrame::OFFSET_SEQ_NUM] = msgSeqNum;
    txQueue[txHead][WireFrame::OFFSET_RESERVED] = RESERVED_DEFAULT;
    
    uint8_t sum = msgType ^ payloadSize ^ msgSeqNum ^ RESERVED_DEFAULT;
    for (uint8_t i = 0; i < payloadSize; ++i) {
        txQueue[txHead][HEADER_SIZE + i] = ((uint8_t*)payload)[i];
        sum ^= ((uint8_t*)payload)[i];
    }
    msgSeqNum++;

    txQueue[txHead][HEADER_SIZE + payloadSize] = sum;
    // remember REPLY MESSAGE SIZE, in queue: THIS will decide if data is requested from a slave. 0xff if no reply expected
    txExpReplyMsgType[txHead] = expReplyMsgType;
    txExpReplyMsgSize[txHead] = (expReplyPayloadSize == 0xff) ? 0xff : HEADER_SIZE + expReplyPayloadSize + 1;

    // AVR: strongly ordered, no hardware fences needed BUT memory fence added to keep code generic.
    release_barrier();                                              // ensure data is visible before updating head

    txHead = next;
    return true;
};


// ========== dequeueRx: safe from ISR. Returns true if enqueued, false if queue empty ==========

bool WireMaster::dequeueRx(uint8_t& msgType, uint8_t& payloadSize, void* payload) {
#if TRACK_FREE_MEM
    trackFree();
#endif


    // 8-bit index: atomic access by nature
    if (rxHead == rxTail) { return false; }                         // rx queue empty ?

    // AVR: strongly ordered, no hardware fences needed BUT memory fence added to keep code generic.
    acquire_barrier();                                              // ensure we see latest tail before reading data

    msgType = rxQueue[rxTail][WireFrame::OFFSET_MSG_TYPE];
    payloadSize = rxQueue[rxTail][WireFrame::OFFSET_PAYLOAD_SIZE];
    if (payloadSize > SLAVE_PAYLOAD_MAX) payloadSize = 0;
    for (uint8_t i = 0; i < payloadSize; ++i) {
        ((uint8_t*)payload)[i] = rxQueue[rxTail][HEADER_SIZE + i];
    }

    rxTail = (rxTail + 1) % RX_QUEUE_SIZE;
    return true;
};


// ========== PROCESS: SEND TX QUEUES AND RECEIVE RX QUEUES ==========

// must be frequently called from within main loop 

WireMaster::WireStatus WireMaster::sendAndReceiveMessage() {
#if TRACK_FREE_MEM
    trackFree();
#endif

    static uint8_t expReplyMsgType, expReplyMsgSize{}, expReplySeqNumber{};

    static unsigned long lastTaskTime_micros = 0;                   // delay scheduling
    static unsigned long lastPollTime_micros = 0;

    static uint8_t sendRetryCount = 0;

    unsigned long now_micros = micros();
    unsigned long timePassed{};

    /* ---------- state machine: MS_IDLE: enqueue message if available ---------- */

    if (state == MasterState::MS_IDLE) {                            // not sending or receiving ?
        // If RX queue is full, we can't accept a reply, so don't send
        uint8_t next = (rxHead + 1) % RX_QUEUE_SIZE;
        if (next == rxTail) { return WireStatus::I_rx_backPressure;}   // wait for rx consumer
        
        // message available in tx queue ? Copy to out buffer
        if (!copyTXqueueTailToOut(txOutBuffer, expReplyMsgType, expReplyMsgSize, expReplySeqNumber)) { return WireStatus::I_idle; }  
        sendRetryCount = 0;
        state = MasterState::MS_SEND;                                           // change status to MS_SEND 
    }

    /* ---------- state machine: MS_SEND: send message to slave (respect inter-message spacing) ---------- */

    if (state == MasterState::MS_SEND) {
        // Respect scheduling (backoff time)

        // check inter-message spacing (non-blocking)
        timePassed = (now_micros - lastTaskTime_micros);                        // unsigned integer subtraction: modulo 2^32
        if (timePassed < MIN_CYCLE_PERIOD_MICROS) { return WireStatus::I_waitForCue; }
        
        // transmit; in case of error, retry a few times (non-blocking) 
        bool ok = i2cWriteMessage(txOutBuffer);                                 // Wire write
        lastTaskTime_micros = now_micros;
        if (!ok) {                                                              // Wire library transmit error 
            // manage retries: if exceeded MAX_RETRIES, drop the packet (advance tail)
            if (++sendRetryCount < MAX_TRIES_PER_PACKET) {                      // max. retries was reached before ?
                // stay in SEND state: max. tries not yet attempted
                ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { masterSendStats.W_stats_tx_retrying++; } // warning only: count retries
                return WireStatus::E_tx_wireXmitError;
            }

            // write retries exhausted: advance txTail (pop the packet), increment error counter and go back to idle
            state = MasterState::MS_IDLE;
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                txTail = (txTail + 1) % TX_QUEUE_SIZE;                          // advance tx tail (even with 8-bit length, operation  is not atomic)
                masterSendStats.E_stats_tx_wireXmitError++;                     // after max retries: error
            }
            return WireStatus::E_tx_wireXmitError;
        }


        // success: advance txTail (pop the packet), increment counter and move state to 'wait before polling'
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            txTail = (txTail + 1) % TX_QUEUE_SIZE;                              // advance tx tail (even with 8-bit length, operation  is not atomic)
            masterSendStats.I_stats_tx_sent++;
        }

        state = MasterState::MS_WAIT_BEFORE_POLLING;
    }


    /* ---------- state machine: MS_WAIT_BEFORE_POLLING ---------- */

    // wait a fixed time (the minimum cycle time) before starting polling
    if (state == MasterState::MS_WAIT_BEFORE_POLLING) {
        timePassed = (now_micros - lastTaskTime_micros);                        // unsigned integer subtraction: modulo 2^32
        if (timePassed < MIN_CYCLE_PERIOD_MICROS) { return WireStatus::I_waitForCue; }  // Respect scheduling (holdoff)
        
        lastTaskTime_micros = lastPollTime_micros = now_micros;                 // init
        while (Wire.available()) { (void)Wire.read(); }                         // drain any stale bytes
        state = MasterState::MS_WAIT_FOR_SLAVE_READY;
    }



    /* ---------- state machine: poll until slave ready or timeout (non-blocking) ---------- */

    if (state == MasterState::MS_WAIT_FOR_SLAVE_READY) {
        timePassed = (now_micros - lastTaskTime_micros);                        // unsigned integer subtraction: modulo 2^32
        if (timePassed > MAX_WAIT_FOR_SLAVE_READY_MICROS) {
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { masterReceiveStats.E_stats_rx_timeOut++; }
            state = MasterState::MS_IDLE;
            return WireStatus::E_rx_timeOut;                                    // reply did not arrive in time
        }

        // poll every (SLAVE_POLL_INTERVAL_micros)
        timePassed = now_micros - lastPollTime_micros;         // unsigned integer subtraction: modulo 2^32
        if (timePassed < SLAVE_POLL_INTERVAL_micros) { return WireStatus::I_waitForCue; }
        lastPollTime_micros = now_micros;

        // start polling
        constexpr uint8_t request = (uint8_t)WireTransport::M_CTRL_POLL;
        uint8_t reply{ (uint8_t)WireTransport::S_CTRL_BUSY };                   // init, will be unchanged if write error 
        Wire.beginTransmission(I2C_SLAVE_ADDR);
        Wire.write(&request, 1);
        uint8_t err = Wire.endTransmission();                                   // 0 == success
        if (err == 0) {
            if (i2cReadMessage(&reply, 1)) {
                // Any poll result other than READY is treated as BUSY by design
                if (reply != (uint8_t)WireTransport::S_CTRL_READY) {            // an erroneous reply byte is captured here (although nothing is done with it) 
                    reply = (uint8_t)WireTransport::S_CTRL_BUSY;
                }
            }
            else { reply = (uint8_t)WireTransport::S_CTRL_BUSY; }               // reading busy/ready reply timeout error (although nothing is done with it)
        }

        if (reply == (uint8_t)WireTransport::S_CTRL_READY) {
            state = MasterState::MS_RECEIVE;
        }
    }


    /* ---------- state machine: receive slave message  ---------- */

    // RECEIVE state: attempt read if expected by protocol
    // NOTE: one message out results in one message in (1:1)

    if (state == MasterState::MS_RECEIVE) {
        timePassed = now_micros - lastTaskTime_micros;                          // unsigned integer subtraction: modulo 2^32
        if (timePassed < MIN_CYCLE_PERIOD_MICROS) { return WireStatus::I_waitForCue; }  // Respect scheduling (holdoff)
        
        bool ok = i2cReadMessage(rxInBuffer, expReplyMsgSize);                  // size needed to read correct number of bytes from buffer
        lastTaskTime_micros = now_micros;                                       // init
        if (!ok) {
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { masterReceiveStats.E_stats_rx_timeOut++; } // do NOT re-enqueue; we drop/ignore reply
            state = MasterState::MS_POST_RX_ERROR_HOLDOFF;
            return WireStatus::E_rx_timeOut;
        }

        WireStatus wireStatus = copyInToRXqueueHead(rxInBuffer, expReplyMsgType, expReplyMsgSize, expReplySeqNumber);
        _triggerWireCommLed = true;                                             // handled in timer interrupt
        state = ((wireStatus == WireStatus::I_xmitOK) ||(wireStatus == WireStatus::E_rx_full)) ? MasterState::MS_IDLE : MasterState::MS_POST_RX_ERROR_HOLDOFF;
        return wireStatus;
    }


    /* ---------- state machine: MS_POST_RX_ERROR_HOLDOFF: extra hold off time after rx error ---------- */

    if (state == MasterState::MS_POST_RX_ERROR_HOLDOFF) {
        timePassed = now_micros - lastTaskTime_micros;
        if (timePassed < POST_ERROR_HOLDOFF_MICROS) { return WireStatus::I_waitForCue; }
        state = MasterState::MS_IDLE;
        return WireStatus::I_idle;                                                  // safety (control doesn't pass here)
    }

    return WireStatus::I_idle;                                                      // safety (control doesn't pass here)
}


// ========== GET master send and receive stats ==========

void WireMaster::getAndClearSendStats(I2C_m_masterSendStats& sendStatSnapshot) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // constant cast needed because it removes 'volatile' implicitly (can not assign volatile to non-volatile)
        sendStatSnapshot = const_cast<const I2C_m_masterSendStats&>(masterSendStats);
        masterSendStats.zeroMembers();
    }
}

void WireMaster::getAndClearReceiveStats(I2C_m_masterReceiveStats& receiveStatSnapshot) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // constant cast needed because it removes 'volatile' implicitly (can not assign volatile to non-volatile)
        receiveStatSnapshot = const_cast<const I2C_m_masterReceiveStats&>(masterReceiveStats);
        masterReceiveStats.zeroMembers();
    }
}


// ================= HELPERS: SEND TX QUEUE TAIL  =================

bool WireMaster::copyTXqueueTailToOut(uint8_t* const out, uint8_t& expReplyMsgType, uint8_t& expReplyMsgSize, uint8_t&expReplySeqNum) {
#if TRACK_FREE_MEM
    trackFree();
#endif

    uint8_t head, tail;
    head = txHead;
    tail = txTail;

    if (head == tail) { return false; }                             // empty

    // AVR: strongly ordered, no hardware fences needed BUT memory fence added to keep code generic.
    acquire_barrier();                                              // ensure we see latest tail before reading data

    uint8_t msgLength = HEADER_SIZE + txQueue[tail][WireFrame::OFFSET_PAYLOAD_SIZE] + 1;         // message ID, payload length, checksum
    memcpy(out, (uint8_t*)txQueue[tail], msgLength);                // copy struct
    expReplyMsgType = txExpReplyMsgType[tail];
    expReplyMsgSize = txExpReplyMsgSize[tail];                      // copy as well (0xff: no reply expected)
    expReplySeqNum = txQueue[tail][WireFrame::OFFSET_SEQ_NUM];
    return true;
}


bool WireMaster::i2cWriteMessage(const uint8_t* out) {
#if TRACK_FREE_MEM
    trackFree();
#endif

    uint8_t msgLength = HEADER_SIZE + out[WireFrame::OFFSET_PAYLOAD_SIZE] + 1;                   // message ID, payload length, checksum
    bool ok{ false };
    Wire.beginTransmission(I2C_SLAVE_ADDR);
    Wire.write(out, msgLength);
    uint8_t err = Wire.endTransmission();                           // 0 == success
    ok = (err == 0);

    return ok;
}


// ========== HELPERS: RECEIVE RX QUEUE HEAD ===========

bool WireMaster::i2cReadMessage(uint8_t* in, uint8_t expReplyMsgSize) {
#if TRACK_FREE_MEM
    trackFree();
#endif

    // if the slave sends less  bytes than expected, the Wire master will pad with 0xFF bytes. Extra bytes sent are simply discarded 

    // calculation of blocking time out value: at 100 kHz, sending one byte takes approx. 100 microseconds
    // per byte 50% extra time is added to the (blocking !) time out value. Example: 10 byte msg => time out = 250 + 10 * 50 = 750 micros
    const uint32_t timeoutValue = 250 + 50 * expReplyMsgSize;       // microseconds; allowed timeout depends on message size  
    Wire.requestFrom(I2C_SLAVE_ADDR, (uint8_t)expReplyMsgSize);

    unsigned long start_micros = micros();
    uint8_t idx = 0;
    bool ok{ false };

    // while loop: must execute fast to free internal Wire buffer
    while (idx < expReplyMsgSize) {
        if (Wire.available()) {
            in[idx++] = Wire.read();
        }
        else {
            if ((micros() - start_micros) > timeoutValue) { break; } // blocking operation: timeout (message only partially received)
        }
    }

    ok = (idx == expReplyMsgSize);

    return ok;
}


WireMaster::WireStatus WireMaster::copyInToRXqueueHead(uint8_t* const in, uint8_t expReplyMsgType, uint8_t expReplyMsgSize, uint8_t expReplySeqNumber) {
#if TRACK_FREE_MEM
    trackFree();
#endif

// compute next head index in standard SPSC ring-buffer
    uint8_t head = rxHead;
    uint8_t next = (head + 1) % RX_QUEUE_SIZE;

    if (next == rxTail) {    // Buffer full -> drop packet
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { masterReceiveStats.E_stats_rx_full++; }    // message dropped (32-bit stats_ variable increment must be atomic
        return WireStatus::E_rx_full;
    }

    uint8_t sum = 0;
    for (uint8_t i = 0; i < expReplyMsgSize - 1; ++i) {                         // includes header and payload, excludes received checksum from checksum calculation
        rxQueue[head][i] = in[i];
        sum ^= in[i];
    }
    rxQueue[head][expReplyMsgSize - 1] = in[expReplyMsgSize - 1];               // checksum 

    // check checksum, message type and sequence number
    if (sum != rxQueue[head][expReplyMsgSize - 1]) {                            // checksum correct ?
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { masterReceiveStats.E_stats_rx_checksum++; }// message dropped (32-bit stats_ variable increment must be atomic
        return WireStatus::E_rx_checksum;
    };

    if (in[WireFrame::OFFSET_MSG_TYPE] != expReplyMsgType) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { masterReceiveStats.E_stats_rx_typeMismatch++; }
        return WireStatus::E_rx_typeMismatch;
    }

    if (in[WireFrame::OFFSET_SEQ_NUM] != expReplySeqNumber) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { masterReceiveStats.E_stats_rx_seqMismatch++; }
        return WireStatus::E_rx_seqMismatch;  // new enum value
    }   

    // AVR: strongly ordered, no hardware fences needed BUT memory fence added to keep code generic.
    release_barrier();                                                          // ensure data is visible before updating head

    rxHead = next;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { masterReceiveStats.I_stats_rx_received++; }

    return WireStatus::I_xmitOK;
}




