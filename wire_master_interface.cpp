#include <Wire.h>
#include "wire_master_interface.h"

/*
    --- SPSC queues ---
    send and receive queues are implemented as SPSC (single producer single consumer) ring buffers:
    the same queue element is never read and written concurrently => NO LOCKS (no atomic blocks) while ISR safe
    -> txHead, rxHead = indexes for WRITING to tx, rx queues: ONLY WRITTEN TO DURING ENQUEUING
    -> txTail, rxTail = indexes for READING to tx, rx queues: ONLY READ FROM DURING ENQUEUING
    -> if head and tail are equal, the queue is either full (write attempt will fail) or empty (nothing to read)

    --- notes ---
    1. volatile variables
    make ALL variables shared by MAIN and ISR volatile (force compiler never to rely on stored register values but to always access memory)
    2. atomicity of operations
    if indexes are 8-bit: index reads and writes are atomic: NO atomic blocks required while reading / writing indexes
    operations with 16 or 32-bit variables (error counters etc.) MUST BE ATOMIC if shared between main and ISR
    3. memory fences
    writing the queue: DATA must be visible before updating the head index : use a memory fence to force ordered memory operations
    reading the queue: tail index must be visible before reading the DATA  : idem

    AVR MCU's
    ---------
    - the Wire master library does not call any user functions (no callbacks) during the execution of Wire library ISRs (the Wire slave library does).
    - AVR memory operations are strongly ordered: no hardware fences needed, but added to make code generic
    */

#define release_barrier() asm volatile ("" ::: "memory")
#define acquire_barrier() asm volatile ("" ::: "memory")


Wire_master_interface::Wire_master_interface() {
    Wire.begin(); // join I2C bus (address optional for master)
    Wire.setClock(I2C_CLOCK);
}

Wire_master_interface::~Wire_master_interface() {
}


// ================= RING BUFFER IMPLEMENTATION ================= 

//  enqueueTx: safe from ISR. Returns true if enqueued, false if queue full.

bool Wire_master_interface::enqueueTx(uint8_t type, uint8_t payloadSize, const void* payload, uint8_t expReplyPayloadSize) {
    uint8_t next = (txHead + 1) % TX_QUEUE_SIZE;
    if (next == txTail) {                                           // queue full ?
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { sendStats.E_stats_tx_full++; }    // message dropped (32-bit stats_ variable increment must be atomic
        return false;
    }

    txQueue[txHead][0] = type;
    txQueue[txHead][1] = payloadSize;
    uint8_t sum = type ^ payloadSize;
    for (uint8_t i = 0; i < payloadSize; ++i) {
        txQueue[txHead][HEADER_SIZE + i] = ((uint8_t*)payload)[i];
        sum ^= ((uint8_t*)payload)[i];
    }

    txQueue[txHead][HEADER_SIZE + payloadSize] = sum;
    // remember REPLY MESSAGE SIZE, in queue: THIS will decide if data is requested from a slave. 0xff if no reply expected
    txExpReplyMsgSize[txHead] = (expReplyPayloadSize == 0xff) ? 0xff : HEADER_SIZE + expReplyPayloadSize + 1;

    // AVR: strongly ordered, no hardware fences needed BUT memory fence added to keep code generic.
    release_barrier();                                              // ensure data is visible before updating head

    txHead = next;
    return true;
};


//  dequeueRx: safe from ISR. Returns true if enqueued, false if queue empty.

bool Wire_master_interface::dequeueRx(uint8_t& type, uint8_t& payloadSize, void* payload) {
    // 8-bit index: atomic access by nature
    if (rxHead == rxTail) { return false; }                         // rx queue empty ?

    // AVR: strongly ordered, no hardware fences needed BUT memory fence added to keep code generic.
    acquire_barrier();                                              // ensure we see latest tail before reading data

    type = rxQueue[rxTail][0];
    payloadSize = rxQueue[rxTail][1];
    if (payloadSize > PAYLOAD_IN_MAX) payloadSize = 0;
    for (uint8_t i = 0; i < payloadSize; ++i) {
        ((uint8_t*)payload)[i] = rxQueue[rxTail][HEADER_SIZE + i];
    }

    rxTail = (rxTail + 1) % RX_QUEUE_SIZE;
    return true;

};


// ========== PROCESS: SEND TX QUEUES AND RECEIVE RX QUEUES ==========

// must be frequently called from within main loop 

Wire_master_interface::WireStatus Wire_master_interface::sendAndReceiveMessage(uint8_t* pmsgType) {
    static uint8_t expReplyMsgSize{};

    unsigned long currentTime = millis();

    *pmsgType = 0;                                                   // init

    if (state == MS_IDLE) {                                         // not sending or receiving ?
        if (!copyTXqueueTailToOut(txOutBuffer, expReplyMsgSize)) { return WireStatus::I_idle; }  // message available ? Copy to out buffer
        sendMaxTries = 0;
        *pmsgType = txOutBuffer[0];                                  // useful in case of error: last message (in or out) handled
        state = MS_SEND;
    }


    if (state == MS_SEND) {
        // Respect scheduling (backoff time)
        if (lastCycleTime + CYCLE_PERIOD_MS > currentTime) { return WireStatus::I_waitForCue; }
        lastCycleTime = currentTime;                                // init: assume transmission will be ok

        // check inter-packet spacing
        bool ok = i2cWriteMessage(txOutBuffer);                     // Wire write
        if (!ok) {                                                  // Wire library transmit error 
            // manage retries: if exceeded MAX_RETRIES, drop the packet (advance tail)
            if (++sendMaxTries < MAX_RETRIES_PER_PACKET) {          // max. retries was reached before ?
                // stay in SEND state: max. tries not yet attempted
                ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { sendStats.W_stats_tx_retrying++; } // warning only: count retries
                return WireStatus::E_tx_WireXmitError;
            }

            txTail = (txTail + 1) % TX_QUEUE_SIZE;              // advance tail
            state = MS_IDLE;                                    // go back to IDLE
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { sendStats.E_stats_tx_wireXmitError++; }
            return WireStatus::E_tx_WireXmitError;
        }


        // success: advance send counters and pop the packet (advance tail atomically)
        txTail = (txTail + 1) % TX_QUEUE_SIZE;
        // after successful write, allow optional receive based on message type
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { sendStats.I_stats_sent++; }

        state = (expReplyMsgSize == 0xff) ? MS_IDLE : MS_RECEIVE;
        if (state == MS_IDLE) { return WireStatus::I_xmitOK; }
    }


    // RECEIVE state: attempt read if expected by protocol
    // NOTE: one message out results in one message in (1:1)

    if (state == MS_RECEIVE) {
        if (lastCycleTime + CYCLE_PERIOD_MS > currentTime) { return WireStatus::I_waitForCue; }  // Respect scheduling (holdoff)
        lastCycleTime = currentTime;                                    // init

        bool ok = i2cReadMessage(rxInBuffer, expReplyMsgSize);
        *pmsgType = rxInBuffer[0];                                       // useful in case of error: last message (in or out) handled
        if (!ok) {
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { receiveStats.E_stats_rx_timeOut++; } // do NOT re-enqueue; we drop/ignore reply
            state = MS_IDLE;
            return WireStatus::E_rx_timeOut;
        }

        WireStatus wireStatus = copyInToRXqueueHead(rxInBuffer, expReplyMsgSize);
        state = MS_IDLE;
        return wireStatus;
    }

    return WireStatus::I_idle;                                          // safety (control doesn't pass here)
}


// ========== PROCESS: SEND TX QUEUES AND RECEIVE RX QUEUES ==========

// safe from ISR

void Wire_master_interface::getSendStats(I2C_m_sendStats& sendStatSnapshot) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        sendStatSnapshot = const_cast<const I2C_m_sendStats&>(sendStats);
    }
}

void Wire_master_interface::getReceiveStats(I2C_m_receiveStats& receiveStatSnapshot) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        receiveStatSnapshot = const_cast<const I2C_m_receiveStats&>(receiveStats);
    }
}


// ================= HELPERS: SEND TX QUEUE TAIL  =================

bool Wire_master_interface::copyTXqueueTailToOut(uint8_t* const out, uint8_t& expReplyMsgSize) {
    uint8_t head, tail;
    head = txHead;
    tail = txTail;

    if (head == tail) { return false; }                             // empty

    // AVR: strongly ordered, no hardware fences needed BUT memory fence added to keep code generic.
    acquire_barrier();                                              // ensure we see latest tail before reading data

    uint8_t msgLength = HEADER_SIZE + txQueue[tail][1] + 1;         // message ID, payload length, checksum
    memcpy(out, (uint8_t*)txQueue[tail], msgLength);                // copy struct
    expReplyMsgSize = txExpReplyMsgSize[tail];                      // copy as well (0xff: no reply expected)
    
    Serial.print("COPY txQueue to OUT: expected size "); Serial.println(expReplyMsgSize);
    Serial.print("msg out type + length ");Serial.print(txQueue[tail][0]);Serial.write(" "); Serial.println(msgLength);
    for (int i = 0; i < msgLength; i++) {
        Serial.print(txQueue[tail][i], HEX); Serial.write(' '); // last is checksum as received
    }
    Serial.write(' '), Serial.println(millis());Serial.println();

    sendMaxTries = 0;
    return true;
}


bool Wire_master_interface::i2cWriteMessage(const uint8_t* out) {
    uint8_t msgLength = HEADER_SIZE + out[1] + 1;                   // message ID, payload length, checksum
    bool ok{ false };
    Wire.beginTransmission(I2C_SLAVE_ADDR);
    Wire.write(out, msgLength);
    uint8_t err = Wire.endTransmission();                           // 0 == success
    ok = (err == 0);
    return ok;
}


// ========== HELPERS: RECEIVE RX QUEUE HEAD ===========

bool Wire_master_interface::i2cReadMessage(uint8_t* in, uint8_t expReplyMsgSize) {
    // if the slave sends less  bytes than expected, the Wire master will pad with 0xFF bytes. Extra bytes sent are simply discarded 

    const uint32_t timeoutValue = 100 + 150 * expReplyMsgSize;     // microseconds; allowed timeout depends on message size
    Wire.requestFrom(I2C_SLAVE_ADDR, (uint8_t)expReplyMsgSize);

    unsigned long start = micros();
    uint8_t idx = 0;
    bool ok{ false };

    // while loop: must execute fast to free internal Wire buffer
    while (idx < expReplyMsgSize) {
        if (Wire.available()) {
            ////delayMicroseconds(1000);
            in[idx++] = Wire.read();
        }
        else {  //// check timeout value
            if ((micros() - start) > timeoutValue) { break; }   // blocking operation: timeout (message only partially received)     
        }
    }

    ok = (idx == expReplyMsgSize);//// <> timeout msg
    
    return ok;
}


Wire_master_interface::WireStatus Wire_master_interface::copyInToRXqueueHead(uint8_t* const in, uint8_t expReplyMsgSize) {
    // compute next head index in standard SPSC ring-buffer
    uint8_t head = rxHead;
    uint8_t next = (head + 1) % RX_QUEUE_SIZE;

    if (next == rxTail) {    // Buffer full -> drop packet
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { receiveStats.E_stats_rx_full++; }    // message dropped (32-bit stats_ variable increment must be atomic
        return WireStatus::E_rx_full;
    }

    uint8_t sum = 0;
    for (uint8_t i = 0; i < expReplyMsgSize - 1; ++i) {             // includes header and payload, excludes received checksum from checksum calculation
        rxQueue[head][i] = in[i];
        sum ^= in[i];
    }
    rxQueue[head][expReplyMsgSize-1] = in[expReplyMsgSize-1];

    Serial.print("COPY in TO rxQueue: expected size "); Serial.println(expReplyMsgSize);
    for (int i = 0; i < expReplyMsgSize - 1; i++) {
        Serial.print(rxQueue[head][i], HEX); Serial.write(' '); // last is checksum as received
    }
    Serial.print(rxQueue[head][expReplyMsgSize-1], HEX); Serial.write(':'), Serial.print(sum, HEX);
    Serial.write(' '), Serial.print(millis());

    if (sum != rxQueue[head][expReplyMsgSize - 1]) {                           // checksum correct ?
        Serial.println("*******\r\n--------------");
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { receiveStats.E_stats_rx_checksum++; }// message dropped (32-bit stats_ variable increment must be atomic
        return WireStatus::E_rx_checksum;
    };
    Serial.println("\r\n--------------");


    // AVR: strongly ordered, no hardware fences needed BUT memory fence added to keep code generic.
    release_barrier();                                              // ensure data is visible before updating head

    rxHead = next;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { receiveStats.I_stats_received++; }
    return WireStatus::I_xmitOK;
}




