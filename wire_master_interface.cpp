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
    2. atomicity of operations: required for operations that can be interrupted by operations accessing the same variables.
       if indexes are 8-bit: index reads and writes are atomic: NO atomic blocks required while reading / writing indexes, BUT
       atomic blocks ARE NEEDED when performing read-modify-write operations (which are not atomic by nature)
       operations with 16 or 32-bit variables (error counters etc.) MUST ALWAYS BE ATOMIC if shared between main and ISR
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

bool Wire_master_interface::enqueueTx(uint8_t msgType, uint8_t payloadSize, const void* payload, uint8_t expReplyPayloadType, uint8_t expReplyPayloadSize) {
    uint8_t next = (txHead + 1) % TX_QUEUE_SIZE;
    if (next == txTail) {                                           // queue full ?
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { masterSendStats.E_stats_tx_full++; }    // message dropped (32-bit stats_ variable increment must be atomic
        Serial.println("!! Enqueue: tx full");
        return false;
    }

    txQueue[txHead][0] = msgType;
    txQueue[txHead][1] = payloadSize;
    uint8_t sum = msgType ^ payloadSize;
    for (uint8_t i = 0; i < payloadSize; ++i) {
        txQueue[txHead][HEADER_SIZE + i] = ((uint8_t*)payload)[i];
        sum ^= ((uint8_t*)payload)[i];
    }

    txQueue[txHead][HEADER_SIZE + payloadSize] = sum;
    // remember REPLY MESSAGE SIZE, in queue: THIS will decide if data is requested from a slave. 0xff if no reply expected
    txExpReplyMsgType[txHead] = expReplyPayloadType;
    txExpReplyMsgSize[txHead] = (expReplyPayloadSize == 0xff) ? 0xff : HEADER_SIZE + expReplyPayloadSize + 1;

    Serial.print(F("\r\n\r\nENQUEUE-seq ")); Serial.print(txQueue[txHead][2], HEX); Serial.print(" - msg type "); Serial.println(msgType, HEX);

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

    Serial.print(F("DEQUEUE-seq ")); Serial.println(rxQueue[rxTail][2], HEX);

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

Wire_master_interface::WireStatus Wire_master_interface::sendAndReceiveMessage() {

    static uint8_t expReplyMsgType, expReplyMsgSize{};

    static unsigned long lastTaskTime_millis = 0;                                // delay scheduling, periods > 10 ms
    static unsigned long lastTaskTime_micros = 0;                                // delay scheduling, periods < 10 ms
    static unsigned long lastPollTime_micros = 0;

    static uint8_t sendRetryCount = 0;

    unsigned long now_millis = millis();
    unsigned long now_micros = micros();

    unsigned long timePassed{};

    /* ---------- state machine: MS_IDLE: enqueue message if available ---------- */

    if (state == MasterState::MS_IDLE) {                                         // not sending or receiving ?
        if (!copyTXqueueTailToOut(txOutBuffer, expReplyMsgType, expReplyMsgSize)) { return WireStatus::I_idle; }  // message available ? Copy to out buffer
        sendRetryCount = 0;
        state = MasterState::MS_SEND;                                           // change status to MS_SEND 
        Serial.println(F("-> SEND"));
    }



    /* ---------- state machine: MS_SEND: send message to slave (respect inter-message spacing) ---------- */

    if (state == MasterState::MS_SEND) {
        // Respect scheduling (backoff time)

        // check inter-message spacing (non-blocking)
        timePassed = (now_micros - lastTaskTime_micros);            // unsigned integer subtraction: modulo 2^32
        if (timePassed < MIN_CYCLE_PERIOD_MICROS) { return WireStatus::I_waitForCue; }
        lastTaskTime_millis = now_millis;                                // init: assume transmission will be ok
        lastTaskTime_micros = now_micros;

        // transmit; in case of error, retry a few times (non-blocking) 
        bool ok = i2cWriteMessage(txOutBuffer);                     // Wire write
        if (!ok) {                                                  // Wire library transmit error 
            // manage retries: if exceeded MAX_RETRIES, drop the packet (advance tail)
            if (++sendRetryCount < MAX_RETRIES_PER_PACKET) {          // max. retries was reached before ?
                // stay in SEND state: max. tries not yet attempted
                ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { masterSendStats.W_stats_tx_retrying++; } // warning only: count retries
                return WireStatus::E_tx_WireXmitError;
            }

            state = MasterState::MS_IDLE;                    // go back to IDLE
            Serial.println(F("-> IDLE-write error"));
            // error: advance txTail, increment counter and pop the packet (advance tail atomically)
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                txTail = (txTail + 1) % TX_QUEUE_SIZE;              // advance tx tail (even with 8-bit length, operation  is not atomic)
                masterSendStats.E_stats_tx_wireXmitError++;               // after max retries: error
            }
            return WireStatus::E_tx_WireXmitError;
        }


        // success: advance txTail, increment counter and pop the packet (advance tail atomically)
        // after successful write, allow optional receive based on message msgType
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            txTail = (txTail + 1) % TX_QUEUE_SIZE;                                  // advance tx tail (even with 8-bit length, operation  is not atomic)
            masterSendStats.I_stats_sent++;
        }

        state = MasterState::MS_WAIT_BEFORE_POLLING;        ////   // 0xff: msg does not expect a reply
        Serial.println(F("-> WAIT BEFORE POLL"));
    }


    /* ---------- state machine: MS_WAIT_BEFORE_POLLING ---------- */

    // wait a fixed time (the minimum cycle time) before starting polling
    if (state == MasterState::MS_WAIT_BEFORE_POLLING) {
        timePassed = (now_micros - lastTaskTime_micros);        // unsigned integer subtraction: modulo 2^32
        if (timePassed < MIN_CYCLE_PERIOD_MICROS) { return WireStatus::I_waitForCue; }  // Respect scheduling (holdoff)
        lastPollTime_micros = lastTaskTime_micros = now_micros;              // init
        lastTaskTime_millis = now_millis;

        state = MasterState::MS_WAIT_FOR_SLAVE_READY;
        Serial.println(F("-> WAIT SLAVE READY"));
    }



    /* ---------- state machine: poll until slave ready or timeout (non-blocking) ---------- */

    if (state == MasterState::MS_WAIT_FOR_SLAVE_READY) {
        timePassed = (now_millis - lastTaskTime_millis);         // unsigned integer subtraction: modulo 2^32
        if (timePassed > MAX_RECEIVE_CYCLE_MILLIS) {
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { masterReceiveStats.E_stats_rx_timeOut++; }
            state = MasterState::MS_IDLE;
            Serial.println(F("-> IDLE-no slave"));
            return WireStatus::E_rx_timeOut;                        // reply did not arrive in time
        }

        // poll every (SLAVE_POLL_INTERVAL_micros)
        timePassed = now_micros - lastPollTime_micros;         // unsigned integer subtraction: modulo 2^32
        if (timePassed < SLAVE_POLL_INTERVAL_micros ) { return WireStatus::I_waitForCue; }
        lastPollTime_micros = now_micros;
        Serial.println(F("poll now"));

        // start polling
        constexpr uint8_t request = (uint8_t)WireTransport::M_CTRL_POLL;
        uint8_t reply{ (uint8_t)WireTransport::S_CTRL_BUSY };                 // init, will be unchanged if write error 
        Wire.beginTransmission(I2C_SLAVE_ADDR);
        Wire.write(&request, 1);
        uint8_t err = Wire.endTransmission();                           // 0 == success
        if (err == 0) {
            if (i2cReadMessage(&reply, 0, 1)) {
                // Any poll result other than READY is treated as BUSY by design
                if (reply != (uint8_t)WireTransport::S_CTRL_READY) {       // an erroneous reply byte is captured here (although nothing is done with it) 
                    reply = (uint8_t)WireTransport::S_CTRL_BUSY;
                }
            }
            else { reply = (uint8_t)WireTransport::S_CTRL_BUSY; }       // reading busy/ready reply timeout error (although nothing is done with it)
        }
        Serial.print(F("slave ready ? ")); Serial.println(reply == (uint8_t)WireTransport::S_CTRL_READY);      // slave ready to send ?

        if (reply == (uint8_t)WireTransport::S_CTRL_READY) {
            state = MasterState::MS_RECEIVE;
            Serial.println(F("-> RECEIVE"));
        }
    }


    /* ---------- state machine: receive slave message  ---------- */

    // RECEIVE state: attempt read if expected by protocol
    // NOTE: one message out results in one message in (1:1)

    if (state == MasterState::MS_RECEIVE) {
        timePassed = now_micros - lastTaskTime_micros;         // unsigned integer subtraction: modulo 2^32
        if (timePassed < MIN_CYCLE_PERIOD_MICROS) { return WireStatus::I_waitForCue; }  // Respect scheduling (holdoff)
        lastTaskTime_millis = now_millis;                                    // init
        lastTaskTime_micros = now_micros;                                    // init

        bool ok = i2cReadMessage(rxInBuffer, expReplyMsgType, expReplyMsgSize);
        
        if (!ok) {
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { masterReceiveStats.E_stats_rx_timeOut++; } // do NOT re-enqueue; we drop/ignore reply
            state = MasterState::MS_IDLE;
            Serial.println(F("-> IDLE-read error"));
            return WireStatus::E_rx_timeOut;
        }
        else { Serial.print(F("Wire in-seq ")); Serial.println(rxInBuffer[2], HEX); }
        WireStatus wireStatus = copyInToRXqueueHead(rxInBuffer, expReplyMsgType, expReplyMsgSize);
        state = MasterState::MS_IDLE;
        Serial.println(F("-> IDLE-receive success"));
        return wireStatus;
    }

    return WireStatus::I_idle;                                          // safety (control doesn't pass here)
}


// ========== GET master send and receive stats ==========

void Wire_master_interface::getSendStats(I2C_MasterSendStats& sendStatSnapshot) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        sendStatSnapshot = const_cast<const I2C_MasterSendStats&>(masterSendStats);
    }
}

void Wire_master_interface::getReceiveStats(I2C_masterReceiveStats& receiveStatSnapshot) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        receiveStatSnapshot = const_cast<const I2C_masterReceiveStats&>(masterReceiveStats);
    }
}


// ================= HELPERS: SEND TX QUEUE TAIL  =================

bool Wire_master_interface::copyTXqueueTailToOut(uint8_t* const out, uint8_t& expReplyMsgType, uint8_t& expReplyMsgSize) {
    uint8_t head, tail;
    head = txHead;
    tail = txTail;

    if (head == tail) { return false; }                             // empty

    // AVR: strongly ordered, no hardware fences needed BUT memory fence added to keep code generic.
    acquire_barrier();                                              // ensure we see latest tail before reading data

    uint8_t msgLength = HEADER_SIZE + txQueue[tail][1] + 1;         // message ID, payload length, checksum
    memcpy(out, (uint8_t*)txQueue[tail], msgLength);                // copy struct
    expReplyMsgType = txExpReplyMsgType[tail];
    expReplyMsgSize = txExpReplyMsgSize[tail];                      // copy as well (0xff: no reply expected)

    Serial.print(F("txQueue to OUT-seq ")); Serial.print(out[2], HEX); Serial.print(F("/msg: "));
    for (int i = 0; i < msgLength; i++) {
        Serial.print(txQueue[tail][i], HEX); Serial.write(' '); // last is checksum as received
    }
    Serial.write('('), Serial.print(millis()); Serial.println(F(")"));

    return true;
}


bool Wire_master_interface::i2cWriteMessage(const uint8_t* out) {

    uint8_t msgLength = HEADER_SIZE + out[1] + 1;                   // message ID, payload length, checksum
    bool ok{ false };
    Wire.beginTransmission(I2C_SLAVE_ADDR);
    Wire.write(out, msgLength);
    uint8_t err = Wire.endTransmission();                           // 0 == success
    ok = (err == 0);

    Serial.print(F("Wire out-seq ")); Serial.println(out[2], HEX);
    return ok;
}


// ========== HELPERS: RECEIVE RX QUEUE HEAD ===========

bool Wire_master_interface::i2cReadMessage(uint8_t* in, uint8_t expReplyMsgType, uint8_t expReplyMsgSize) {
    // if the slave sends less  bytes than expected, the Wire master will pad with 0xFF bytes. Extra bytes sent are simply discarded 

    const uint32_t timeoutValue = 100 + 150 * expReplyMsgSize;     // microseconds; allowed timeout depends on message size  //// check constant and first degree term
    Wire.requestFrom(I2C_SLAVE_ADDR, (uint8_t)expReplyMsgSize);

    unsigned long start_micros = micros();
    uint8_t idx = 0;
    bool ok{ false };

    // while loop: must execute fast to free internal Wire buffer
    while (idx < expReplyMsgSize) {
        if (Wire.available()) {
            in[idx++] = Wire.read();
        }
        else {  //// check timeout value
            if ((micros() - start_micros) > timeoutValue) { break; }   // blocking operation: timeout (message only partially received)
        }
    }

    ok = (idx == expReplyMsgSize);

    return ok;
}


Wire_master_interface::WireStatus Wire_master_interface::copyInToRXqueueHead(uint8_t* const in, uint8_t expReplyMsgType, uint8_t expReplyMsgSize) {
    // compute next head index in standard SPSC ring-buffer
    uint8_t head = rxHead;
    uint8_t next = (head + 1) % RX_QUEUE_SIZE;

    if (next == rxTail) {    // Buffer full -> drop packet
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { masterReceiveStats.E_stats_rx_full++; }    // message dropped (32-bit stats_ variable increment must be atomic
        return WireStatus::E_rx_full;
    }

    uint8_t sum = 0;
    for (uint8_t i = 0; i < expReplyMsgSize - 1; ++i) {             // includes header and payload, excludes received checksum from checksum calculation
        rxQueue[head][i] = in[i];
        sum ^= in[i];
    }
    rxQueue[head][expReplyMsgSize - 1] = in[expReplyMsgSize - 1];

    if (sum != rxQueue[head][expReplyMsgSize - 1]) {                           // checksum correct ?
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { masterReceiveStats.E_stats_rx_checksum++; }// message dropped (32-bit stats_ variable increment must be atomic
        return WireStatus::E_rx_checksum;
    };

    Serial.print(F("IN to rxQueue-seq ")); Serial.print(in[2], HEX); Serial.print(F("/msg: "));
    for (int i = 0; i < expReplyMsgSize - 1; i++) {
        Serial.print(rxQueue[head][i], HEX); Serial.write(' '); // last is checksum as received
    }
    Serial.print(rxQueue[head][expReplyMsgSize - 1], HEX); Serial.write(':'), Serial.print(sum, HEX);
    Serial.write('('), Serial.print(millis()); Serial.println(F(")"));



    // AVR: strongly ordered, no hardware fences needed BUT memory fence added to keep code generic.
    release_barrier();                                              // ensure data is visible before updating head

    rxHead = next;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { masterReceiveStats.I_stats_received++; }

    return WireStatus::I_xmitOK;
}




