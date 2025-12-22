#include <Wire.h>
#include "wire_master_interface.h"

Wire_master_interface::Wire_master_interface() {
    Wire.begin(); // join I2C bus (address optional for master)
    ////Wire.setClock(I2C_CLOCK);
}

Wire_master_interface::~Wire_master_interface() {
}



/* ================= RING BUFFER (ISR-safe enqueue) ================= */

/*
  enqueueTx: safe from ISR. Returns true if enqueued, false if queue full.
  On success sets retry count for that slot to 0 (main loop must not overwrite).
*/


bool Wire_master_interface::enqueueTx(uint8_t type, uint8_t payloadSize, const void* payload, uint8_t expReplyPayloadSize) {
    uint8_t next = (txHead + 1) % TX_QUEUE_SIZE;
    if (next == txTail) { stats_dropped++; return false; }    // queue full

    txQueue[txHead][0] = type;         // struct copy
    txQueue[txHead][1] = payloadSize;
    uint8_t sum = type ^ payloadSize;
    for (uint8_t i = 0; i < payloadSize; ++i) {
        txQueue[txHead][HEADER_SIZE + i] = ((uint8_t*)payload)[i];
        sum ^= ((uint8_t*)payload)[i];
    }

    txQueue[txHead][HEADER_SIZE + payloadSize] = sum;
    // remember reply MESSAGE SIZE, in queue: THIS will decide if data is requested from a slave. 0xff if no reply expected
    txExpReplyMsgSize[txHead] = (expReplyPayloadSize == 0xff) ? 0xff : HEADER_SIZE + expReplyPayloadSize + 1;

    txHead = next;
    return true;
};


bool Wire_master_interface::dequeueRx(uint8_t& type, uint8_t& payloadSize, void* payload) {
    if (!rxAvailable) { return false; }        // no data to dequeue

    type = rxQueue[0];
    payloadSize = rxQueue[1];
    for (uint8_t i = 0; i < payloadSize; ++i) {
        ((uint8_t*)payload)[i] = rxQueue[HEADER_SIZE + i];
    }

    rxAvailable = false;
    return true;

};


/* ================= I2C HELPERS ================= */

// ========== receive data from the slave ===========


/* ================= COPY QUEUE TAIL (atomic read of tail/head) =================
   Peek at the current packet at txTail without advancing the tail.
   Returns true and fills txWorking if not empty; false if empty.
*/

bool Wire_master_interface::copyTXqueueTail(uint8_t* const out, uint8_t& indexAtTail, uint8_t& expReplyMsgSize) {     //// indexAtTail: weg ? (doet niets)
    uint8_t head, tail;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        head = txHead;
        tail = txTail;
    }

    if (head == tail) { return false; } // empty

    indexAtTail = tail;
    uint8_t msgLength = HEADER_SIZE + txQueue[tail][1] + 1;            // message ID, payload length, checksum
    memcpy(out, (uint8_t*)txQueue[tail], msgLength); // copy struct
    expReplyMsgSize = txExpReplyMsgSize[tail];            // copy as well (0xff: no reply expected)
    sendRetryCount = 0;
    return true;
}


bool Wire_master_interface::i2cWriteMessage(const uint8_t* p) {
    uint8_t msgLength = HEADER_SIZE + p[1] + 1;            // message ID, payload length, checksum
    Wire.beginTransmission(I2C_SLAVE_ADDR);
    Wire.write(p, msgLength);
    uint8_t err = Wire.endTransmission(); // 0 == success
    return (err == 0);
}


// pop tx tail (advance)
void Wire_master_interface::popTXtailAtomic() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        txTail = (txTail + 1) % TX_QUEUE_SIZE;
    }
}


// ========== receive data from the slave ===========

uint8_t Wire_master_interface::i2cReadMessage(uint8_t* p, uint8_t expReplyMsgSize) {
    // always a reply, even if no payload (at least msg type and length)
    // if the slave sends less  bytes than expected, the master will pad with 0xFF bytes. Extra bytes sent are simply discarded 

    const uint32_t timeoutValue = 1000 + 150 * expReplyMsgSize;       // microseconds
    Wire.requestFrom(I2C_SLAVE_ADDR, (uint8_t)expReplyMsgSize);

    unsigned long start = micros();
    uint8_t idx = 0;
    while (idx < expReplyMsgSize) {
        if (Wire.available()) {
            p[idx++] = Wire.read();
        }
        else {
            if ((unsigned long)(micros() - start) > timeoutValue) break;        // blocking operation: timeout (message only partially received)     
        }
    }
    return idx;
}


bool Wire_master_interface::copyRXqueue(uint8_t* const in, uint8_t expReplyMsgSize) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        bool available = rxAvailable;
        if (available) { return false; }      // a reply is already available for dequeue
    }

    uint8_t sum = 0;
    for (uint8_t i = 0; i < expReplyMsgSize - 1; ++i) {     // includes header and payload, excludes received checksum from checksum calculation
        rxQueue[i] = in[i];
        sum ^= in[i];
    }

    // at this point rxAvailable is false, an ISR will never set it true
    rxAvailable = (sum == in[expReplyMsgSize - 1]);        // checksum correct ?
    return rxAvailable;
}




// ========== must be frequently called from within main loop ==========

void Wire_master_interface::sendAndReceiveMessage() {
    unsigned long currentTime = millis();

    static uint8_t expReplyMsgSize{};
    // If currently idle, attempt to peek next packet

    if (state == MS_IDLE) {
        uint8_t indexAtTail;
        if (!copyTXqueueTail(txWorking, indexAtTail, expReplyMsgSize)) { return; }// nothing to send
        // We have txWorking (peeked) and indexAtTail points to its slot in queue.
        sendRetryCount = 0;
        state = MS_SEND;
    }

    // SEND state: attempt send of txWorking (which is the packet at queue tail)
    if (state == MS_SEND) {
        // Respect scheduling/backoff
        if (lastCycleTime + CYCLE_PERIOD_MS > currentTime) {Serial.write('='); return; }
        lastCycleTime = currentTime;      // init: assume transmission will be ok

        // check inter-packet spacing
        bool ok = i2cWriteMessage(txWorking);

        if (!ok) {
          // failure: increment retry count for the slot (atomic)
          // we need to find index of the slot we peeked earlier: it's the current txTail
            if (sendRetryCount < 255) sendRetryCount++;
            stats_errors++;

            // If exceeded MAX_RETRIES, drop the packet (advance tail)
            if (sendRetryCount > MAX_RETRIES_PER_PACKET) {
                popTXtailAtomic();
                stats_dropped++;
                state = MS_IDLE;
                return;
            }

            // keep the SEND state
            return;
        }

        // success: advance send counters and pop the packet (advance tail atomically)
        popTXtailAtomic();
        stats_sent++;
        // after successful write, allow optional receive based on message type

        static bool ledState{};
        state = (expReplyMsgSize == 0xff) ? MS_IDLE : MS_RECEIVE;
        digitalWrite(13, ledState = !ledState);
    }




    // RECEIVE state: attempt read if expected by protocol
    if (state == MS_RECEIVE) {
        // Respect scheduling/holdoff
        if (lastCycleTime + CYCLE_PERIOD_MS > currentTime) { return; }
        lastCycleTime = currentTime;      // init: assume transmission will be ok
        //delay(1);

        uint8_t rxReceivedCount = i2cReadMessage(rxWorking, expReplyMsgSize);
        if (rxReceivedCount != expReplyMsgSize) { stats_errors++; }               // do NOT re-enqueue; we drop/ignore reply
        else if (copyToRXqueueHead(rxWorking, expReplyMsgSize)) {  }    ////
        else { stats_errors++; }

        if (!rxAvailable) { Serial.println("  nothing received"); }

    // after receive (or no expected), go back to idle
        state = MS_IDLE;
    }
}








