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


bool Wire_master_interface::enqueueTx(uint8_t type, const void* payload, uint8_t payloadSize, uint8_t replyPayloadSize) {
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
    expectedRxMsgLength[txHead] = HEADER_SIZE + replyPayloadSize + 1;
    txHead = next;
    return true;
};


bool Wire_master_interface::dequeueRx(uint8_t& type, void* payload, uint8_t& payloadSize) {
    if (!rxAvailable) { return false; }        // no data to dequeue

    type = rxQueue[0];
    payloadSize = rxQueue[1];
    memcpy((uint8_t*)payload, (uint8_t*)(rxQueue)+HEADER_SIZE, payloadSize);
    rxAvailable = false;
    return true;

};


/* ================= I2C HELPERS ================= */

/* ================= COPY QUEUE TAIL (atomic read of tail/head) =================
   Peek at the current packet at txTail without advancing the tail.
   Returns true and fills txWorking if not empty; false if empty.
*/

bool Wire_master_interface::copyTXqueueTail(uint8_t* const out, uint8_t& indexAtTail) {     //// indexAtTail: weg ? (doet niets)
    uint8_t head, tail;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        head = txHead;
        tail = txTail;
    }

    if (head == tail) { return false; } // empty

    indexAtTail = tail;
    uint8_t msgLength = HEADER_SIZE + txQueue[tail][1] + 1;            // message ID, payload length, checksum
    memcpy(out, (uint8_t*)txQueue[tail], msgLength); // copy struct
    rxMsgLength = expectedRxMsgLength[tail];            // still expected at this time
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

uint8_t Wire_master_interface::i2cReadMessage(uint8_t* p) {
    // always a reply, even if no payload (at least msg type and length)
    Wire.requestFrom(I2C_SLAVE_ADDR, (uint8_t)rxMsgLength);

    unsigned long start = micros();
    uint8_t idx = 0;
    while (idx < rxMsgLength) {
        if (Wire.available()) {
            p[idx++] = Wire.read();
        }
        else {
            if ((unsigned long)(micros() - start) > 8000) break;        // message only partially received
        }
    }
    return idx;
}


bool Wire_master_interface::copyRXqueue(uint8_t* const in) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        bool available = rxAvailable;
        if (available) { return false; }      // a reply is already available for dequeue
    }

    uint8_t sum = 0;
    for (uint8_t i = 0; i < rxMsgLength - 1; ++i) {     // includes header and payload, excludes received checksum from checksum calculation
        rxQueue[i] = in[i];
        sum ^= in[i];
    }

    // at this point rxAvailable is false, an ISR will never set it true
    rxAvailable = (sum == in[rxMsgLength - 1]);        // checksum correct ?
    return rxAvailable;
}


// pop tx tail (advance)
void Wire_master_interface::popTXtailAtomic() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        txTail = (txTail + 1) % TX_QUEUE_SIZE;
    }
}


// ========== must be frequently called from within main loop ==========

void Wire_master_interface::sendAndReceiveMessage() {
    unsigned long currentTime = millis();

    // If currently idle, attempt to peek next packet
    
    if (state == MS_IDLE) {
        uint8_t indexAtTail;
        if (!copyTXqueueTail(txWorking, indexAtTail)) { return; }// nothing to send
        // We have txWorking (peeked) and indexAtTail points to its slot in queue.
        sendRetryCount = 0;
        state = MS_SEND;
    }

    // SEND state: attempt send of txWorking (which is the packet at queue tail)
    if (state == MS_SEND) {
        // Respect scheduling/backoff
        if (currentTime < nextCycleTime) return;
        nextCycleTime = currentTime + CYCLE_PERIOD_MS;      // init: assume transmission will be ok

        // check inter-packet spacing
        bool ok = i2cWriteMessage(txWorking);

        if (!ok) {
          // failure: increment retry count for the slot (atomic)
          // we need to find index of the slot we peeked earlier: it's current txTail
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
            nextCycleTime = currentTime + BACKOFF_DELAY_MS;      // do not wait a full cycle time to retry 
            return;
        }

        // success: advance send counters and pop the packet (advance tail atomically)
        popTXtailAtomic();
        stats_sent++;
        // after successful write, allow optional receive based on message type
        
        static bool ledState    {};
        digitalWrite(13, ledState = !ledState);

        
        ////
        state = MS_IDLE;
        //state = MS_RECEIVE;  //// ms_idle: for testing (nothing to receive yet)
    }




    // RECEIVE state: attempt read if expected by protocol
    if (state == MS_RECEIVE) {
        if (rxMsgLength > HEADER_SIZE + 1) {    // payload expected
            uint8_t rxReceivedCount = i2cReadMessage(rxWorking);
            if (rxReceivedCount != rxMsgLength) { stats_errors++; }               // do NOT re-enqueue; we drop/ignore reply
            else if (copyRXqueue(rxWorking)) { rxAvailable = true; }
            else { stats_errors++; }
        
            if(rxAvailable){ Serial.print("slave response type: "); Serial.print(rxQueue[1], DEC); Serial.print(", reply payload size: "); Serial.println(rxQueue[1], DEC);}
            else{Serial.println("  nothing received"); }
        }
        // after receive (or no expected), go back to idle
        state = MS_IDLE;
    }
}








