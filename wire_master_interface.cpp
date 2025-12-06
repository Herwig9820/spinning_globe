#include <Wire.h>
#include "wire_master_interface.h"

Wire_master_interface::Wire_master_interface() {
    Wire.begin(); // join I2C bus (address optional for master)
    ////Wire.setClock(I2C_CLOCK);
    for (uint8_t i = 0; i < TX_QUEUE_SIZE; ++i) retryCounts[i] = 0;     // init: zero retry counts
}

Wire_master_interface::~Wire_master_interface() {
}



/* ================= RING BUFFER (ISR-safe enqueue) ================= */

/*
  enqueueTxISR: safe from ISR. Returns true if enqueued, false if queue full.
  On success sets retry count for that slot to 0 (main loop must not overwrite).
*/

bool Wire_master_interface::enqueueTxISR(uint8_t type, const void* payload, uint8_t length){
};


bool Wire_master_interface::enqueueTxISR(const PacketTX& p) {
    uint8_t next = txHead + 1;
    if (next >= TX_QUEUE_SIZE) next = 0;
    if (next == txTail)return false;
    // queue full
    txQueue[txHead] = p;         // struct copy
    retryCounts[txHead] = 0;     // initialize retries (main loop only modifies)
    txHead = next;
    return true;
}


/* ================= CHECKSUM ================= */

uint8_t Wire_master_interface::calcChecksum( PacketTX &p, uint8_t len) {
    uint8_t* data = reinterpret_cast<uint8_t*>(&p);
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; ++i) sum ^= data[i];
    return sum;
}

uint8_t Wire_master_interface::calcChecksum(PacketRX& p, uint8_t len) {
    uint8_t* data = reinterpret_cast<uint8_t*>(&p);
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; ++i) sum ^= data[i];
    return sum;
}



/* ================= I2C HELPERS ================= */

bool Wire_master_interface::i2cWritePacket(const PacketTX& p) {
    uint8_t sendLen = (uint8_t)(2 + p.len + 1); // type + size + payload + checksum
    Wire.beginTransmission(I2C_SLAVE_ADDR);
    Wire.write(reinterpret_cast<const uint8_t*>(&p), sendLen);
    uint8_t err = Wire.endTransmission(); // 0 == success
    return (err == 0);
}

uint8_t Wire_master_interface::i2cMasterReadWithTimeout(uint8_t len, PacketRX p, unsigned long reply_timeout_ms) {
    const unsigned long reply_timeout_us = reply_timeout_ms * 1000ul;
    if (len == 0) return 0;
    Wire.requestFrom(I2C_SLAVE_ADDR, (uint8_t)len);
    
    unsigned long start = micros();
    uint8_t idx = 0;
    uint8_t* dst = reinterpret_cast<uint8_t*>(&p);
    while (idx < len) {
        if (Wire.available()) {
            dst[idx++] = Wire.read();
        }
        else {
            if ((unsigned long)(micros() - start) > reply_timeout_us) break;
        }
    }
    return idx;
}


/* ================= QUEUE PEEK (atomic read of tail/head) =================
   Peek at the current packet at txTail without advancing the tail.
   Returns true and fills txWorking if not empty; false if empty.
*/
bool Wire_master_interface::peekQueueTail(PacketTX& out, uint8_t& indexAtTail) {
    uint8_t head, tail;
    uint8_t sreg = SREG;
    cli();
    head = txHead;
    tail = txTail;
    SREG = sreg;

    if (head == tail) return false; // empty

    indexAtTail = tail;
    out = txQueue[tail]; // copy struct
    return true;
}


// pop tail (advance)
void Wire_master_interface::popTailAtomic() {
    uint8_t sreg = SREG;
    cli();
    uint8_t newTail = txTail + 1;
    if (newTail >= TX_QUEUE_SIZE) newTail = 0;
    // clear retry counter of popped slot
    retryCounts[txTail] = 0;
    txTail = newTail;
    SREG = sreg;
}


uint32_t Wire_master_interface::incr_stats_sent(){return ++stats_sent;}
uint32_t Wire_master_interface::incr_stats_dropped(){return ++stats_dropped;}
uint32_t Wire_master_interface::incr_stats_errors(){return ++stats_errors;}


void Wire_master_interface::processQueueAndSend() {
    unsigned long now = millis();

    // Respect scheduling/backoff
    if (now < nextRetryMillis) return;

    // If currently idle, attempt to peek next packet
    if (state == MS_IDLE) {
        uint8_t indexAtTail;
        if (!peekQueueTail(txWorking, indexAtTail)) {
          // nothing to send
            return;
        }
        // We have txWorking (peeked) and indexAtTail points to its slot in queue.
        state = MS_SEND;
    }

    // SEND state: attempt send of txWorking (which is the packet at queue tail)
    if (state == MS_SEND) {
      // check inter-packet spacing
        if (millis() < nextActionMillis) return;

        // compute total bytes to send: id + len + payload + checksum = 3 + len -  ??? 
        // formula: id(1) + len(1) + payload(len) + checksum(1) => 3 + txWorking.len
        uint8_t outLen = 3 + txWorking.len;

        bool ok = i2cWritePacket(txWorking);
        Serial.print("wire send message type "); Serial.print(txWorking.id); Serial.print(" -  success ? "); Serial.println(ok);

        if (!ok) {
          // failure: increment retry count for the slot (atomic)
          // we need to find index of the slot we peeked earlier: it's current txTail
            uint8_t sreg = SREG;
            cli();
            uint8_t slot = txTail; // the packet we attempted was at tail
            // increment retry count atomically
            if (retryCounts[slot] < 255) retryCounts[slot]++;
            uint8_t tries = retryCounts[slot];
            SREG = sreg;

            stats_errors++;

            // If exceeded MAX_RETRIES, drop the packet (advance tail)
            if (tries > MAX_RETRIES_PER_PACKET) {
                sreg = SREG;
                cli();
                // advance tail
                uint8_t newTail = txTail + 1;
                if (newTail >= TX_QUEUE_SIZE) newTail = 0;
                txTail = newTail;
                // reset retry count for that slot
                retryCounts[slot] = 0;
                SREG = sreg;

                stats_dropped++;
                // schedule a small delay before trying next message
                nextRetryMillis = now + RETRY_BACKOFF_MS;
                state = MS_IDLE;
                return;
            }

            // schedule a backoff and stay in MS_IDLE (we will peek again later and try same packet)
            nextRetryMillis = now + RETRY_BACKOFF_MS;
            state = MS_IDLE;
            return;
        }

        // success: advance send counters and pop the packet (advance tail atomically)
        stats_sent++;
        uint8_t sreg = SREG;
        cli();
        // advance tail (pop)
        uint8_t newTail = txTail + 1;
        if (newTail >= TX_QUEUE_SIZE) newTail = 0;
        // clear retry count for that slot (defensive)
        retryCounts[txTail] = 0;
        txTail = newTail;
        SREG = sreg;

        // after successful write, allow optional receive based on message type
        state = MS_RECEIVE;
        nextActionMillis = now + CYCLE_MS;
    }

    // RECEIVE state: attempt read if expected by protocol
    if (state == MS_RECEIVE) {
        if (millis() < nextActionMillis) return;

        uint8_t expectedRX = 0;
        // policy: define expected reply lengths for certain requests
        if (txWorking.id == MSG_GET_SETTINGS) expectedRX = 5;
        else if (txWorking.id == MSG_PING) expectedRX = 1;
        else expectedRX = 0;

        if (expectedRX > 0) {
            uint8_t got = i2cMasterReadWithTimeout(expectedRX, rxWorking, REPLY_TIMEOUT_MS);
            if (got != expectedRX) {
              // read error/timeout
                stats_errors++;
                // do NOT re-enqueue; we drop/ignore reply
            }
            else {
           // check checksum: rxWorking.checksum expected at last byte
                uint8_t cs = calcChecksum(rxWorking, 2 + rxWorking.len);
                if (cs != rxWorking.checksum) {
                    stats_errors++;
                }
                else {
               // process reply packet (example)
                    switch (rxWorking.id) {
                        case MSG_PING:
                          // ack handled
                            break;
                        case MSG_GET_SETTINGS:
                          // handle status
                            break;
                        default:
                            break;
                    }
                }
            }
        }
        // after receive (or no expected), go back to idle
        state = MS_IDLE;
        nextActionMillis = millis() + CYCLE_MS;
    }
}




