/***********************************************************************************************************
*   Wire slave library for Arduino nano esp32                                                              *
*                                                                                                          *
*   Copyright 2026 Herwig Taveirne                                                                         *
*                                                                                                          *
*   The Wire slave library is free software: you can redistribute it and/or modify it under                *
*   the terms of the GNU General Public License as published by the Free Software Foundation, either       *
*   version 3 of the License, or (at your option) any later version.                                       *
*                                                                                                          *
*   This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;              *
*   without even the implied warranty of  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.             *
*   See the GNU General Public License for more details.                                                   *
*                                                                                                          *
*   You should have received a copy of the GNU General Public License along with this program. If not,     *
*   see https://www.gnu.org/licenses.                                                                      *
*                                                                                                          *
*   See GitHub for more information and documentation: https://github.com/Herwig9820/spinning_globe_wifi   *
*                                                                                                          *
***********************************************************************************************************/

#include <Wire.h>
#include "wireSlave_transport.h"

WireSlave* WireSlave::instance = nullptr;

WireSlave::WireSlave() {
    instance = this;
    Wire.begin(9);
    Wire.onReceive(wireReceiveEvent);
    Wire.onRequest(wireRequestEvent);
}

WireSlave::~WireSlave() {
};

void WireSlave::wireReceiveEvent(int byteCount) {
    if (instance) instance->pushIncomingWireMsg(byteCount);
}

void WireSlave::wireRequestEvent() {
    if (instance) instance->popOutgoingWireMsg();
}

// ========== enqueue message out to wire master: producer ==========

bool WireSlave::pushOutgoingWireMsg(uint8_t messageType, void* payload, uint8_t payloadSize) {
    // Lockstep SPSC with acquire / release fences

    bool empty = txEmpty;                   // atomic read (8 bit)

    if (!empty) {
        wireSlaveCommStats.E_stats_tx_full++;   // single writer
        return false;
    }

    ////Serial.print("sending: ");

    txQueue[0] = messageType;               // struct copy
    txQueue[1] = payloadSize;
    uint8_t sum = messageType ^ payloadSize;
    for (uint8_t i = 0; i < payloadSize; ++i) {
        txQueue[HEADER_SIZE + i] = ((uint8_t*)payload)[i];
        ////Serial.print(txQueue[HEADER_SIZE + i], HEX); Serial.print(' ');
        sum ^= ((uint8_t*)payload)[i];
    }
    txQueue[HEADER_SIZE + payloadSize] = sum;
    ////Serial.print("checksum OUT : "); Serial.println(txQueue[HEADER_SIZE + payloadSize], HEX);


    RELEASE_BARRIER();                      // Ensure packet bytes are visible BEFORE publishing

    txEmpty = false;
    return true;
}


// ========== dequeue message in from wire master: consumer ==========

bool WireSlave::popIncomingWireMsg(uint8_t& messageType, void* payload, uint8_t& payloadSize) {

    // Lockstep SPSC with acquire / release fences

    bool empty = rxEmpty;                   // atomic read

    if (empty) { return false; }            // queue empty

    ACQUIRE_BARRIER();                      // Ensure we see fully published packet

    messageType = rxQueue[0];
    payloadSize = rxQueue[1];
    // Make sure payloadSize is sane before memcpy
    if (payloadSize > MASTER_PAYLOAD_MAX) payloadSize = 0;
    if (payloadSize > 0) {
        memcpy((uint8_t*)payload, ((uint8_t*)(rxQueue)+HEADER_SIZE), payloadSize);
    }

    rxEmpty = true;
    return true;
}


// ========== GET slave send and receive stats ==========

void WireSlave::getCommStats(I2C_wireSlaveCommStats& commStatSnapshot) {

    portENTER_CRITICAL(&wireMux);
    commStatSnapshot = const_cast<const I2C_wireSlaveCommStats&>(wireSlaveCommStats);
    portEXIT_CRITICAL(&wireMux);
}


// ========== ISR: handle (enqueue) message in from wire master: producer ==========

void WireSlave::pushIncomingWireMsg(int byteCount) {

    //---------- 1. Test for valid packet lengths and for WireTransport::M_CTRL_POLL control messages ---------- 

    if (byteCount == 1) {                                           // receive one byte only ? must be a WireTransport::M_CTRL_POLL control message 
        if (Wire.read() == (uint8_t)WireTransport::M_CTRL_POLL) {
            waitState = SS_WAIT_FOR_POLL_REPLY_REQUEST;             // new state: wait for request (from master) to send poll reply ctrl message            
        }
        else {
            while (Wire.available()) { Wire.read(); }               // not a M_CTRL_POLL message: invalid message
        }
        return;
    }

    if (byteCount <= 0 || byteCount > PACKET_IN_MAX) {              // within message boundaries ? 
        while (Wire.available()) { Wire.read(); }
        return;
    }


    // ---------- 2. Get the Wire data and store in a temporary buffer (do this fast - test data later) ----------

    static uint8_t tmp[PACKET_IN_MAX];
    uint8_t sum = 0;
    for (size_t i = 0; i < (size_t)byteCount - 1; i++) {
        tmp[i] = Wire.read();
        sum ^= tmp[i];
    }
    tmp[byteCount - 1] = Wire.read();           // received checksum

    if (tmp[byteCount - 1] != sum) {
        // increment stats_errors atomically
        wireSlaveCommStats.E_stats_rx_checksum++;
        return;                                 // invalid message
    }

    // ---------- 3. Lockstep SPSC with acquire / release fences ----------

    bool empty = rxEmpty;
    if (!empty) {                               // Buffer full -> drop packet
        wireSlaveCommStats.E_stats_rx_full++;
        return;
    }

    for (int i = 0; i < byteCount; i++) { rxQueue[i] = tmp[i]; }

    wireSlaveCommStats.I_stats_received++;

    RELEASE_BARRIER();                          // Ensure packet bytes are visible BEFORE publishing

    rxEmpty = false;                            // publish 'buffer full' state
}


// ========== ISR: handle (dequeue) message out to wire master: consumer ==========

void WireSlave::popOutgoingWireMsg() {

    // Lockstep SPSC with acquire / release fences

    bool empty = txEmpty;

    // ---------- master is requesting buffer status ? Answer 'ready' as soon as buffer is full
    if (waitState == SS_WAIT_FOR_POLL_REPLY_REQUEST) {
        uint8_t reply = (empty ? (uint8_t)WireTransport::S_CTRL_BUSY : (uint8_t)WireTransport::S_CTRL_READY);
        Wire.write(reply);
        waitState = SS_WAIT_FOR_REPLY_REQUEST;
        return;
    }

    if (empty) { return; }                      // buffer is empty

    ACQUIRE_BARRIER();                          // Ensure we see fully published packet

    uint8_t msgSize = txQueue[1] + HEADER_SIZE + 1;
    Wire.write((uint8_t*)(txQueue), msgSize);

    wireSlaveCommStats.I_stats_sent++;

    txEmpty = true;

    return;
}















