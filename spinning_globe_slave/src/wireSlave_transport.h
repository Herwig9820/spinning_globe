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

/*
===============================================================================
Wire Slave State Machine
===============================================================================

Current State	    Event	                    Action	        Next State
-------------       -----                       ------          ----------
IDLE	            Receive normal message	    Enqueue RX	    PROCESS
IDLE	            Receive POLL	            Respond BUSY	IDLE
PROCESS	            Application consumes RX	    Prepare reply	READY
READY	            Receive POLL	            Respond READY	WAIT_REPLY_REQUEST
WAIT_REPLY_REQUEST	onRequest()	                Send reply	    IDLE

===============================================================================
Slave-side design invariants
===============================================================================

Single logical RX and TX slot
- The slave processes at most one master request at a time.
- At most one reply is pending at any time.

Receive-before-reply
- A reply is prepared only after a full, valid request has been received and verified.

Reply persistence
- Once prepared, a reply remains stable until successfully sent to the master.

No overwrite without consumption
- RX or TX buffers are never overwritten until their contents have been consumed.

Key invariant:
- The slave sends exactly ONE reply per received request.

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
*/

#ifndef FG_SLAVE_TRANSPORT_h
#define FG_SLAVE_TRANSPORT_h

#include <arduino.h>
#include "shared/wire_protocol.h"

// Full memory fence for ESP32 / GCC
#define MEMORY_BARRIER() __sync_synchronize()

#define RELEASE_BARRIER() MEMORY_BARRIER()
#define ACQUIRE_BARRIER() MEMORY_BARRIER()

// SINGLE global spinlock used by both ISR and main code
static portMUX_TYPE wireMux = portMUX_INITIALIZER_UNLOCKED;


class  WireSlave {

    /* ================= constants ================= */

    static constexpr  uint8_t I2C_SLAVE_ADDR = 9;

    // Lockstep SPSC (single producer single consumer) single-slot buffer with time-phased reading & writing 
    static constexpr uint8_t RX_QUEUE_SIZE = 1;                                     
    static constexpr uint8_t TX_QUEUE_SIZE = 1;       

public:
    // Packet sizing
    static constexpr uint8_t HEADER_SIZE = 3;                                       // message type and length, (not currently used) version byte
    static constexpr uint8_t PACKET_IN_MAX = HEADER_SIZE + MASTER_PAYLOAD_MAX + 1;      // type + len + payload + checksum: length max. 32
    static constexpr uint8_t PACKET_OUT_MAX = HEADER_SIZE + SLAVE_PAYLOAD_MAX + 1;    // type + len + payload + checksum: length max. 32


/* ================= Lockstep SPSC with acquire / release fences ================= */

private:
    // SPSC buffers (single producer single consumer)
    alignas(4) volatile uint8_t rxQueue[HEADER_SIZE + MASTER_PAYLOAD_MAX + 1];          // type + len + payload + checksum
    alignas(4) volatile uint8_t txQueue[HEADER_SIZE + SLAVE_PAYLOAD_MAX + 1];         // type + len + payload + checksum

    alignas(4) volatile bool rxEmpty = true;
    alignas(4) volatile bool txEmpty = true;


    /* ================= STATE ================= */

    // !!!!!!!!!! 0x00 - 0x1f RESERVED for CONTROL SIGNALS between master and slave libraries !!!!!!!!!!
    enum class WireTransport {                  // control messages exchanged between slave and master
        NONE = 0x00,
        M_CTRL_POLL = 0x01,                     // one-byte message; slave reply msg type: S_CTRL_BUSY or S_CTRL_READY
        S_CTRL_BUSY,                            // one-byte reply: ONLY allowed in response to M_MSG_POLL_SLAVE
        S_CTRL_READY                            // idem
    };

    enum SlaveWaitState { SS_WAIT_FOR_POLL_REPLY_REQUEST, SS_WAIT_FOR_REPLY_REQUEST };

    struct I2C_wireSlaveCommStats {
        uint32_t I_stats_sent = 0;              // send: msg from tx queue -> wire out
        uint32_t I_stats_received = 0;          // receive: msg from wire in -> rx queue
        uint32_t E_stats_tx_full = 0;           // send: tx queue full (enqueue failed)
        uint32_t E_stats_rx_full = 0;           // receive: rx queue full (copy from wire failed)
        uint32_t E_stats_rx_checksum = 0;
    };


    /* ================= METHODS ================= */

    volatile SlaveWaitState waitState{ SS_WAIT_FOR_POLL_REPLY_REQUEST };
    volatile I2C_wireSlaveCommStats wireSlaveCommStats{};

    static void wireReceiveEvent(int byteCount);
    static void wireRequestEvent();

    void pushIncomingWireMsg(int byteCount);
    void popOutgoingWireMsg();

    static WireSlave* instance;   // global pointer to our our object


public:
    WireSlave();
    ~WireSlave();

    bool popIncomingWireMsg(uint8_t& msgType, void* payload, uint8_t& payloadSize);
    bool pushOutgoingWireMsg(uint8_t messageType, void* payload, uint8_t payloadSize);
    void getCommStats(I2C_wireSlaveCommStats& commStatSnapshot);
};

#endif
