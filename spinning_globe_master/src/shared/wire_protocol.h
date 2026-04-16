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
I2C MASTER–SLAVE DESIGN CONTRACT (Lockstep, Robust, Minimal)
===============================================================================

Scope
-----
This contract defines the behavioral guarantees and responsibilities between
an I2C master and a single I2C slave. It is intentionally simple, deterministic,
and suitable for small MCUs (AVR, ESP32).

Core Principle
--------------
WIRE PROTOCOL: STRICT LOCKSTEP REQUEST–RESPONSE.
Master application interface: buffered, jitter-tolerant

• At most ONE outstanding master request (sent from master tx queue) exists at any time.
• Each master request results in EXACTLY ONE slave reply (received in master rx queue).
• The master MUST receive (or timeout) the reply before sending the next request.
• The slave NEVER sends unsolicited data.

This guarantees:
• No queue buildup
• No message reordering
• No buffer overwrite
• No flow-control ambiguity

-------------------------------------------------------------------------------

Message Structure
-----------------
All protocol messages (except control messages) have:

    [0] Message Type (uint8_t)
    [1] Payload Length (uint8_t)
    [2..N] Payload
    [N+1] Checksum (XOR over all previous bytes)

Control messages (POLL/BUSY/READY) are ONE BYTE ONLY.

-------------------------------------------------------------------------------

Opcode Ranges
-------------
Opcode ranges are frozen and MUST NOT overlap.

0x00 – 0x1F : Reserved for internal control (library-only)
    - M_CTRL_POLL       master polls slave for 'reply ready'
    - S_CTRL_BUSY       slave response: reply not yet ready
    - S_CTRL_READY      slave response: reply ready

0x20 – 0x3F : Master → Slave application messages (b2 clear; M_*)

0x40 - 0x5F : not used

0x60 – 0x7F : Slave → Master application replies (b2 set; S_*)

0x80 - 0xFE : not used

0xFF        : MSG_ERROR (reserved placeholder)

-------------------------------------------------------------------------------

Master Responsibilities
-----------------------
1. The master MAY enqueue outgoing messages locally (optional).
2. The master SHALL send a new message ONLY when:
       - No previous message is outstanding
       - OR the previous transaction ended with a terminal error
3. If a reply is expected:
       - Wait at least MIN_CYCLE_TIME
       - Poll the slave using M_CTRL_POLL
       - On S_CTRL_READY, read exactly ONE reply message
4. The master SHALL enforce:
       - Minimum cycle time
       - Maximum receive timeout
       - Retry limits on transmit errors
5. A message is removed from the TX buffer ONLY AFTER:
       - A valid reply is received
       - OR retries/timeouts are exhausted

-------------------------------------------------------------------------------

Slave Responsibilities
----------------------
1. The slave SHALL process at most ONE request at a time.
2. Upon receiving a master message:
       - Validate checksum and size
       - Prepare exactly ONE reply message
3. Until the reply is ready:
       - Respond to POLL with S_CTRL_BUSY
4. When the reply is ready:
       - Respond to POLL with S_CTRL_READY
       - Send the reply ONCE upon request
5. After sending the reply:
       - Return to idle state
       - Await next master request

The slave MUST NOT:
• Send unsolicited messages
• Queue multiple outgoing replies
• Overwrite a reply before it is sent

-------------------------------------------------------------------------------

Buffers and Concurrency
-----------------------
• Wire slave: effective queue depth is ONE (single-slot buffers).
• No simultaneous read/write occurs on the same buffer.
• This design remains SPSC by construction.
• Memory barriers are used only at publish/consume boundaries.

-------------------------------------------------------------------------------

Error Handling
--------------
• Checksum error → message dropped, no reply published
• Size mismatch → message dropped
• Timeout → transaction aborted
• Errors are TERMINAL for the current transaction
• Recovery is always initiated by the master

-------------------------------------------------------------------------------

Slave-Initiated Data
--------------------
If the slave has data to send:
• It advertises this in the payload of its NEXT reply
• The payload contains the REQUIRED master request opcode
• The master explicitly requests that data in the next transaction

No spontaneous slave transmission is permitted.

-------------------------------------------------------------------------------

Design Intent
-------------
This protocol favors:
• Determinism over throughput
• Explicit control over implicit buffering
• Simplicity over speculative optimization

This contract is NOT intended to support:
• Multiple outstanding transactions
• Broadcast or multi-slave arbitration
• Automatic retransmission of application-level data

===============================================================================
Global protocol invariants (master + slave)
===============================================================================
Strict lockstep on the wire
- At most one application-level request is in flight at any time.
- A new request SHALL NOT be sent until the previous request has completed with:
    - a valid reply, or
    - a terminal error (timeout or retry exhaustion).

One request -> at most one reply
- The slave produces at most one reply per valid master request.
- Replies are never unsolicited.

Slave speaks only when asked
- The slave transmits data only during a master-initiated read (onRequest()).

Protocol-level determinism
- For a given request type, the expected reply type and length are known in advance.

===============================================================================
*/

#ifndef FG_WIRE_PROTOCOL_h
#define FG_WIRE_PROTOCOL_h


/*
===============================================================================
Declarations common to the wire master (classic nano) and wire slave (nano ESP32)
===============================================================================
*/

#include <stdint.h> 

// Packet (message) sizing 
static constexpr uint8_t HEADER_SIZE = 3;
static constexpr uint8_t SLAVE_PAYLOAD_MAX = 24;                   // max. payload sizes in bytes 
static constexpr uint8_t MASTER_PAYLOAD_MAX = 24;


// !!!!!!!!!! 0x00 - 0x1f RESERVED for CONTROL SIGNALS between master and slave libraries !!!!!!!!!!
enum class WireTransport :uint8_t {
    NONE = 0x00,
    M_CTRL_POLL = 0x01,                                         // one-byte message; slave reply msg type: S_CTRL_BUSY or S_CTRL_READY
    S_CTRL_BUSY,                                                // one-byte reply: ONLY allowed in response to M_MSG_POLL_SLAVE
    S_CTRL_READY                                                // idem
};


enum MsgType : uint8_t {
    // 0x00 - 0x1f RESERVED for CONTROL SIGNALS between master and slave libraries


    // ========== 0x20–0x3F: master → slave (bit b2 clear) ==========

    M_MSG_NONE = 0x20,
    M_MSG_HELLO = 0x21,                 // simple hello message  (reserved, currently not implemented)  
    M_MSG_PING = 0x22,                  // no payload, but expects ack msg back

    // message types to send globe status data to slave. Slave reply: ACK message type
    M_MSG_GREENWICH = 0x23,             // globe Greenwich meridian passes hall detector
    M_MSG_STATUS = 0x24,                // globe status changes
    M_MSG_TELEMETRY = 0x25,             // heatsink temp., magnet duty cycle, ...
    M_MSG_TELEMETRY_EXTRA = 0x26,       // number of minutes the globe is floating

    // message types to send master comm stats to slave. Slave reply: ACK message type
    M_MSG_SEND_STATS = 0x28,            // wire transport send stats
    M_MSG_RECEIVE_STATS = 0x29,         // wire transport receive stats
    M_MSG_MESSAGE_STATS = 0x2A,         // wire message stats

    // message types to send current master settings to slave. Slave reply: ACK message type
    M_MSG_GLOBE_SETTINGS = 0x2C,        // rotation time, ...
    M_MSG_PID_SETTINGS = 0x2D,          // PID controller gain, time constants
    M_MSG_VERT_POS_SETPOINT = 0x2E,     // PID setpoint (vertical position)
    M_MSG_COIL_PHASE_ADJUST = 0x2F,     // globe rotation controller: phase adjust between coils magnetic field and globe Greenwich meridian

    // master requests slave to send data. Slave reply: message types with the requested slave data
    M_MSG_GLOBE_SETTINGS_REQ = 0x30,    // master requests globe settings as maintained / known by slave
    M_MSG_PID_SETTINGS_REQ = 0x31,      // master requests PID settings as maintained / known by slave
    M_MSG_VERT_POS_SETPOINT_REQ = 0x32, // master requests vertical position setpoint as maintained / known by slave
    M_MSG_COIL_PHASE_ADJUST_REQ = 0x33, // master requests coil phase adjustment as maintained / known by slave

    // ========== 0x40–0x5F: not used ==========


    // ========== 0x60–0x7F: slave → master  (bit b2 set) ==========

    S_MSG_NONE = 0x60,
    S_MSG_HELLO_ACK = 0x61,             // reply to master hello message (reserved, currently not implemented)

    // most messages types sent by the master simply require a reply msg type acknowledging that the message was received.
    S_MSG_ACK = 0x62,

    // the following slave messages types are only sent in response to specific master message types
    S_MSG_GLOBE_SETTINGS_SET = 0x70,        // sending globe settings to master:             response to msg type M_MSG_GLOBE_SETTINGS_REQ
    S_MSG_PID_SETTINGS_SET = 0x71,          // sending PID settings to master:               response to msg type M_MSG_PID_SETTINGS_REQ 
    S_MSG_VERT_POS_SETPOINT_SET = 0x72,     // sending vertical position setpoint to master: response to msg type M_MSG_VERT_POS_SETPOINT_REQ
    S_MSG_COIL_PHASE_ADJUST_SET = 0x73,     // sending coil phase adjustment to master:      response to msg type M_MSG_COIL_PHASE_ADJUST_REQ  


    // ========== 0x80–0xFE: not used ==========


    // ========== 0xFF: reserved placeholder ==========

    MSG_ERROR = 0xFF
};


enum Action :uint8_t {
    M_ACTION_NONE,
    M_ACTION_START_RING,
    M_ACTION_STOP_RING,                         // or stop alarm
    M_ACTION_START_ALARM,                       // alarm can also be stopped by action 'M_ACTION_STOP_RING'
    M_ACTION_HEARTBEAT                          // no payload
};


// ========== I2C message payloads: common messages exchanged between master and slave; in two directions ==========

struct __attribute__((packed)) I2C_globeSettings {
    uint8_t rotationPeriodIndex;                // index into array with defined rotation periods 
    uint8_t ledEffect;                          // led effect number
    uint8_t ledCycleSpeed;                      // led effect speed number
    uint8_t slaveHasData{ 0 };                  // slave=>master only: the slave data the master requested is available
};

struct __attribute__((packed)) I2C_PIDsettings {
    uint8_t gainAdjustSteps;                    // 0 (minimum) to settingSteps (maximum)
    uint8_t intTimeCstAdjustSteps;
    uint8_t difTimeCstAdjustSteps;
    uint8_t slaveHasData{ 0 };                  // slave=>master only: the slave data the master requested is available
};

struct __attribute__((packed)) I2C_vertPosSetpoint {
    uint8_t vertPosIndex;                       // index into array with vert. positions (in mV)
    uint8_t slaveHasData{ 0 };                  // slave=>master only: the slave data the master requested is available
};

struct __attribute__((packed)) I2C_coilPhaseAdjust {
    uint8_t coilPhaseAdjust;                    // coil phase in 2-degrees increments (0: 0 degrees, 89: 178 degrees, 90: -180 degrees, 179: -2 degrees   
    uint8_t slaveHasData{ 0 };                  // slave=>master only: the slave data the master requested is available
};

struct __attribute__((packed)) I2C_buttonStates {
    uint8_t buttonStates;
    uint8_t slaveHasData{ 0 };
};


// ========== I2C message payloads: messages from master to slave ==========

struct __attribute__((packed)) I2C_m_status {
    uint8_t status;
    uint8_t stateFlags;
};

struct __attribute__((packed)) I2C_m_greenwich {
    uint32_t actualRotationTime;
    int32_t rotationOutOfSyncTime;      // can be negative
    int32_t greenwichLag;               // can be negative
    uint8_t status;
};

struct __attribute__((packed)) I2C_m_telemetry {
    int32_t tempSmooth;
    float magnetOnCyclesSmooth;
    float ISRdurationSmooth;
    float idleLoopMicrosSmooth;
    float errSignalMagnitudeSmooth;
    int32_t realTTTintegrationTerm;
};

struct __attribute__((packed)) I2C_m_telemetry_extra {
    float secondsFloating;
    uint8_t currentMaxEventsPending{ 0 };           // No of ISR events currently logged for processing in main loop 
    uint16_t largestEventBufferBytesUsed{ 0 };      // No of ISR events logged for processing in main loop: all-time max
    uint32_t eventsMissed{ 0 };                     // keeps track of events missed (not used at this stage)    
};

// ensure consistency between master and slave
using I2C_m_globeSettings = I2C_globeSettings;      // '_m_': data flows from wire master to wire slave 
using I2C_m_PIDsettings = I2C_PIDsettings;
using I2C_m_coilPhaseAdjust = I2C_coilPhaseAdjust;
using I2C_m_vertPosSetpoint = I2C_vertPosSetpoint;
using I2C_m_buttonStates = I2C_buttonStates;


// ========== I2C message payloads: diagnostic messages from master to slave ==========

// message level counters
struct __attribute__((packed))I2C_m_messageStats {
    uint32_t I_stats_replyReceived{ 0 };
    uint32_t E_stats_lockStepError{ 0 };

    uint32_t inline errorMsgCount(){return  E_stats_lockStepError;}
    void inline zeroMembers() { I_stats_replyReceived = E_stats_lockStepError = 0; }
};

// transport level counters
struct I2C_m_masterSendStats {
    uint32_t I_stats_tx_sent = 0;                               // info: counts     
    uint32_t W_stats_tx_retrying = 0;                           // warning: count 
    uint32_t E_stats_tx_wireXmitError = 0;
    uint32_t E_stats_tx_full = 0;                               // errors: counts

    uint32_t inline errorMsgCount(){return W_stats_tx_retrying + E_stats_tx_wireXmitError + E_stats_tx_full;}
    void inline zeroMembers() { I_stats_tx_sent = W_stats_tx_retrying = E_stats_tx_wireXmitError = E_stats_tx_full = 0; }
};

struct I2C_m_masterReceiveStats {
    uint32_t I_stats_rx_received = 0;
    uint32_t E_stats_rx_checksum = 0;
    uint32_t E_stats_rx_timeOut = 0;
    uint32_t E_stats_rx_full = 0;

    uint32_t inline errorMsgCount(){return E_stats_rx_checksum + E_stats_rx_timeOut + E_stats_rx_full;}
    void inline zeroMembers() { I_stats_rx_received = E_stats_rx_checksum = E_stats_rx_timeOut = E_stats_rx_full = 0; }
};


// ========== I2C message payloads: messages from slave to master ==========

struct __attribute__((packed)) I2C_s_ack {
    uint8_t ack{ 0x00 };                                    // currently not used 
    // the slave can request the master to SEND a specific message type to the slave. This can be used if the slave HAS INFO to share 
    // with the master, or it REQUESTS the master to send specific info
    Action action{ M_ACTION_NONE };            // action requested from master
    MsgType requestMasterMsgType{ M_MSG_NONE };   // slave requests master to send message type

};

// ensure consistency between master and slave
using I2C_s_globeSettings_set = I2C_globeSettings;  // '_s_': data flows from wire slave to wire master 
using I2C_s_PIDsettings_set = I2C_PIDsettings;
using I2C_s_coilPhaseAdjust_set = I2C_coilPhaseAdjust;
using I2C_s_vertPosSetpoint_set = I2C_vertPosSetpoint;
using I2C_s_buttonStates_set = I2C_buttonStates;

/* ========== END OF COMMON SECTION BETWEEN WIRE MASTER AND WIRE SLAVE ========== */

#endif