
/* ========== PROTOCOL CONTRACT ========== */
/* ========== THIS IS A COMMON HEADER BETWEEN WIRE MASTER AND WIRE SLAVE ========== */

#ifndef _WIRE_COMMON_h
#define _WIRE_COMMON_h

#include "wireMaster.h"

#define IS_WIRE_MASTER 1     // wire master 



/*
*   spinning globe WiFi / MQTT extension with Arduino nano esp32
*   ------------------------------------------------------------
*   definitions, declarations common to the wire master (classic nano) and wire slave (nano ESP32)
*/

enum MsgType : uint8_t {
    // ========== 0x00-0x1F RESERVED for CONTROL SIGNALS between master and slave libraries ==========


    // ========== 0x20–0x3F: master → slave (bit b2 clear) ==========

    M_MSG_NONE = 0x20,
    M_MSG_PING = 0x21,                  // simple ping message (no payload)  

    // message types to send status data to slave. Slave reply: ACK message type
    M_MSG_GREENWICH = 0x22,             // globe Greenwich meridian passes hall detector
    M_MSG_STATUS = 0x23,                // globe status changes
    M_MSG_SECOND = 0x24,                // second tick

    // message types to send master comm stats to slave. Slave reply: ACK message type
    M_MSG_SEND_STATS = 0x25,            // master send stats
    M_MSG_RECEIVE_STATS = 0x26,         // master slave stats

    // message types to send current master settings to slave. Slave reply: ACK message type
    M_MSG_USER_SETTINGS = 0x28,         // rotation time, ...
    M_MSG_PID_SETTINGS = 0x29,          // PID controller gain, time constants
    M_MSG_VERT_POS_SETPOINT = 0x2A,     // PID setpoint (vertical position)
    M_MSG_COIL_PHASE_ADJUST = 0x2B,     // globe rotation controller: phase adjust between coils magnetic field and globe Greenwich meridian

    // master requests slave to send data. Slave reply: message types with the requested slave data
    M_MSG_REQ_USER_SETTINGS = 0x30,     // master requests settings as maintained by slave
    M_MSG_REQ_PID_SETTINGS = 0x31,
    M_MSG_REQ_VERT_POS_SETPOINT = 0x32,
    M_MSG_REQ_COIL_PHASE_ADJUST = 0x33,


    // ========== 0x40–0x5F: not used ==========


    // ========== 0x60–0x7F: slave → master  (bit b2 set) ==========

    S_MSG_NONE = 0x60,
    S_MSG_PING = 0x61,

    // message type to reply to messages not requiring a specific message type as answer.
    // the slave can request the master to send specific data, by setting PAYLOAD field 'requestMasterMsgType'...
    // ... to the MASTER message type it would like to receive. Allowed message types:
    // M_MSG_SEND_STATS, M_MSG_RECEIVE_STATS, M_MSG_USER_SETTINGS, M_MSG_PID_SETTINGS, M_MSG_VERT_POS_SETPOINT, M_MSG_COIL_PHASE_ADJUST
    S_MSG_ACK = 0x62,

    // slave sends data specifically requested by master
    S_MSG_USER_SETTINGS = 0x70,
    S_MSG_PID_SETTINGS = 0x71,
    S_MSG_VERT_POS_SETPOINT = 0x72,
    S_MSG_COIL_PHASE_ADJUST = 0x73

    // ========== 0x80–0xFF: not used ==========

};


// ========== I2C messages: common messages exchanged between master and slave; in two directions ==========

struct __attribute__((packed)) I2C_userSettings {
    uint8_t userSet_rotationPeriod;                 // index into array with defined rotation periods 
    uint8_t userSet_ledEffect;
    uint8_t userSet_ledCycleSpeed;
};

struct __attribute__((packed)) I2C_PIDsettings {
    int8_t gainAdjustSteps;                         // positive or negative
    int8_t intTimeCstAdjustSteps;
    int8_t difTimeCstAdjustSteps;
};

struct __attribute__((packed)) I2C_vertPosSetpoint {
    uint8_t userSet_vertPosIndex = 0;               // index into array with vert. positions (in mV)
};

struct __attribute__((packed)) I2C_coilPhaseAdjust {
    uint8_t userSet_coilPhaseAdjust = 0;               // index into array with vert. positions (in mV)
};



// ========== I2C messages from master to slave ==========

struct __attribute__((packed)) I2C_m_status {
    uint8_t status;
};

struct __attribute__((packed)) I2C_m_greenwich {
    uint32_t actualRotationTime;
    int32_t rotationOutOfSyncTime;      // can be negative
    int32_t greenwichLag;               // can be negative
};

struct __attribute__((packed)) I2C_m_secondCue {
    uint32_t tempSmooth;
    float magnetOnCyclesSmooth;
    float ISRdurationSmooth;
    float idleLoopMicrosSmooth;
    float errSignalMagnitudeSmooth;
};

using I2C_m_userSettings = I2C_userSettings;
using I2C_m_PIDsettings = I2C_PIDsettings;
using I2C_m_coilPhaseAdjust = I2C_coilPhaseAdjust;
using I2C_m_vertPosSetpoint = I2C_vertPosSetpoint;

#if IS_WIRE_MASTER
struct __attribute__((packed)) I2C_m_sendStats {               // '_m_': message from >m<aster to slave
    WireMaster::I2C_MasterSendStats sendStats;
};

struct __attribute__((packed)) I2C_m_receiveStats {              // '_m_': message from >m<aster to slave
    WireMaster::I2C_masterReceiveStats receiveStats;
};

#else
struct __attribute__((packed)) I2C_m_MasterSendStats {                // '_m_': message from >m<aster to slave
    uint32_t I_stats_sent;
    uint32_t W_stats_tx_retrying;
    uint32_t E_stats_tx_wireXmitError;
    uint32_t E_stats_tx_full;
};

struct __attribute__((packed)) I2C_m_masterReceiveStats {             // '_m_': message from >m<aster to slave
    uint32_t I_stats_received;
    uint32_t E_stats_rx_checksum;
    uint32_t E_stats_rx_timeOut;
    uint32_t E_stats_rx_full;
};
#endif


// ========== I2C messages from slave to master ==========

struct __attribute__((packed)) I2C_s_ack {
    uint8_t ack;                                    // currently not used 
    uint8_t requestMasterMsgType;                   // slave requests master to send message type
};

using I2C_s_userSettings = I2C_userSettings;
using I2C_s_PIDsettings = I2C_PIDsettings;
using I2C_s_coilPhaseAdjust = I2C_coilPhaseAdjust;
using I2C_s_vertPosSetpoint = I2C_vertPosSetpoint;

/* ========== END OF COMMON SECTION BETWEEN WIRE MASTER AND WIRE SLAVE ========== */

#endif