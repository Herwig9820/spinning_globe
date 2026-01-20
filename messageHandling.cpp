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
An Arduino nano esp32, acting as wire slave, will control the spinning globe (change settings, check states)
over WiFi, e.g. using MQTT.

Note that, if the program is compiled with this option enabled, hardware buttons and LCD (connector SV2)...
...will be inoperable (switches are still functioning). USB terminal is not used except for a welcome message.

===============================================================================================
*/


/*
===============================================================================
Wire master: message handling layer
===============================================================================
*/

#include "floatingGlobeState.h"
#include "messageHandling.h"

MessageHandling::MessageHandling(GreenwichData& greenwichData, StatusData& statusData, SecondData& secondData,
    SmoothedMeasurements& smoothedMeasurements, PIDsettings& pidSettings, int* pGobeAttributes,
    LedStripSettings& ledStripSettings, EventData& globeEventSnapshot) :
    _greenwichData(greenwichData), _statusData(statusData), _secondData(secondData),
    _smoothedMeasurements(smoothedMeasurements), _pidSettings(pidSettings), _pGlobeAttributes(pGobeAttributes),
    _ledStripSettings(ledStripSettings), _globeEventSnapshot(globeEventSnapshot) {
};

MessageHandling::~MessageHandling() {};

uint8_t MessageHandling::transmit() {
    return (uint8_t)_wireMaster.sendAndReceiveMessage();                                // return master or slave message in error
}

// ========== enqueue data to send buffer ==========

void MessageHandling::enqueueI2CmessageToSlave(uint8_t& msgTypeOut) {

    if (msgTypeOut == MsgType::M_MSG_NONE) { return; }

    // note: if tx buffer full, message is silently dropped but error counter is incremented

    switch (msgTypeOut) {

        // ========== message type requesting a hello acknowledge from slave ==========

        case MsgType::M_MSG_HELLO:
        {
            _wireMaster.enqueueTx(M_MSG_HELLO, 0, nullptr, S_MSG_HELLO_ACK, 0);         // no payload
        }
        break;


        // ========== message types SENDING DATA to slave after a globe event ==========

        case  MsgType::M_MSG_SECOND:
        {
            I2C_m_secondCue p{};
            p.tempSmooth = _smoothedMeasurements.tempSmooth;                            // raw input for wire slave
            p.magnetOnCyclesSmooth = _smoothedMeasurements.magnetOnCyclesSmooth;
            p.ISRdurationSmooth = _smoothedMeasurements.ISRdurationSmooth;
            p.idleLoopMicrosSmooth = _smoothedMeasurements.idleLoopMicrosSmooth;
            p.errSignalMagnitudeSmooth = _smoothedMeasurements.errSignalMagnitudeSmooth;
            _wireMaster.enqueueTx(M_MSG_SECOND, sizeof(p), &p, S_MSG_ACK, sizeof(I2C_s_ack));
        }
        break;

        case  MsgType::M_MSG_GREENWICH:
        {
            I2C_m_greenwich p{};
            p.actualRotationTime = _greenwichData.globeRotationTime;
            p.rotationOutOfSyncTime = _greenwichData.rotationOutOfSyncTime;
            p.greenwichLag = _greenwichData.greenwichLag;
            _wireMaster.enqueueTx(M_MSG_GREENWICH, sizeof(p), &p, S_MSG_ACK, sizeof(I2C_s_ack));
        }
        break;

        case MsgType::M_MSG_STATUS:
        {
            I2C_m_status p{};
            p.status = (int8_t)((_statusData.errorCondition == errNoError) ? _statusData.rotationStatus : (_statusData.errorCondition | 0x10));
            _wireMaster.enqueueTx(M_MSG_STATUS, sizeof(p), &p, S_MSG_ACK, sizeof(I2C_s_ack));
        }
        break;

        // ========== message types SENDING DATA to slave when requested ==========

        // send wire master library send stats to wire slave
        case MsgType::M_MSG_SEND_STATS:
        {
            I2C_m_masterSendStats p{};
            _wireMaster.getSendStats(p);
            _wireMaster.enqueueTx(M_MSG_SEND_STATS, sizeof(p), &p, S_MSG_ACK, sizeof(I2C_s_ack));
        }
        break;

        // send wire master library receive stats to wire slave
        case MsgType::M_MSG_RECEIVE_STATS:
        {
            I2C_m_masterReceiveStats p{};
            _wireMaster.getReceiveStats(p);
            _wireMaster.enqueueTx(M_MSG_RECEIVE_STATS, sizeof(p), &p, S_MSG_ACK, sizeof(I2C_s_ack));
        }
        break;

        // send message library stats to wire slave
        case MsgType::M_MSG_MESSAGE_STATS:
        {
            // note: _msgStats is already up to date 
            _wireMaster.enqueueTx(M_MSG_MESSAGE_STATS, sizeof(_msgStats), &_msgStats, S_MSG_ACK, sizeof(I2C_s_ack));
        }
        break;

        // send spinning globe stats to wire slave
        case MsgType::M_MSG_GLOBE_STATS:
        {
            I2C_m_GlobeStats p{};
            p.eventsMissed = _globeEventSnapshot.eventsMissed;
            p.largestEventsPending = _globeEventSnapshot.largestEventsPending;
            p.largestEventBufferBytesUsed = _globeEventSnapshot.largestEventBufferBytesUsed;
            _wireMaster.enqueueTx(M_MSG_GLOBE_STATS, sizeof(_globeEventSnapshot), &_globeEventSnapshot, S_MSG_ACK, sizeof(I2C_s_ack));
        }
        break;


        // send user settings to wire slave
        case MsgType::M_MSG_GLOBE_SETTINGS:
        {
            I2C_m_globeSettings p{};
            // set rotation time is stored in globe attributes array, not in a struct
            p.userSet_rotationPeriod = _pGlobeAttributes[attributeIndex_rotTimes];    // (set, not actual)
            p.userSet_ledEffect = _ledStripSettings.ledEffect;
            p.userSet_ledCycleSpeed = _ledStripSettings.ledCycleSpeed;
            _wireMaster.enqueueTx(M_MSG_GLOBE_SETTINGS, sizeof(p), &p, S_MSG_ACK, sizeof(I2C_s_ack));

            Serial.println(F("Globe settings (sent):"));
            Serial.print(F("    rotation period ")); Serial.println(p.userSet_rotationPeriod);
            Serial.print(F("    led effect      ")); Serial.println(p.userSet_ledEffect);
            Serial.print(F("    led cycle speed ")); Serial.println(p.userSet_ledCycleSpeed);
        }
        break;

        // send PID settings to wire slave
        case MsgType::M_MSG_PID_SETTINGS:
        {
            I2C_m_PIDsettings p{};
            p.gainAdjustSteps = _pidSettings.gainAdjustSteps;
            p.intTimeCstAdjustSteps = _pidSettings.intTimeCstAdjustSteps;
            p.difTimeCstAdjustSteps = _pidSettings.difTimeCstAdjustSteps;
            _wireMaster.enqueueTx(M_MSG_PID_SETTINGS, sizeof(p), &p, S_MSG_ACK, sizeof(I2C_s_ack));

            Serial.println(F("PID controller adjust steps (sent):"));
            Serial.print(F("    gain          ")); Serial.println(p.gainAdjustSteps);
            Serial.print(F("    int. time cst ")); Serial.println(p.intTimeCstAdjustSteps);
            Serial.print(F("    dif. time cst ")); Serial.println(p.difTimeCstAdjustSteps);
        }
        break;


        // send globe vertical position setpoint to wire slave
        case MsgType::M_MSG_VERT_POS_SETPOINT:
        {
            I2C_m_vertPosSetpoint p{};
            // vertical position (in mVolt) is stored in the globe attributes array, not in a struct
            p.userSet_vertPosIndex = _pGlobeAttributes[attributeIndex_hallmVoltRefs];
            _wireMaster.enqueueTx(M_MSG_VERT_POS_SETPOINT, sizeof(p), &p, S_MSG_ACK, sizeof(I2C_s_ack));
        }
        break;

        // send coil phase adjustment setting to wire slave
        case MsgType::M_MSG_COIL_PHASE_ADJUST:
        {
            I2C_m_coilPhaseAdjust p{};
            // set coil phase adjust is stored in the globe attributes array, not in a struct
            p.userSet_coilPhaseAdjust = _pGlobeAttributes[attributeIndex_coilPhaseAdjust];
            _wireMaster.enqueueTx(M_MSG_COIL_PHASE_ADJUST, sizeof(p), &p, S_MSG_ACK, sizeof(I2C_s_ack));
        }
        break;


        // ========== message types REQUESTING DATA from slave ==========

        case MsgType::M_MSG_REQ_GLOBE_SETTINGS:
        {
            _wireMaster.enqueueTx(M_MSG_REQ_GLOBE_SETTINGS, 0, nullptr, S_MSG_GLOBE_SETTINGS, sizeof(I2C_s_globeSettings));
        }
        break;

        case MsgType::M_MSG_REQ_PID_SETTINGS:
        {
            _wireMaster.enqueueTx(M_MSG_REQ_PID_SETTINGS, 0, nullptr, S_MSG_PID_SETTINGS, sizeof(I2C_s_PIDsettings));
        }
        break;

        case MsgType::M_MSG_REQ_VERT_POS_SETPOINT:
        {
            _wireMaster.enqueueTx(M_MSG_REQ_VERT_POS_SETPOINT, 0, nullptr, S_MSG_VERT_POS_SETPOINT, sizeof(I2C_s_vertPosSetpoint));
        }
        break;

        case MsgType::M_MSG_REQ_COIL_PHASE_ADJUST:
        {
            _wireMaster.enqueueTx(M_MSG_REQ_COIL_PHASE_ADJUST, 0, nullptr, S_MSG_COIL_PHASE_ADJUST, sizeof(I2C_s_coilPhaseAdjust));
        }
        break;
    }
    msgTypeOut = MsgType::M_MSG_NONE;
    return;
}


// ========== process Wire slave reply ==========

void MessageHandling::dequeueI2CmessageFromSlave(uint8_t& nextMsgTypeOut) {
    uint8_t msgTypeIn{ MsgType::M_MSG_NONE };                   // message type received from slave
    uint8_t i2cPayloadSizeIn{ 0 };                              // payload size as reported by slave
    uint8_t plIn[WireMaster::PAYLOAD_IN_MAX];
    uint8_t expMsgType{};

    // message available  ?
    bool msgAvailable = _wireMaster.dequeueRx(msgTypeIn, i2cPayloadSizeIn, &plIn, expMsgType);
    if (!msgAvailable) {
        nextMsgTypeOut = M_MSG_NONE;
        return;
    }

    // this is not the expected message type (strict lockStep implemented)
    // (note that, if the message type is correct, then the message size is as well (handled in WireMaster library) 
    if (msgTypeIn != expMsgType) {
        _msgStats.E_stats_lockStepError++;
        return;
    }

    switch (msgTypeIn) {

        // answer to hello message
        case S_MSG_HELLO_ACK:
        {
            nextMsgTypeOut = MsgType::M_MSG_NONE;
        }
        break;

        // receive message acknowledge ('slave received message')
        case S_MSG_ACK:
        {
            I2C_s_ack* p = reinterpret_cast<I2C_s_ack*>(plIn);
            // next requested message type     
            nextMsgTypeOut = p->requestMasterMsgType;
        }
        break;

        // receive user settings from wire slave
        case S_MSG_GLOBE_SETTINGS:
        #if 0
        #endif
        {
            nextMsgTypeOut = MsgType::M_MSG_NONE;

            I2C_s_globeSettings* p = reinterpret_cast<I2C_s_globeSettings*>(plIn);

            Serial.print(F("== rot. time idx ")); Serial.print(p->userSet_rotationPeriod);
            Serial.print(F(", leds ")); Serial.print(p->userSet_ledEffect);
            Serial.print(F(", ")); Serial.println(p->userSet_ledCycleSpeed);

            // set rotation time is stored in globe attributes array, not in a struct
            int rotTimesCount = globeAttributes_valueListLength[attributeIndex_rotTimes];
            if ((p->userSet_rotationPeriod >= 0) && (p->userSet_rotationPeriod < rotTimesCount)) {
                saveAndUseGlobeAttribute(attributeIndex_rotTimes, p->userSet_rotationPeriod);
            }

            // digital LED settings
            bool valid = ((p->userSet_ledEffect >= cLedstripOff) && (p->userSet_ledEffect <= cRedGreenBlue)
                && (p->userSet_ledCycleSpeed >= cLedstripVeryFast) && (p->userSet_ledCycleSpeed <= cLedStripVerySlow));
            if (valid) { setColorCycle(p->userSet_ledEffect, p->userSet_ledCycleSpeed); }

        }
        break;

        // receive PID settings from wire slave
        case S_MSG_PID_SETTINGS:
        {
            nextMsgTypeOut = MsgType::M_MSG_NONE;

            I2C_s_PIDsettings* p = reinterpret_cast<I2C_s_PIDsettings*>(plIn);

            Serial.print(F("== PID gain ")); Serial.print(p->gainAdjustSteps);
            Serial.print(F(", int.cst ")); Serial.print(p->intTimeCstAdjustSteps);
            Serial.print(F(", dif.cst ")); Serial.println(p->difTimeCstAdjustSteps);

            // setting steps: positive values (preset value = mid point) 
            _pidSettings.gainAdjustSteps = (uint8_t)p->gainAdjustSteps;                  // preset gain corresponds to gainAdjustSteps mid value   
            _pidSettings.intTimeCstAdjustSteps = (uint8_t)p->intTimeCstAdjustSteps;
            _pidSettings.difTimeCstAdjustSteps = (uint8_t)p->difTimeCstAdjustSteps;

            if (_pidSettings.gainAdjustSteps >= settingSteps) { _pidSettings.gainAdjustSteps = settingSteps - 1; }
            if (_pidSettings.intTimeCstAdjustSteps >= settingSteps) { _pidSettings.intTimeCstAdjustSteps = settingSteps - 1; }
            if (_pidSettings.difTimeCstAdjustSteps >= settingSteps) { _pidSettings.difTimeCstAdjustSteps = settingSteps - 1; }

            saveAndUseGlobeAttribute(attributeIndex_gainAdjust, _pidSettings.gainAdjustSteps);
            saveAndUseGlobeAttribute(attributeIndex_intTimeConstAdjust, _pidSettings.intTimeCstAdjustSteps);
            saveAndUseGlobeAttribute(attributeIndex_difTimeConstAdjust, _pidSettings.difTimeCstAdjustSteps);
        }
        break;

        // receive globe vertical position setpoint from wire slave
        case S_MSG_VERT_POS_SETPOINT:
        {
            nextMsgTypeOut = MsgType::M_MSG_NONE;

            I2C_s_vertPosSetpoint* p = reinterpret_cast<I2C_s_vertPosSetpoint*>(plIn);

            Serial.print(F("== vertical position ")); Serial.println(p->userSet_vertPosIndex);

            // vertical position (in mVolt) is stored in the globe attributes array, not in a struct
            int vertPosCount = globeAttributes_valueListLength[attributeIndex_hallmVoltRefs];
            if ((p->userSet_vertPosIndex >= 0) && (p->userSet_vertPosIndex < vertPosCount)) {
                saveAndUseGlobeAttribute(attributeIndex_hallmVoltRefs, p->userSet_vertPosIndex);
            }
        }
        break;


        // receive coil phase adjustment setting from wire slave
        case  S_MSG_COIL_PHASE_ADJUST:
        {
            nextMsgTypeOut = MsgType::M_MSG_NONE;

            I2C_s_coilPhaseAdjust* p = reinterpret_cast<I2C_s_coilPhaseAdjust*>(plIn);

            Serial.print(F("== coil phase adjust ")); Serial.println(p->userSet_coilPhaseAdjust);

            // set coil phase adjust is stored in the globe attributes array, not in a struct
            // phase adjustment in 2-degree increments (0 to 358 degrees)
            if (p->userSet_coilPhaseAdjust > 179) { p->userSet_coilPhaseAdjust = 179; }
            saveAndUseGlobeAttribute(attributeIndex_coilPhaseAdjust, p->userSet_coilPhaseAdjust);
        }
        break;

        default:
        {
            nextMsgTypeOut = MsgType::M_MSG_NONE;
        }
        break;

    }

    _msgStats.I_stats_replyReceived++;

    return;
}




