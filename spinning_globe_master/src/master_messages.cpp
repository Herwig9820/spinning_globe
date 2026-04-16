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
Wire master: message handling layer
===============================================================================
*/

#include "master_context.h"
#include "master_messages.h"

// forward declarations
void saveAndUseGlobeAttribute(uint8_t attributeIndex, uint8_t attributeValue);
void setColorCycle(uint8_t newColorCycle, uint8_t newColorTiming, bool initColorCycle = false);


// ==========  constructor / destructor ==========

MessageHandling::MessageHandling(GreenwichData& greenwichData, StatusData& statusData, SecondData& secondData,
    SmoothedMeasurements& smoothedMeasurements, PIDsettings& pidSettings, int* globeMetrics,
    LedStripSettings& ledStripSettings, EventData& globeEventSnapshot, VisualRing& visualRing, volatile bool& triggerWireCommLed) :
    _greenwichData(greenwichData), _statusData(statusData), _secondData(secondData),
    _smoothedMeasurements(smoothedMeasurements), _pidSettings(pidSettings), _globeMetrics(globeMetrics),
    _ledStripSettings(ledStripSettings), _globeEventSnapshot(globeEventSnapshot), _visualRing(visualRing),
    _wireMaster(triggerWireCommLed) {
};

MessageHandling::~MessageHandling() {};

uint8_t MessageHandling::transmit() {
    return (uint8_t)_wireMaster.sendAndReceiveMessage();                                    // return master or slave message in error
}

// ========== enqueue data to send buffer ==========

void MessageHandling::enqueueI2CmessageToSlave(MsgType& msgTypeOut) {

    if (msgTypeOut == MsgType::M_MSG_NONE) { return; }

    // note: if tx buffer full, message is silently dropped but error counter is incremented

    switch (msgTypeOut) {

        // ========== message type requesting a hello acknowledge from slave ==========

        case MsgType::M_MSG_HELLO:                                                          //NOTE: logic not yet implemented !                                                    
        {
            _wireMaster.enqueueTx(M_MSG_HELLO, 0, nullptr, S_MSG_HELLO_ACK, 0);             // no payload
        }
        break;

        // ========== empty message type requesting an acknowledge from slave ==========

        case MsgType::M_MSG_PING:
        {
           // sent regularly; allows slave to reply and inform master that either it has data for, or it requests data from master
            _wireMaster.enqueueTx(M_MSG_PING, 0, nullptr, S_MSG_ACK, sizeof(I2C_s_ack));    // no payload
        }
        break;


        // ========== message types SENDING DATA to slave after a globe event or when requested ==========

        case MsgType::M_MSG_STATUS:
        {
            I2C_m_status p{};
            p.status = (int8_t)((_statusData.errorCondition == errNoError) ? _statusData.rotationStatus : (_statusData.errorCondition | 0x10));
            if (p.status == rotNoPosSync) {
                if (_statusData.isFloating) {
                    uint8_t rotTimeIndex = _globeMetrics[attributeIndex_rotTimes];  // index into list of rotation times
                    int setRotationTime = *(globeMetrics_valueListsPointers[attributeIndex_rotTimes] + rotTimeIndex);
                    if (rotTimeIndex == 0) { p.status = wire_rotOff; }
                }
                else { p.status = wire_notFloating; }
            }
            p.stateFlags = _visualRing.ringType();       // 0,1,2 = off, ringing, alarm
            _wireMaster.enqueueTx(M_MSG_STATUS, sizeof(p), &p, S_MSG_ACK, sizeof(I2C_s_ack));
        }
        break;

        case  MsgType::M_MSG_GREENWICH:
        {
            I2C_m_greenwich p{};
            // globe status: only states not locked and locked are relevant
            p.status = (int8_t)((_statusData.errorCondition == errNoError) ? _statusData.rotationStatus : (_statusData.errorCondition | 0x10));
            if ((p.status == rotUnlocked) || (p.status == rotLocked)) {
                p.actualRotationTime = _greenwichData.globeRotationTime;
            }
            if (p.status == rotLocked) {  // this status does not occur when rotation is OFF
                p.rotationOutOfSyncTime = _greenwichData.rotationOutOfSyncTime;
                // multiply by 360 degrees and divide by time of 1 rotation (ms)
                uint8_t rotTimeIndex = _globeMetrics[attributeIndex_rotTimes];
                int setRotationTime = *(globeMetrics_valueListsPointers[attributeIndex_rotTimes] + rotTimeIndex);
                p.greenwichLag = (int32_t)((_greenwichData.greenwichLag * 360) / setRotationTime);
            }
            _wireMaster.enqueueTx(M_MSG_GREENWICH, sizeof(p), &p, S_MSG_ACK, sizeof(I2C_s_ack));
        }
        break;

        case  MsgType::M_MSG_TELEMETRY:
        {
            I2C_m_telemetry p{};
            p.tempSmooth = _smoothedMeasurements.tempSmooth;                                // raw input for wire slave
            p.magnetOnCyclesSmooth = _smoothedMeasurements.magnetOnCyclesSmooth;
            p.ISRdurationSmooth = _smoothedMeasurements.ISRdurationSmooth;
            p.idleLoopMicrosSmooth = _smoothedMeasurements.idleLoopMicrosSmooth;
            p.errSignalMagnitudeSmooth = _smoothedMeasurements.errSignalMagnitudeSmooth;
            p.realTTTintegrationTerm = _secondData.realTTTintegrationTerm;
            _wireMaster.enqueueTx(M_MSG_TELEMETRY, sizeof(p), &p, S_MSG_ACK, sizeof(I2C_s_ack));
        }
        break;

        case  MsgType::M_MSG_TELEMETRY_EXTRA:
        {
            I2C_m_telemetry_extra p{};
            p.secondsFloating = _secondData.liftingSecond;                                   // raw input for wire slave
            p.eventsMissed = _globeEventSnapshot.eventsMissed;
            p.currentMaxEventsPending = _globeEventSnapshot.largestEventsPending;
            p.largestEventBufferBytesUsed = _globeEventSnapshot.largestEventBufferBytesUsed;
            _wireMaster.enqueueTx(M_MSG_TELEMETRY_EXTRA, sizeof(p), &p, S_MSG_ACK, sizeof(I2C_s_ack));
        }
        break;

        // send user settings to wire slave
        case MsgType::M_MSG_GLOBE_SETTINGS:
        {
            I2C_m_globeSettings p{};
            // set rotation time is stored in globe attributes array, not in a struct
            p.rotationPeriodIndex = _globeMetrics[attributeIndex_rotTimes];      // index
            p.ledEffect = _ledStripSettings.ledEffect;
            p.ledCycleSpeed = _ledStripSettings.ledCycleSpeed;
            _wireMaster.enqueueTx(M_MSG_GLOBE_SETTINGS, sizeof(p), &p, S_MSG_ACK, sizeof(I2C_s_ack));
        }
        break;

        // send PID settings to wire slave
        case MsgType::M_MSG_PID_SETTINGS:
        {
            I2C_m_PIDsettings p{};
            p.gainAdjustSteps = _pidSettings.gainAdjustSteps;
            p.difTimeCstAdjustSteps = _pidSettings.difTimeCstAdjustSteps;
            p.intTimeCstAdjustSteps = _pidSettings.intTimeCstAdjustSteps;
            _wireMaster.enqueueTx(M_MSG_PID_SETTINGS, sizeof(p), &p, S_MSG_ACK, sizeof(I2C_s_ack));
        }
        break;


        // send globe vertical position setpoint to wire slave
        case MsgType::M_MSG_VERT_POS_SETPOINT:
        {
            I2C_m_vertPosSetpoint p{};
            // vertical position (in mVolt) is stored in the globe metrics array, not in a struct
            p.vertPosIndex = _globeMetrics[attributeIndex_hallmVoltRefs];
            _wireMaster.enqueueTx(M_MSG_VERT_POS_SETPOINT, sizeof(p), &p, S_MSG_ACK, sizeof(I2C_s_ack));
        }
        break;

        // send coil phase adjustment setting to wire slave
        case MsgType::M_MSG_COIL_PHASE_ADJUST:
        {
            I2C_m_coilPhaseAdjust p{};
            // set coil phase adjust is stored in the globe attributes array, not in a struct
            p.coilPhaseAdjust = _globeMetrics[attributeIndex_coilPhaseAdjust];               // 0 to 179
            _wireMaster.enqueueTx(M_MSG_COIL_PHASE_ADJUST, sizeof(p), &p, S_MSG_ACK, sizeof(I2C_s_ack));
        }
        break;


        // send wire master library send stats to wire slave
        case MsgType::M_MSG_SEND_STATS:
        {
            I2C_m_masterSendStats p{};
            _wireMaster.getAndClearSendStats(p);
            _wireMaster.enqueueTx(M_MSG_SEND_STATS, sizeof(p), &p, S_MSG_ACK, sizeof(I2C_s_ack));
        }
        break;

        // send wire master library receive stats to wire slave
        case MsgType::M_MSG_RECEIVE_STATS:
        {
            I2C_m_masterReceiveStats p{};
            _wireMaster.getAndClearReceiveStats(p);
            _wireMaster.enqueueTx(M_MSG_RECEIVE_STATS, sizeof(p), &p, S_MSG_ACK, sizeof(I2C_s_ack));
        }
        break;

        // send message library stats to wire slave
        case MsgType::M_MSG_MESSAGE_STATS:
        {
            // note: _msgStats is already up to date 
            _wireMaster.enqueueTx(M_MSG_MESSAGE_STATS, sizeof(_msgStats), &_msgStats, S_MSG_ACK, sizeof(I2C_s_ack));
            _msgStats.zeroMembers();
        }
        break;


        // ========== message types REQUESTING DATA from slave (no payload) ==========

        case MsgType::M_MSG_GLOBE_SETTINGS_REQ:
        {
            _wireMaster.enqueueTx(M_MSG_GLOBE_SETTINGS_REQ, 0, nullptr, S_MSG_GLOBE_SETTINGS_SET, sizeof(I2C_s_globeSettings_set));
        }
        break;

        case MsgType::M_MSG_PID_SETTINGS_REQ:
        {
            _wireMaster.enqueueTx(M_MSG_PID_SETTINGS_REQ, 0, nullptr, S_MSG_PID_SETTINGS_SET, sizeof(I2C_s_PIDsettings_set));
        }
        break;

        case MsgType::M_MSG_VERT_POS_SETPOINT_REQ:
        {
            _wireMaster.enqueueTx(M_MSG_VERT_POS_SETPOINT_REQ, 0, nullptr, S_MSG_VERT_POS_SETPOINT_SET, sizeof(I2C_s_vertPosSetpoint_set));
        }
        break;

        case MsgType::M_MSG_COIL_PHASE_ADJUST_REQ:
        {
            _wireMaster.enqueueTx(M_MSG_COIL_PHASE_ADJUST_REQ, 0, nullptr, S_MSG_COIL_PHASE_ADJUST_SET, sizeof(I2C_s_coilPhaseAdjust_set));
        }
        break;
    }
    msgTypeOut = MsgType::M_MSG_NONE;
    return;
}


// ========== process Wire slave reply ==========

bool MessageHandling::dequeueI2CmessageFromSlave(MsgType& nextMsgTypeOut, Action& nextAction, uint32_t& dashboardAliveAt) {
    uint8_t msgTypeIn{ MsgType::M_MSG_NONE };                   // message type received from slave
    uint8_t i2cPayloadSizeIn{ 0 };                              // payload size as reported by slave
    uint8_t plIn[SLAVE_PAYLOAD_MAX];
    uint8_t expMsgType{};

    // message available  ?
    bool msgAvailable = _wireMaster.dequeueRx(msgTypeIn, i2cPayloadSizeIn, &plIn, expMsgType);
    if (!msgAvailable) {
        nextMsgTypeOut = M_MSG_NONE;
        return false;
    }

    // this is not the expected message type (strict lockStep implemented)
    // (note that, if the message type is correct, then the message size is as well (handled in WireMaster library) 
    if (msgTypeIn != expMsgType) {
        _msgStats.E_stats_lockStepError++;
        return false;
    }

    switch (msgTypeIn) {
        // answer to hello message
        case S_MSG_HELLO_ACK:   // not yet implemented !
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
            nextAction = p->action;
            // !!! setting dashboardAliveAt: ONLY FOR ACK MSG PAYLOADS ORIGINATING FROM DASHBOARD TOPICS (test if needed) !!!
            if ((nextMsgTypeOut != M_MSG_NONE) || (nextAction != M_ACTION_NONE)) { dashboardAliveAt = millis();  }
        }
        break;

        // receive user settings from wire slave
        case S_MSG_GLOBE_SETTINGS_SET:
        {
            nextMsgTypeOut = MsgType::M_MSG_NONE;

            I2C_s_globeSettings_set* p = reinterpret_cast<I2C_s_globeSettings_set*>(plIn);
            if (!p->slaveHasData) { break; }

            // set rotation time is stored in globe attributes array, not in a struct
            int rotTimesCount = globeMetrics_listLengths[attributeIndex_rotTimes];
            if ((p->rotationPeriodIndex >= 0) && (p->rotationPeriodIndex < rotTimesCount)) {
                saveAndUseGlobeAttribute(attributeIndex_rotTimes, p->rotationPeriodIndex);
            }

            // digital LED settings
            bool valid = ((p->ledEffect >= cLedstripOff) && (p->ledEffect <= cRedGreenBlue)
                && (p->ledCycleSpeed >= cLedstripVeryFast) && (p->ledCycleSpeed <= cLedStripVerySlow));
            if (valid) { setColorCycle(p->ledEffect, p->ledCycleSpeed); }

        }
        break;

        // receive PID settings from wire slave
        case S_MSG_PID_SETTINGS_SET:
        {
            nextMsgTypeOut = MsgType::M_MSG_NONE;

            I2C_s_PIDsettings_set* p = reinterpret_cast<I2C_s_PIDsettings_set*>(plIn);
            if (!p->slaveHasData) { break; }

            // setting steps: positive values (preset value = mid point) 
            _pidSettings.gainAdjustSteps = (uint8_t)p->gainAdjustSteps;                  // preset gain corresponds to gainAdjustSteps mid value   
            _pidSettings.intTimeCstAdjustSteps = (uint8_t)p->intTimeCstAdjustSteps;
            _pidSettings.difTimeCstAdjustSteps = (uint8_t)p->difTimeCstAdjustSteps;

            if (_pidSettings.gainAdjustSteps > settingSteps) { _pidSettings.gainAdjustSteps = settingSteps; }
            if (_pidSettings.intTimeCstAdjustSteps > settingSteps) { _pidSettings.intTimeCstAdjustSteps = settingSteps; }
            if (_pidSettings.difTimeCstAdjustSteps > settingSteps) { _pidSettings.difTimeCstAdjustSteps = settingSteps; }

            saveAndUseGlobeAttribute(attributeIndex_gainAdjust, _pidSettings.gainAdjustSteps);
            saveAndUseGlobeAttribute(attributeIndex_intTimeConstAdjust, _pidSettings.intTimeCstAdjustSteps);
            saveAndUseGlobeAttribute(attributeIndex_difTimeConstAdjust, _pidSettings.difTimeCstAdjustSteps);
        }
        break;

        // receive globe vertical position setpoint from wire slave
        case S_MSG_VERT_POS_SETPOINT_SET:
        {
            nextMsgTypeOut = MsgType::M_MSG_NONE;

            I2C_s_vertPosSetpoint_set* p = reinterpret_cast<I2C_s_vertPosSetpoint_set*>(plIn);
            if (!p->slaveHasData) { break; }

            // vertical position (in mVolt) is stored in the globe attributes array, not in a struct
            int vertPosCount = globeMetrics_listLengths[attributeIndex_hallmVoltRefs];
            if ((p->vertPosIndex >= 0) && (p->vertPosIndex < vertPosCount)) {
                saveAndUseGlobeAttribute(attributeIndex_hallmVoltRefs, p->vertPosIndex);
            }
        }
        break;


        // receive coil phase adjustment setting from wire slave
        case  S_MSG_COIL_PHASE_ADJUST_SET:
        {
            nextMsgTypeOut = MsgType::M_MSG_NONE;

            I2C_s_coilPhaseAdjust_set* p = reinterpret_cast<I2C_s_coilPhaseAdjust_set*>(plIn);
            if (!p->slaveHasData) { break; }

            // set coil phase adjust is stored in the globe attributes array, not in a struct
            // phase adjustment in 2-degree increments (0 to 358 degrees)
            if (p->coilPhaseAdjust > 179) { p->coilPhaseAdjust = 179; }
            saveAndUseGlobeAttribute(attributeIndex_coilPhaseAdjust, p->coilPhaseAdjust);
        }
        break;

        default:
        {
            nextMsgTypeOut = MsgType::M_MSG_NONE;                       // safety
        }
        break;

    }

    _msgStats.I_stats_replyReceived++;

    return true;
}




