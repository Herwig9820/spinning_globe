/*
==================================================================================================
Spinning globe extension: using the Wire interface to exchange messages with an Arduino nano esp32.
The nano esp32 acts as a bridge between MQTT and the spinning globe nano (I2C).
over WiFi, e.g. using MQTT.
---------------------------------------------------------------------------------------------------
Copyright 2026 Herwig Taveirne

Program written and tested for Arduino Nano esp32.

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

A complete description of the project can be found here:
https://www.instructables.com/Floating-and-Spinning-Earth-Globe/

===============================================================================================
*/


#include "wireSlave_messages.h"
#include "json_helpers.h"
#include "shared/wire_hw_config.h"

WireSlaveMessages::WireSlaveMessages(SharedContext& sharedContext) : _sharedContext(sharedContext) {
};

// ============================================================================
// Wire messages processing loop: call directly from main loop(), call regularly 
// ============================================================================
bool WireSlaveMessages::loop() {

    static uint32_t lastCommStatsAt{ millis() };

    uint8_t messageTypeIn{};          // as received
    uint8_t payloadSizeIn{};          // as received
    uint8_t msgSequence{};
    uint8_t payloadIn[MASTER_PAYLOAD_MAX];


    // periodically, calculate wire comm. quality and publish to MQTT (5120 = 40 x 128 ms)
    if (millis() - lastCommStatsAt > wireCommQuality_measPeriod) { lastCommStatsAt = millis(); prepareWirecommStatsForMQTT(); }

    bool msgAvailable = wireSlave.popIncomingWireMsg(messageTypeIn, payloadIn, payloadSizeIn, msgSequence);
    if (!msgAvailable) { return false; }

    // signal that wire data is received
    _sharedContext.triggerWireCommLed = true;

    switch (messageTypeIn)
    {
        // hello message: not yet implemented
        /*
        case MsgType::M_MSG_HELLO:  // not yet implemented
        {
            wireSlave.pushOutgoingWireMsg(MsgType::S_MSG_HELLO_ACK, nullptr, 0);             // no payload
        }
        break;
        */

       // ========== message types in: RECEIVE DATA from wire master after an event ==========

        case MsgType::M_MSG_PING:
        {
            // a wire master ping message does not trigger an MQTT message, but master expects an ack reply
            // pings are sent often to (1) get fast ACK replies, possibly containing requests, (2) be used as slave comm. stat. input
            replyAndFlagSlaveDataAvailable(msgSequence);
            pingCount++;
        }
        break;

        case MsgType::M_MSG_STATUS:
        {
            I2C_m_status* pStatus = reinterpret_cast<I2C_m_status*>(payloadIn);
            convertGlobeStatusToMQTT(pStatus);
            replyAndFlagSlaveDataAvailable(msgSequence);
        }
        break;

        case MsgType::M_MSG_GREENWICH:
        {
            I2C_m_greenwich* pGreenwich = reinterpret_cast<I2C_m_greenwich*>(payloadIn);
            convertGlobeGreenwichCueToMQTT(pGreenwich);
            replyAndFlagSlaveDataAvailable(msgSequence);
        }
        break;

        case MsgType::M_MSG_TELEMETRY:
        {
            I2C_m_telemetry* pTelemetry = reinterpret_cast<I2C_m_telemetry*>(payloadIn);
            convertTelemetryToMQTT(pTelemetry);
            replyAndFlagSlaveDataAvailable(msgSequence);
        }
        break;

        case MsgType::M_MSG_TELEMETRY_EXTRA:
        {
            I2C_m_telemetry_extra* pTelemetryExtra = reinterpret_cast<I2C_m_telemetry_extra*>(payloadIn);
            convertTelemetryExtraToMQTT(pTelemetryExtra);
            replyAndFlagSlaveDataAvailable(msgSequence);
        }
        break;


        // ========== message types in: RECEIVE DATA from wire master when requested by slave ==========

        // receive wire master library tx stats 
        case MsgType::M_MSG_SEND_STATS:
        {
        // detailed master-side statistics are not used to calculate wire comm. quality
        /*
            I2C_m_masterSendStats* pMasterSendStats = reinterpret_cast<I2C_m_masterSendStats*>(payloadIn);
            wireStatSummary.masterValidMsgCount += pMasterSendStats->I_stats_tx_sent;
            wireStatSummary.masterErrorMsgCount += pMasterSendStats->errorMsgCount();  // let aside retry warnings
            pMasterSendStats->zeroMembers();
        */
            replyAndFlagSlaveDataAvailable(msgSequence);
        }
        break;

        // receive wire master library rx stats
        case MsgType::M_MSG_RECEIVE_STATS:
        {
        // detailed master-side statistics are not used to calculate wire comm. quality
        /*
            I2C_m_masterReceiveStats* pMasterReceiveStats = reinterpret_cast<I2C_m_masterReceiveStats*>(payloadIn);
            wireStatSummary.masterValidMsgCount += pMasterReceiveStats->I_stats_rx_received;
            wireStatSummary.masterErrorMsgCount += pMasterReceiveStats->errorMsgCount();
            pMasterReceiveStats->zeroMembers();
        */
            replyAndFlagSlaveDataAvailable(msgSequence);
        }
        break;

        // receive globe settings from master (changeable globe attributes)
        case MsgType::M_MSG_GLOBE_SETTINGS:
        {
            I2C_m_globeSettings* pGlobeIn = reinterpret_cast<I2C_m_globeSettings*>(payloadIn);
            convertGlobeSettingsToMQTT(pGlobeIn);
            replyAndFlagSlaveDataAvailable(msgSequence);
        }
        break;

        // receive PID controller settings from master
        case MsgType::M_MSG_PID_SETTINGS:
        {
            I2C_m_PIDsettings* pPIDin = reinterpret_cast<I2C_m_PIDsettings*>(payloadIn);
            convertPIDsettingsToMQTT(pPIDin);
            replyAndFlagSlaveDataAvailable(msgSequence);
        }
        break;

        // receive vertical position setpoint (mV) from master
        case MsgType::M_MSG_VERT_POS_SETPOINT:
        {
            I2C_m_vertPosSetpoint* pVertPosIn = reinterpret_cast<I2C_m_vertPosSetpoint*>(payloadIn);
            convertVertPosSetpointToMQTT(pVertPosIn);
            replyAndFlagSlaveDataAvailable(msgSequence);
        }
        break;

        // receive coil phase adjust (degrees) from master
        case MsgType::M_MSG_COIL_PHASE_ADJUST:
        {
            I2C_m_coilPhaseAdjust* pCoilPhaseIn = reinterpret_cast<I2C_m_coilPhaseAdjust*>(payloadIn);
            convertCoilPhaseAdjustmentToMQTT(pCoilPhaseIn);
            replyAndFlagSlaveDataAvailable(msgSequence);
        }
        break;

        // ========== message types in: RECEIVE REQUEST from master to send data to master ==========

        case MsgType::M_MSG_GLOBE_SETTINGS_REQ:
        {
            // but master needs to observe a delay between this message it sent and the response expected, because this response will only be enqueued now.
            // alternatively, let the slave first inform the master that data is available and let the master then request to send it
            // note: lockStep. Answer is mandatory, even if no data available yet (master can test .slaveHasData)
            wireSlave.pushOutgoingWireMsg(MsgType::S_MSG_GLOBE_SETTINGS_SET, &_sharedContext.committedGlobeSettings, sizeof(I2C_s_globeSettings_set), msgSequence);
            _sharedContext.committedGlobeSettings.slaveHasData = 0;
        }
        break;

        case MsgType::M_MSG_PID_SETTINGS_REQ:
        {
            wireSlave.pushOutgoingWireMsg(MsgType::S_MSG_PID_SETTINGS_SET, &_sharedContext.committedPIDsettings, sizeof(I2C_s_PIDsettings_set), msgSequence);
            _sharedContext.committedPIDsettings.slaveHasData = 0;
        }
        break;

        case MsgType::M_MSG_VERT_POS_SETPOINT_REQ:
        {
            wireSlave.pushOutgoingWireMsg(S_MSG_VERT_POS_SETPOINT_SET, &_sharedContext.committedVertPosSetpoint, sizeof(I2C_s_vertPosSetpoint_set), msgSequence);
            _sharedContext.committedVertPosSetpoint.slaveHasData = 0;
        }
        break;

        case MsgType::M_MSG_COIL_PHASE_ADJUST_REQ:
        {
            wireSlave.pushOutgoingWireMsg(S_MSG_COIL_PHASE_ADJUST_SET, &_sharedContext.committedCoilPhaseAdjust, sizeof(I2C_s_coilPhaseAdjust_set), msgSequence);
            _sharedContext.committedCoilPhaseAdjust.slaveHasData = 0;
        }
        break;

        default:
        {}  // unknown message type: do nothing (master will time out)
        break;

    }
    return true;
}


// ============================================================================
// Convert received wire messages to MQTT and store in outgoing MQTT queue
// ============================================================================
void WireSlaveMessages::convertGlobeStatusToMQTT(I2C_m_status* p) {
    MQTTmsgFromWire msg{};
    // JSON
    snprintf(msg.topic, sizeof(msg.topic), TOPIC_STATUS);
    JsonAssemble::startJson(msg.payload, sizeof(msg.payload));
    JsonAssemble::add(msg.payload, sizeof(msg.payload), PL_KEY_STATUS, "\"%u\"", p->status);
    JsonAssemble::add(msg.payload, sizeof(msg.payload), PL_KEY_FLAGS, "\"%u\"", p->stateFlags);
    JsonAssemble::closeJson(msg.payload, sizeof(msg.payload));
    msg.retain = true;
    _sharedContext.queueToMQTT.push(msg);
};

void WireSlaveMessages::convertGlobeGreenwichCueToMQTT(I2C_m_greenwich* p) {
    MQTTmsgFromWire msg{};
    // JSON
    snprintf(msg.topic, sizeof(msg.topic), TOPIC_GREENWICH);
    JsonAssemble::startJson(msg.payload, sizeof(msg.payload));
    if ((p->status == rotUnlocked) || (p->status == rotLocked)) {
        JsonAssemble::add(msg.payload, sizeof(msg.payload), PL_KEY_ACT_TIME, "\"%.2f s\"", p->actualRotationTime / 1000.);
    }
    else { JsonAssemble::add(msg.payload, sizeof(msg.payload), PL_KEY_ACT_TIME, "\"--.-- s\""); }

    if (p->status == rotLocked) {
        JsonAssemble::add(msg.payload, sizeof(msg.payload), PL_KEY_ROT_SYNC_ERROR, "\"%.2f s\"", p->rotationOutOfSyncTime / 1000.);
        JsonAssemble::add(msg.payload, sizeof(msg.payload), PL_KEY_GREENWICH_LAG, "\"%.2ld degrees\"", p->greenwichLag);      // already in degrees
    }
    else {
        JsonAssemble::add(msg.payload, sizeof(msg.payload), PL_KEY_ROT_SYNC_ERROR, "\"--.-- s\"");
        JsonAssemble::add(msg.payload, sizeof(msg.payload), PL_KEY_GREENWICH_LAG, "\"--- degrees\"");
    }
    JsonAssemble::closeJson(msg.payload, sizeof(msg.payload));
    msg.retain = true;
    _sharedContext.queueToMQTT.push(msg);
};

void WireSlaveMessages::convertTelemetryToMQTT(I2C_m_telemetry* p) {
    float tempC = p->tempSmooth / 100.0f;
    float magnetDutyCycle = (p->magnetOnCyclesSmooth / fastDataRateSamplingPeriods) * 100.0f / (float)timer1Top;
    float isrDuration = p->ISRdurationSmooth / fastDataRateSamplingPeriods;
    float load = ((1000.0f * fastDataRateSamplingPeriods - p->idleLoopMicrosSmooth)
        * 100.0f / (1000.0f * fastDataRateSamplingPeriods));
    float vertPosAvgError = (p->errSignalMagnitudeSmooth / (float)fastDataRateSamplingPeriods) * (ADCvolt / (float)ADCsteps);

    MQTTmsgFromWire msg{};
    // JSON
    snprintf(msg.topic, sizeof(msg.topic), TOPIC_TELEMETRY);
    JsonAssemble::startJson(msg.payload, sizeof(msg.payload));
    JsonAssemble::add(msg.payload, sizeof(msg.payload), PL_KEY_HEATSINK_TEMP, "\"%.1f deg.C\"", tempC);
    JsonAssemble::add(msg.payload, sizeof(msg.payload), PL_KEY_MAGNET_DUTY_CYCLE, "\"%.1f%%\"", magnetDutyCycle);
    JsonAssemble::add(msg.payload, sizeof(msg.payload), PL_KEY_ISR_DURATION, "\"%.0f us\"", isrDuration);
    JsonAssemble::add(msg.payload, sizeof(msg.payload), PL_KEY_PROC_LOAD, "\"%.1f%%\"", load);
    JsonAssemble::add(msg.payload, sizeof(msg.payload), PL_KEY_VERT_POS_ERROR, "\"%.1f mV\"", vertPosAvgError);
    JsonAssemble::add(msg.payload, sizeof(msg.payload), PL_KEY_TTT_INTEGR_TERM, "\"%6ld\"", p->realTTTintegrationTerm);
    JsonAssemble::closeJson(msg.payload, sizeof(msg.payload));
    msg.retain = true;
    _sharedContext.queueToMQTT.push(msg);
}

void WireSlaveMessages::convertTelemetryExtraToMQTT(I2C_m_telemetry_extra* p) {
    MQTTmsgFromWire msg{};
    // JSON
    snprintf(msg.topic, sizeof(msg.topic), TOPIC_TELEMETRY_EXTRA);
    JsonAssemble::startJson(msg.payload, sizeof(msg.payload));
    JsonAssemble::add(msg.payload, sizeof(msg.payload), PL_KEY_SECONDS_FLOATING, "\"%lu\"", (uint32_t)(p->secondsFloating));
    JsonAssemble::add(msg.payload, sizeof(msg.payload), PL_KEY_EVENTS_MISSED, "\"%lu\"", p->eventsMissed);
    JsonAssemble::add(msg.payload, sizeof(msg.payload), PL_KEY_MAX_EVENTS_PENDING, "\"%u\"", p->currentMaxEventsPending);
    JsonAssemble::add(msg.payload, sizeof(msg.payload), PL_KEY_MAX_EVENT_QUEUE_BYTES_USED, "\"%u\"", p->largestEventBufferBytesUsed);
    JsonAssemble::closeJson(msg.payload, sizeof(msg.payload));
    msg.retain = true;
    _sharedContext.queueToMQTT.push(msg);
}

void WireSlaveMessages::convertGlobeSettingsToMQTT(I2C_m_globeSettings* pGlobeIn) {
    MQTTmsgFromWire msg{};
    // JSON
    snprintf(msg.topic, sizeof(msg.topic), TOPIC_GLOBE_SETTINGS);
    JsonAssemble::startJson(msg.payload, sizeof(msg.payload));
    JsonAssemble::add(msg.payload, sizeof(msg.payload), PL_KEY_SET_ROT_TIME, "\"%1u\"", pGlobeIn->rotationPeriodIndex);     // rotation speed
    JsonAssemble::add(msg.payload, sizeof(msg.payload), PL_KEY_SET_LED_EFFECT, "\"%1u\"", pGlobeIn->ledEffect);             // led effect
    JsonAssemble::add(msg.payload, sizeof(msg.payload), PL_KEY_SET_LED_EFFECT_SPEED, "\"%1u\"", pGlobeIn->ledCycleSpeed);   // led effect speed
    JsonAssemble::closeJson(msg.payload, sizeof(msg.payload));
    msg.retain = true;
    _sharedContext.queueToMQTT.push(msg);
};

void WireSlaveMessages::convertPIDsettingsToMQTT(I2C_m_PIDsettings* pPIDIn) {
    MQTTmsgFromWire msg{};
    // JSON
    snprintf(msg.topic, sizeof(msg.topic), TOPIC_PID_SETTINGS);
    JsonAssemble::startJson(msg.payload, sizeof(msg.payload));
    JsonAssemble::add(msg.payload, sizeof(msg.payload), PL_KEY_SET_GAIN_ADJUST, "%1d", pPIDIn->gainAdjustSteps);                // gain adjustment
    JsonAssemble::add(msg.payload, sizeof(msg.payload), PL_KEY_SET_DIFF_TIME_ADJUST, "%1d", pPIDIn->difTimeCstAdjustSteps);     // diff. time constant adjustment
    JsonAssemble::add(msg.payload, sizeof(msg.payload), PL_KEY_SET_INTEGR_TIME_ADJUST, "%1d", pPIDIn->intTimeCstAdjustSteps);   // int. time constant adjustment
    JsonAssemble::closeJson(msg.payload, sizeof(msg.payload));
    msg.retain = true;
    _sharedContext.queueToMQTT.push(msg);
};

void WireSlaveMessages::convertVertPosSetpointToMQTT(I2C_m_vertPosSetpoint* pVertPosIn) {
    MQTTmsgFromWire msg{};
    snprintf(msg.topic, sizeof(msg.topic), TOPIC_VERT_POS_SETPOINT);
    snprintf(msg.payload, sizeof(msg.payload), "%1d", pVertPosIn->vertPosIndex);
    msg.retain = true;
    _sharedContext.queueToMQTT.push(msg);
};

void WireSlaveMessages::convertCoilPhaseAdjustmentToMQTT(I2C_m_coilPhaseAdjust* pCoilPhaseIn) {
    MQTTmsgFromWire msg{};
    snprintf(msg.topic, sizeof(msg.topic), TOPIC_COIL_PHASE_ADJUST);
    snprintf(msg.payload, sizeof(msg.payload), "%1d", pCoilPhaseIn->coilPhaseAdjust);
    msg.retain = true;
    _sharedContext.queueToMQTT.push(msg);
};

void WireSlaveMessages::prepareWirecommStatsForMQTT() {
    // pragmatic calculation of wire comm. quality: for every valid message received, a message is sent (lockstep)
    // - rx: count number of valid pings received in period (master pings each 128 ms, the 8th ping is replaced by a status message which is not counted)
    // - tx: calculate ratio of sent messages without error / all sent attempts
    // multiply the two (valid because of lockstep)
    constexpr uint32_t expectedPingCount = (wireCommQuality_measPeriod / 128 * 7) / 8;
    if (pingCount == expectedPingCount - 1) { pingCount++; }
    else if (pingCount > expectedPingCount) { pingCount = expectedPingCount; } // allow for small jitter
    float rxQuality = ((float)pingCount) / expectedPingCount;
    pingCount = 0; // reset
    float txQuality = wireSlave.calculateTxQualityInPeriod();       // public slave transport method
    MQTTmsgFromWire msg{};
    snprintf(msg.topic, sizeof(msg.topic), TOPIC_WIRE_COMM_QUALITY);
    snprintf(msg.payload, sizeof(msg.payload), "%u", (uint16_t)(rxQuality * txQuality * 100.));        // 0 to 100
    msg.retain = true;
    _sharedContext.queueToMQTT.push(msg);
}


// ============================================================================================
// Reply to message from wire master with an 'ACK' message
// -------------------------------------------------------
// ============================================================================================
void WireSlaveMessages::replyAndFlagSlaveDataAvailable(uint8_t msgSequence) {

    I2C_s_ack thisAckResponse{}, nextAckResponse{};

    // ---------- PRIO 1: the slave has a message (settings, ...) available for wire master, NOT fitting in this THIS ack response ? ----------

    // manage the intermediate buffers: move 'pending' messages to 'committed' (make the 'pending' slots available again) 

    // two stage buffer: pending -> committed -> send to wire master
    if (_sharedContext.pendingGlobeSettings.slaveHasData && !_sharedContext.committedGlobeSettings.slaveHasData) {
        _sharedContext.committedGlobeSettings = _sharedContext.pendingGlobeSettings;        // this also sets '.hasSlaveData' to '1'
        _sharedContext.pendingGlobeSettings.slaveHasData = 0;                               // because it was just committed
    }

    if (_sharedContext.pendingPIDsettings.slaveHasData && !_sharedContext.committedPIDsettings.slaveHasData) {
        _sharedContext.committedPIDsettings = _sharedContext.pendingPIDsettings;            // this also sets '.hasSlaveData' to '1'
        _sharedContext.pendingPIDsettings.slaveHasData = 0;                                 // because it was just committed
    }

    if (_sharedContext.pendingVertPosSetpoint.slaveHasData && !_sharedContext.committedVertPosSetpoint.slaveHasData) {
        _sharedContext.committedVertPosSetpoint = _sharedContext.pendingVertPosSetpoint;    // this also sets '.hasSlaveData' to '1'
        _sharedContext.pendingVertPosSetpoint.slaveHasData = 0;                             // because it was just committed
    }

    if (_sharedContext.pendingCoilPhaseAdjust.slaveHasData && !_sharedContext.committedCoilPhaseAdjust.slaveHasData) {
        _sharedContext.committedCoilPhaseAdjust = _sharedContext.pendingCoilPhaseAdjust;    // this also sets '.hasSlaveData' to '1'
        _sharedContext.pendingCoilPhaseAdjust.slaveHasData = 0;                             // because it was just committed
    }

    // if a specific message type is 'committed' (e.g., a PID settings record) for transmission to wire master, the slave must first prepare TWO ACK reply message payloads
    // and store them in a common intermediate queue (this queue is used for the same purpose in MQTTmessages::holdRequestForWireMaster() ).
    // first ACK payload : inform the master it should request to transmit a specific message type
    // second ACK payload: inform the master it should retransmit the data to allow other subscribers to receive it as well

    if (_sharedContext.committedGlobeSettings.slaveHasData) {
        thisAckResponse.requestMasterMsgType = MsgType::M_MSG_GLOBE_SETTINGS_REQ;           // THIS ack response: inform master it should request this data
        nextAckResponse.requestMasterMsgType = MsgType::M_MSG_GLOBE_SETTINGS;               // NEXT ack response: inform master it should send out again this updated data
        _sharedContext.holdAckResponses.push(nextAckResponse);                              // hold NEXT ack response until it's time to send the next ack
    }

    else if (_sharedContext.committedPIDsettings.slaveHasData) {
        thisAckResponse.requestMasterMsgType = MsgType::M_MSG_PID_SETTINGS_REQ;
        nextAckResponse.requestMasterMsgType = MsgType::M_MSG_PID_SETTINGS;
        _sharedContext.holdAckResponses.push(nextAckResponse);
    }

    else if (_sharedContext.committedVertPosSetpoint.slaveHasData) {
        thisAckResponse.requestMasterMsgType = MsgType::M_MSG_VERT_POS_SETPOINT_REQ;
        nextAckResponse.requestMasterMsgType = MsgType::M_MSG_VERT_POS_SETPOINT;
        _sharedContext.holdAckResponses.push(nextAckResponse);
    }

    else if (_sharedContext.committedCoilPhaseAdjust.slaveHasData) {
        thisAckResponse.requestMasterMsgType = MsgType::M_MSG_COIL_PHASE_ADJUST_REQ;
        nextAckResponse.requestMasterMsgType = MsgType::M_MSG_COIL_PHASE_ADJUST;
        _sharedContext.holdAckResponses.push(nextAckResponse);
    }


    // ---------- PRIO 2: the slave has a message for wire master, FITTING in THIS ack response ?  ----------

    // if an ACK payload is available in the intermediate message buffer, transmit it (push it to the outgoing wire queue)
    else if (!_sharedContext.holdAckResponses.empty()) {
        // THIS ack response is used to inform wire master that it should send data (a message type) or it should perform an action (e.g., visual ring) 
        thisAckResponse.requestMasterMsgType = _sharedContext.holdAckResponses.front()->requestMasterMsgType;
        thisAckResponse.action = _sharedContext.holdAckResponses.front()->action;
        I2C_s_ack dummy;
        _sharedContext.holdAckResponses.pop(dummy);

    }


    // ---------- return 'ack' message, containing optional master message type requested from master ----------

    I2C_s_ack p;
    p.ack = 0x0;        // not used
    p.requestMasterMsgType = thisAckResponse.requestMasterMsgType;
    p.action = thisAckResponse.action;
    if (!wireSlave.pushOutgoingWireMsg(MsgType::S_MSG_ACK, &p, sizeof(p), msgSequence)){};

}

/*
Example sequence of events when node-red publishes PID settings:
----------------------------------------------------------------

MQTT     esp32     wire         event
topic    bridge    message
-------------------------------------------------
     -->                        MQTT publishes PID settings. PID settings record is temporarily stored in the 'pending' stage of the
                                intermediate PID settings buffer overwriting any previous message stored there.
                                The pending record will be promoted to 'committed' at the next ACK reply opportunity, provided the committed slot is empty.

               <--              Spinning globe (wire master) transmits a message requiring an ACK response

               -->              Wire slave replies with ACK message, payload is 'wire master must request PID settings'
                                A next ACK reply 'wire master must publish PID settings' is temporarily stored in intermediate ACK reply queue.
                                It will be sent at the next ACK reply opportunity.
                                This ensures the committed settings are confirmed back to MQTT once the master has accepted them.

               <--              Spinning globe requests the PID settings

               -->              The PID settings are sent to the spinning globe

               <--              Spinning globe (wire master) transmits a message requiring an ACK response

               -->              Wire slave replies with ACK message, payload (popped from intermediate buffer) is 'wire master must publish PID settings'

               <--              The spinning globe transmits the PID settings

               -->              An ACK message is sent to the spinning globe

     <--                        Wire slave publishes the received PID settings to MQTT

*/
