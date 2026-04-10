#include "wireSlave_messages.h"
#include "json_helpers.h"
#include "shared/wire_hw_config.h"

WireSlaveMessages::WireSlaveMessages(SharedContext& sharedContext) : _sharedContext(sharedContext) {
};

bool WireSlaveMessages::loop() {

    uint8_t messageTypeIn{};          // as received
    uint8_t payloadSizeIn{};          // as received
    uint8_t payloadIn[MASTER_PAYLOAD_MAX];

    bool msgAvailable = wireSlave.popIncomingWireMsg(messageTypeIn, payloadIn, payloadSizeIn);
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
            replyAndFlagSlaveDataAvailable();
        }
        break;

        case MsgType::M_MSG_STATUS:
        {
            I2C_m_status* pStatus = reinterpret_cast<I2C_m_status*>(payloadIn);
            convertGlobeStatusToMQTT(pStatus);
            replyAndFlagSlaveDataAvailable();
        }
        break;

        case MsgType::M_MSG_GREENWICH:
        {
            I2C_m_greenwich* pGreenwich = reinterpret_cast<I2C_m_greenwich*>(payloadIn);
            convertGlobeGreenwichCueToMQTT(pGreenwich);
            replyAndFlagSlaveDataAvailable();
        }
        break;

        case MsgType::M_MSG_TELEMETRY:
        {
            I2C_m_telemetry* pTelemetry = reinterpret_cast<I2C_m_telemetry*>(payloadIn);
            convertTelemetryToMQTT(pTelemetry);
            replyAndFlagSlaveDataAvailable();
        }
        break;

        case MsgType::M_MSG_TELEMETRY_EXTRA:
        {
            I2C_m_telemetry_extra* pTelemetryExtra = reinterpret_cast<I2C_m_telemetry_extra*>(payloadIn);
            convertTelemetryExtraToMQTT(pTelemetryExtra);
            replyAndFlagSlaveDataAvailable();
        }
        break;


        // ========== message types in: RECEIVE DATA from wire master when requested by slave ==========

        // receive wire master library tx stats 
        case MsgType::M_MSG_SEND_STATS:
        {
            replyAndFlagSlaveDataAvailable();
        }
        break;

        // receive wire master library rx stats
        case MsgType::M_MSG_RECEIVE_STATS:
        {
            replyAndFlagSlaveDataAvailable();
        }
        break;

        // receive wire master message library stats
        case MsgType::M_MSG_MESSAGE_STATS:
        {
            replyAndFlagSlaveDataAvailable();
        }
        break;

        // receive globe settings from master (changeable globe attributes)
        case MsgType::M_MSG_GLOBE_SETTINGS:
        {
            I2C_m_globeSettings* pGlobeIn = reinterpret_cast<I2C_m_globeSettings*>(payloadIn);
            convertGlobeSettingsToMQTT(pGlobeIn);
            replyAndFlagSlaveDataAvailable();
        }
        break;

        // receive PID controller settings from master
        case MsgType::M_MSG_PID_SETTINGS:
        {
            I2C_m_PIDsettings* pPIDin = reinterpret_cast<I2C_m_PIDsettings*>(payloadIn);
            convertPIDsettingsToMQTT(pPIDin);
            replyAndFlagSlaveDataAvailable();
        }
        break;

        // receive vertical position setpoint (mV) from master
        case MsgType::M_MSG_VERT_POS_SETPOINT:
        {
            I2C_m_vertPosSetpoint* pVertPosIn = reinterpret_cast<I2C_m_vertPosSetpoint*>(payloadIn);
            convertVertPosSetpointToMQTT(pVertPosIn);
            replyAndFlagSlaveDataAvailable();
        }
        break;

        // receive coil phase adjust (degrees) from master
        case MsgType::M_MSG_COIL_PHASE_ADJUST:
        {
            I2C_m_coilPhaseAdjust* pCoilPhaseIn = reinterpret_cast<I2C_m_coilPhaseAdjust*>(payloadIn);
            convertCoilPhaseAdjustmentToMQTT(pCoilPhaseIn);
            replyAndFlagSlaveDataAvailable();
        }
        break;

        // ========== message types in: RECEIVE REQUEST from master to send data to master ==========

        case MsgType::M_MSG_GLOBE_SETTINGS_REQ:
        {
            // but master needs to observe a delay between this message it sent and the response expected, because this response will only be enqueued now.
            // alternatively, let the slave first inform the master that data is available and let the master then request to send it
            // note: lockStep. Answer is mandatory, even if no data available yet (master can test .slaveHasData)
            wireSlave.pushOutgoingWireMsg(MsgType::S_MSG_GLOBE_SETTINGS_SET, &_sharedContext.committedGlobeSettings, sizeof(I2C_s_globeSettings_set));
            _sharedContext.committedGlobeSettings.slaveHasData = 0;
        }
        break;

        case MsgType::M_MSG_PID_SETTINGS_REQ:
        {
            wireSlave.pushOutgoingWireMsg(MsgType::S_MSG_PID_SETTINGS_SET, &_sharedContext.committedPIDsettings, sizeof(I2C_s_PIDsettings_set));
            _sharedContext.committedPIDsettings.slaveHasData = 0;
        }
        break;

        case MsgType::M_MSG_VERT_POS_SETPOINT_REQ:
        {
            wireSlave.pushOutgoingWireMsg(S_MSG_VERT_POS_SETPOINT_SET, &_sharedContext.committedVertPosSetpoint, sizeof(I2C_s_vertPosSetpoint_set));
            _sharedContext.committedVertPosSetpoint.slaveHasData = 0;
        }
        break;

        case MsgType::M_MSG_COIL_PHASE_ADJUST_REQ:
        {
            wireSlave.pushOutgoingWireMsg(S_MSG_COIL_PHASE_ADJUST_SET, &_sharedContext.committedCoilPhaseAdjust, sizeof(I2C_s_coilPhaseAdjust_set));
            _sharedContext.committedCoilPhaseAdjust.slaveHasData = 0;
        }
        break;

        default:
        {}  // unknown message type: do nothing (master will time out)
        break;

    }
    return true;
}


void WireSlaveMessages::convertGlobeStatusToMQTT(I2C_m_status* p) {
    MQTTmsgFromWire msg{};
    snprintf(msg.topic, sizeof(msg.topic), TOPIC_STATUS);
    snprintf(msg.payload, sizeof(msg.payload), "%u", p->status);
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

void WireSlaveMessages::convertTelemetryExtraToMQTT(I2C_m_telemetry_extra* p){
    MQTTmsgFromWire msg{};
    // JSON
    snprintf(msg.topic, sizeof(msg.topic), TOPIC_TELEMETRY_EXTRA);
    JsonAssemble::startJson(msg.payload, sizeof(msg.payload));
    JsonAssemble::add(msg.payload, sizeof(msg.payload),PL_KEY_SECONDS_FLOATING, "\"%lu\"", (uint32_t)(p->secondsFloating));
    JsonAssemble::add(msg.payload, sizeof(msg.payload),PL_KEY_EVENTS_MISSED, "\"%lu\"", p->eventsMissed);
    JsonAssemble::add(msg.payload, sizeof(msg.payload),PL_KEY_MAX_EVENTS_PENDING, "\"%u\"", p->currentMaxEventsPending);
    JsonAssemble::add(msg.payload, sizeof(msg.payload),PL_KEY_MAX_EVENT_QUEUE_BYTES_USED, "\"%u\"", p->largestEventBufferBytesUsed);
    JsonAssemble::closeJson(msg.payload, sizeof(msg.payload));
    msg.retain = true;
    _sharedContext.queueToMQTT.push(msg);
}

void WireSlaveMessages::convertGlobeSettingsToMQTT(I2C_m_globeSettings* pGlobeIn) {
    MQTTmsgFromWire msg{};
    // JSON
    snprintf(msg.topic, sizeof(msg.topic), TOPIC_GLOBE_SETTINGS);
    JsonAssemble::startJson(msg.payload, sizeof(msg.payload));
    JsonAssemble::add(msg.payload, sizeof(msg.payload), PL_KEY_SET_ROT_TIME, "\"%1u\"", pGlobeIn->rotationPeriodIndex);    // rotation speed
    JsonAssemble::add(msg.payload, sizeof(msg.payload), PL_KEY_SET_LED_EFFECT, "\"%1u\"", pGlobeIn->ledEffect);            // led effect
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
    JsonAssemble::add(msg.payload, sizeof(msg.payload), PL_KEY_SET_GAIN_ADJUST, "%1d", pPIDIn->gainAdjustSteps);              // gain adjustment
    JsonAssemble::add(msg.payload, sizeof(msg.payload), PL_KEY_SET_DIFF_TIME_ADJUST, "%1d", pPIDIn->difTimeCstAdjustSteps);     // diff. time constant adjustment
    JsonAssemble::add(msg.payload, sizeof(msg.payload), PL_KEY_SET_INTEGR_TIME_ADJUST, "%1d", pPIDIn->intTimeCstAdjustSteps);     // int. time constant adjustment
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


// ============================================================================================
// Reply to message from wire master with an 'ACK' message
// ============================================================================================

void WireSlaveMessages::replyAndFlagSlaveDataAvailable() {

    I2C_s_ack thisAckResponse{}, nextAckResponse{};

    // ---------- PRIO 1: the slave has a message (settings, ...) available for wire master, NOT fitting in this THIS ack response ? ----------

    // 2 stage buffer: pending -> committed -> send to wire master
    if (_sharedContext.pendingGlobeSettings.slaveHasData && !_sharedContext.committedGlobeSettings.slaveHasData) {
        _sharedContext.committedGlobeSettings = _sharedContext.pendingGlobeSettings;        // this also sets '.hasSlaveData' to '1'
        _sharedContext.pendingGlobeSettings.slaveHasData = 0;                               // because it was just committed
    }

    else if (_sharedContext.pendingPIDsettings.slaveHasData && !_sharedContext.committedPIDsettings.slaveHasData) {
        _sharedContext.committedPIDsettings = _sharedContext.pendingPIDsettings;        // this also sets '.hasSlaveData' to '1'
        _sharedContext.pendingPIDsettings.slaveHasData = 0;                               // because it was just committed
    }

    else if (_sharedContext.pendingVertPosSetpoint.slaveHasData && !_sharedContext.committedVertPosSetpoint.slaveHasData) {
        _sharedContext.committedVertPosSetpoint = _sharedContext.pendingVertPosSetpoint;        // this also sets '.hasSlaveData' to '1'
        _sharedContext.pendingVertPosSetpoint.slaveHasData = 0;                                 // because it was just committed
    }

    else if (_sharedContext.pendingCoilPhaseAdjust.slaveHasData && !_sharedContext.committedCoilPhaseAdjust.slaveHasData) {
        _sharedContext.committedCoilPhaseAdjust = _sharedContext.pendingCoilPhaseAdjust;        // this also sets '.hasSlaveData' to '1'
        _sharedContext.pendingCoilPhaseAdjust.slaveHasData = 0;                               // because it was just committed
    }

    /*
    Incoming MQTT topic processing (MQTTmessages class method loop() ) has already converted topics to wire messages and stored these messages in a 2-stage buffer
    per message type. Because these message do not fit in a slave 'ACK' return message, the master must first be notified that a particular message is available.
    The 'ack' message that will be sent now informs the master that it needs to REQUEST to send that message, which will then be sent by the slave in a next 
    lock-step message exchange between wire master and slave.

    But to notify ALL potential subscribers of changed settings AND to be informed about changes made by the wire master (e.g., rounding PID values), the master
    must subsequently send these settings back to the wire slave (this bridge) for MQTT publishing.

    The logic to accomplish that last step is not built into the wire master. Instead, in a NEXT ack reply, the slave will now inform the master that it should send out 
    the changed settings.

    example:

    node-red                        wire master                     wire slave
    ---------------------------------------------------------------------------
                                                                            ////    
    TOPIC_GLOBE_SETTINGS_SET    ->  
    */

    if (_sharedContext.committedGlobeSettings.slaveHasData) {
        thisAckResponse.requestMasterMsgType = MsgType::M_MSG_GLOBE_SETTINGS_REQ;                           // THIS ack response: inform master it should request this data
        nextAckResponse.requestMasterMsgType = MsgType::M_MSG_GLOBE_SETTINGS;                               // NEXT ack response: inform master it should send out again this updated data
        _sharedContext.holdAckResponses.push(nextAckResponse);                                 // hold NEXT ack response until it's time to send the next ack
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


    // ---------- PRIO 2: the slave has a message (request for action, or data, from master) for wire master, FITTING in THIS ack response ?  ----------
    
    else if (!_sharedContext.holdAckResponses.empty()) {
        // THIS ack response is used to inform wire master that it should send data (a message type) or it should perform an action (e.g., visual ring) 
        thisAckResponse.requestMasterMsgType = _sharedContext.holdAckResponses.front()->requestMasterMsgType;
        thisAckResponse.action = _sharedContext.holdAckResponses.front()->action;
        Serial.print("Ack response - msg type "); Serial.print(thisAckResponse.requestMasterMsgType); Serial.print(", action "); Serial.println(thisAckResponse.action);
        I2C_s_ack dummy;
        _sharedContext.holdAckResponses.pop(dummy);
    }


    // ---------- return 'ack' message, containing optional master message type requested from master ----------

    I2C_s_ack p;
    p.ack = 0x0;        // not used
    p.requestMasterMsgType = thisAckResponse.requestMasterMsgType;
    p.action = thisAckResponse.action;
    wireSlave.pushOutgoingWireMsg(MsgType::S_MSG_ACK, &p, sizeof(p));

}


