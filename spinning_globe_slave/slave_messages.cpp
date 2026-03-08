#include "slave_messages.h"
#include "json_helpers.h"
#include "wire_hw_config.h"

WireSlaveMessages::WireSlaveMessages(SharedContext& sharedContext) : _sharedContext(sharedContext) {
};

bool WireSlaveMessages::loop() {

    uint8_t messageTypeIn{};          // as received
    uint8_t payloadSizeIn{};          // as received
    uint8_t payloadIn[MASTER_PAYLOAD_MAX];

    // switch of 'wire data received' LED if no data is received for a set time
    if(_wireLedOn){
        if (millis() - _ledOnStart > 15) {                                                  // say on for 15 ms after last data was received
            _wireLedOn = false;
            digitalWrite(WIRE_RECEIVE_LED, false);
        }
    }

    bool msgAvailable = wireSlave.popIncomingWireMsg(messageTypeIn, payloadIn, payloadSizeIn);
    if (!msgAvailable) { return false; }

    // signal that wire data is received
    _wireLedOn = true;
    digitalWrite(WIRE_RECEIVE_LED, true);
    _ledOnStart = millis();

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
            I2C_m_secondCue* pSecondCue = reinterpret_cast<I2C_m_secondCue*>(payloadIn);
            convertSecondCueToMQTT(pSecondCue);
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

        // receive spinning globe event stats
        case MsgType::M_MSG_GLOBE_STATS:
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
            Serial.println("PID settings - phase 3");
            wireSlave.pushOutgoingWireMsg(MsgType::S_MSG_PID_SETTINGS_SET, &_sharedContext.committedPIDsettings, sizeof(I2C_s_PIDsettings_set));
            _sharedContext.committedPIDsettings.slaveHasData = 0;
        }
        break;

        case MsgType::M_MSG_VERT_POS_SETPOINT_REQ:
        {
            Serial.println("vertical position - phase 3");
            wireSlave.pushOutgoingWireMsg(S_MSG_VERT_POS_SETPOINT_SET, &_sharedContext.committedVertPosSetpoint, sizeof(I2C_s_vertPosSetpoint_set));
            _sharedContext.committedVertPosSetpoint.slaveHasData = 0;
        }
        break;

        case MsgType::M_MSG_COIL_PHASE_ADJUST_REQ:
        {
            Serial.println("coil phase adjust - phase 3");
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
        JsonAssemble::add(msg.payload, sizeof(msg.payload), "actRotTime", "\"%.2f s\"", p->actualRotationTime / 1000.);
    }
    else { JsonAssemble::add(msg.payload, sizeof(msg.payload), "actRotTime", "\"--.-- s\""); Serial.println("**** B"); }

    if (p->status == rotLocked) {
        JsonAssemble::add(msg.payload, sizeof(msg.payload), "rotSyncError", "\"%.2f s\"", p->rotationOutOfSyncTime / 1000.);
        JsonAssemble::add(msg.payload, sizeof(msg.payload), "greenwichLag", "\"%.2ld degrees\"", p->greenwichLag);      // already in degrees
    }
    else {
        JsonAssemble::add(msg.payload, sizeof(msg.payload), "rotSyncError", "\"--.-- s\"");
        JsonAssemble::add(msg.payload, sizeof(msg.payload), "greenwichLag", "\"--- degrees\"");
    }
    JsonAssemble::closeJson(msg.payload, sizeof(msg.payload));
    msg.retain = true;
    _sharedContext.queueToMQTT.push(msg);
};

void WireSlaveMessages::convertSecondCueToMQTT(I2C_m_secondCue* p) {
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
    JsonAssemble::add(msg.payload, sizeof(msg.payload), "heatsinkTemp", "\"%.1f deg.C\"", tempC);
    JsonAssemble::add(msg.payload, sizeof(msg.payload), "magnetDutyCycle", "\"%.1f%%\"", magnetDutyCycle);
    JsonAssemble::add(msg.payload, sizeof(msg.payload), "ISRduration", "\"%.0f us\"", isrDuration);
    JsonAssemble::add(msg.payload, sizeof(msg.payload), "load", "\"%.1f%%\"", load);
    JsonAssemble::add(msg.payload, sizeof(msg.payload), "vertPosError", "\"%.1f mV\"", vertPosAvgError);
    JsonAssemble::add(msg.payload, sizeof(msg.payload), "TTTintegrTerm", "\"%6ld\"", p->realTTTintegrationTerm);
    JsonAssemble::closeJson(msg.payload, sizeof(msg.payload));
    msg.retain = true;
    _sharedContext.queueToMQTT.push(msg);
}

void WireSlaveMessages::convertGlobeSettingsToMQTT(I2C_m_globeSettings* pGlobeIn) {
    MQTTmsgFromWire msg{};
    // JSON
    snprintf(msg.topic, sizeof(msg.topic), TOPIC_GLOBE_SETTINGS);
    JsonAssemble::startJson(msg.payload, sizeof(msg.payload));
    JsonAssemble::add(msg.payload, sizeof(msg.payload), "setRotTime", "\"%1u\"", pGlobeIn->rotationPeriodIndex);    // rotation speed
    JsonAssemble::add(msg.payload, sizeof(msg.payload), "setLedEffect", "\"%1u\"", pGlobeIn->ledEffect);            // led effect
    JsonAssemble::add(msg.payload, sizeof(msg.payload), "setLedEffectSpeed", "\"%1u\"", pGlobeIn->ledCycleSpeed);   // led effect speed
    JsonAssemble::closeJson(msg.payload, sizeof(msg.payload));
    msg.retain = true;
    _sharedContext.queueToMQTT.push(msg);
};

void WireSlaveMessages::convertPIDsettingsToMQTT(I2C_m_PIDsettings* pPIDIn) {
    MQTTmsgFromWire msg{};
    // JSON
    snprintf(msg.topic, sizeof(msg.topic), TOPIC_PID_SETTINGS);
    JsonAssemble::startJson(msg.payload, sizeof(msg.payload));
    JsonAssemble::add(msg.payload, sizeof(msg.payload), "setGainAdjust", "%1d", pPIDIn->gainAdjustSteps);              // gain adjustment
    JsonAssemble::add(msg.payload, sizeof(msg.payload), "setDifTimeAdjust", "%1d", pPIDIn->difTimeCstAdjustSteps);     // diff. time constant adjustment
    JsonAssemble::add(msg.payload, sizeof(msg.payload), "setIntTimeAdjust", "%1d", pPIDIn->intTimeCstAdjustSteps);     // int. time constant adjustment
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
    Serial.print("Wire To MQTT: coil phase "); Serial.println(msg.payload);
    snprintf(msg.topic, sizeof(msg.topic), TOPIC_COIL_PHASE_ADJUST);
    snprintf(msg.payload, sizeof(msg.payload), "%1d", pCoilPhaseIn->coilPhaseAdjust);
    msg.retain = true;
    _sharedContext.queueToMQTT.push(msg);
};


// ============================================================================================
// Reply to wire master with an 'ACK' message
// If an MQTT message is pending and NO MQTT message is committed to be sent to wire master:
//   MQTT message - PHASE 2: commit pending message and log 'message is committed' 
//   (next phase: phase 3: answer to wire master requesting data)
// ============================================================================================
void WireSlaveMessages::replyAndFlagSlaveDataAvailable() {

    MsgType msgType{ M_MSG_NONE };        // init: no message req

    // Each time an 'ack' MsgType is sent to the wire master, the slave can request the master to send a specific message type.
    // If wire slave:
    // PRIO 1: has DATA AVAILABLE for wire master: the master is REQUESTED TO send a message type REQUESTING the specific data available 
    // PRIO 2: REQUESTS DATA from wire master    : the master is REQUESTED TO send a message type CONTAINING the specific data 



    if (_sharedContext.pendingVertPosSetpoint.slaveHasData) { Serial.print("********** vert pos: pending 1, committed "); Serial.println(_sharedContext.committedVertPosSetpoint.slaveHasData); }
    if (_sharedContext.pendingCoilPhaseAdjust.slaveHasData) { Serial.print("********** coil phase pending 1, committed "); Serial.println(_sharedContext.committedCoilPhaseAdjust.slaveHasData); }



    // ----------  data (settings, ...) to send ? commit ----------

    if (_sharedContext.pendingGlobeSettings.slaveHasData && !_sharedContext.committedGlobeSettings.slaveHasData) {
        Serial.println("     globe: phase 2");
        _sharedContext.committedGlobeSettings = _sharedContext.pendingGlobeSettings;        // this also sets '.hasSlaveData' to '1'
        _sharedContext.pendingGlobeSettings.slaveHasData = 0;                               // because it was just committed
    }

    else if (_sharedContext.pendingPIDsettings.slaveHasData && !_sharedContext.committedPIDsettings.slaveHasData) {
        Serial.println("     PID: phase 2");
        _sharedContext.committedPIDsettings = _sharedContext.pendingPIDsettings;        // this also sets '.hasSlaveData' to '1'
        _sharedContext.pendingPIDsettings.slaveHasData = 0;                               // because it was just committed
    }

    else if (_sharedContext.pendingVertPosSetpoint.slaveHasData && !_sharedContext.committedVertPosSetpoint.slaveHasData) {
        Serial.println("     vert.position: phase 2");
        _sharedContext.committedVertPosSetpoint = _sharedContext.pendingVertPosSetpoint;        // this also sets '.hasSlaveData' to '1'
        _sharedContext.pendingVertPosSetpoint.slaveHasData = 0;                                 // because it was just committed
    }

    else if (_sharedContext.pendingCoilPhaseAdjust.slaveHasData && !_sharedContext.committedCoilPhaseAdjust.slaveHasData) {
        Serial.println("     coil phase: phase 2");
        _sharedContext.committedCoilPhaseAdjust = _sharedContext.pendingCoilPhaseAdjust;        // this also sets '.hasSlaveData' to '1'
        _sharedContext.pendingCoilPhaseAdjust.slaveHasData = 0;                               // because it was just committed
    }


    // ---------- PRIO 1: data (settings, ...) to submit to master ? (will include a readback to distribute changed settings to all subscribed clients) ----------

    if (_sharedContext.committedGlobeSettings.slaveHasData) {
        msgType = MsgType::M_MSG_GLOBE_SETTINGS_REQ;                                        // inform master: should request globe settings
        _sharedContext.requestDataFromMaster.push(MsgType::M_MSG_GLOBE_SETTINGS);        // push readback msg type
    }

    else if (_sharedContext.committedPIDsettings.slaveHasData) {
        msgType = MsgType::M_MSG_PID_SETTINGS_REQ;                                        // inform master: should request globe settings
        _sharedContext.requestDataFromMaster.push(MsgType::M_MSG_PID_SETTINGS);        // push readback msg type
    }

    else if (_sharedContext.committedVertPosSetpoint.slaveHasData) {
        msgType = MsgType::M_MSG_VERT_POS_SETPOINT_REQ;                                         // inform master: should request globe settings
        _sharedContext.requestDataFromMaster.push(MsgType::M_MSG_VERT_POS_SETPOINT);            // push readback msg type
    }

    else if (_sharedContext.committedCoilPhaseAdjust.slaveHasData) {
        msgType = MsgType::M_MSG_COIL_PHASE_ADJUST_REQ;                                        // inform master: should request globe settings
        _sharedContext.requestDataFromMaster.push(MsgType::M_MSG_COIL_PHASE_ADJUST);        // push readback msg type
    }


    // ---------- PRIO 2: no data to submit to master. Slave requests data from master ?  ----------

    else if (!_sharedContext.requestDataFromMaster.empty()) {
        MsgType dummyMsgType;
        msgType = *_sharedContext.requestDataFromMaster.front();
        _sharedContext.requestDataFromMaster.pop(dummyMsgType);
    }


    // ---------- return 'ack' message, containing optional master message type requested from master ----------

    I2C_s_ack p;
    p.ack = 0x0;        // not used
    p.requestMasterMsgType = msgType;
    wireSlave.pushOutgoingWireMsg(MsgType::S_MSG_ACK, &p, sizeof(p));
}


