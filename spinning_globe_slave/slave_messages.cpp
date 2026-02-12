#include "slave_messages.h"
#include "json_helpers.h"
#include "wire_hw_config.h"

WireSlaveMessages::WireSlaveMessages(SharedContext& sharedContext) : _sharedContext(sharedContext) {
};

bool WireSlaveMessages::loop() {

    uint8_t messageTypeIn{};          // as received
    uint8_t payloadSizeIn{};          // as received
    uint8_t payloadIn[MASTER_PAYLOAD_MAX];

    bool msgAvailable = wireSlave.popIncomingWireMsg(messageTypeIn, payloadIn, payloadSizeIn);
    if (!msgAvailable) { return false; }

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

        case MsgType::M_MSG_SECOND:
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
            replyAndFlagSlaveDataAvailable();
        }
        break;

        // receive coil phase adjust (degrees) from master
        case MsgType::M_MSG_COIL_PHASE_ADJUST:
        {
            I2C_m_coilPhaseAdjust* pCoilPhaseIn = reinterpret_cast<I2C_m_coilPhaseAdjust*>(payloadIn);
            replyAndFlagSlaveDataAvailable();
        }
        break;

        // ========== message types in: RECEIVE REQUEST from master to send data to master ==========

        case MsgType::M_MSG_GLOBE_SETTINGS_REQ:
        {
            // but master needs to observe a delay between this message it sent and the response expected, because this response will only be enqueued now
            // and must be ready in the queue when 'popOutgoingWireMsg()' is triggered  
            Serial.print("Phase 3: master requests globe settings. MQTT settings to be sent are valid: "); Serial.println(_sharedContext.committedGlobeSettings.slaveHasData);
            Serial.print("         MQTT globe settings now committed: set rot time index: "); Serial.println(_sharedContext.committedGlobeSettings.rotationPeriodIndex);
            Serial.print("         has slave data ?"); Serial.println((bool)_sharedContext.committedGlobeSettings.slaveHasData);
            wireSlave.pushOutgoingWireMsg(MsgType::S_MSG_GLOBE_SETTINGS_SET, &_sharedContext.committedGlobeSettings, sizeof(I2C_s_globeSettings_set));
            _sharedContext.committedGlobeSettings.slaveHasData = 0;
        }
        break;

        case MsgType::M_MSG_PID_SETTINGS_REQ:
        {
            I2C_s_PIDsettings_set p{};
            p.gainAdjustSteps = 16;      // test
            p.intTimeCstAdjustSteps = 16;
            p.difTimeCstAdjustSteps = 16;
            wireSlave.pushOutgoingWireMsg(S_MSG_PID_SETTINGS_SET, &p, sizeof(p));
        }
        break;

        case MsgType::M_MSG_VERT_POS_SETPOINT_REQ:
        {
            I2C_s_vertPosSetpoint_set p{};
            p.vertPosIndex = 0;      // test
            wireSlave.pushOutgoingWireMsg(S_MSG_VERT_POS_SETPOINT_SET, &p, sizeof(p));
        }
        break;

        case MsgType::M_MSG_COIL_PHASE_ADJUST_REQ:
        {
            I2C_s_coilPhaseAdjust_set p{};
            p.coilPhaseAdjust = 0;      // test
            wireSlave.pushOutgoingWireMsg(S_MSG_COIL_PHASE_ADJUST_SET, &p, sizeof(p));
        }
        break;

        default:
        {}  // unknown message type: do nothing (master will time out)
        break;

    }
    return true;
}


void WireSlaveMessages::convertGlobeStatusToMQTT(I2C_m_status* p) {
    MsgToMQTT msg{};
    snprintf(msg.topic, sizeof(msg.topic), TOPIC_STATUS);
    snprintf(msg.payload, sizeof(msg.payload), "%u", p->status);
    msg.retain = true;
    _sharedContext.queueToMQTT.push(msg);
};

void WireSlaveMessages::convertGlobeGreenwichCueToMQTT(I2C_m_greenwich* p) {
    MsgToMQTT msg{};
    // JSON
    snprintf(msg.topic, sizeof(msg.topic), TOPIC_GREENWICH);
    JsonAssemble::startJson(msg.payload, sizeof(msg.payload));
    if ((p->status == rotUnlocked) || (p->status == rotLocked)) {
        JsonAssemble::add(msg.payload, sizeof(msg.payload), "actRotTime", "\"%.2f s\"", p->actualRotationTime / 1000.);
    }
    else { JsonAssemble::add(msg.payload, sizeof(msg.payload), "actRotTime", "\"--.-- s\""); }
    if (p->status == rotLocked) {
        JsonAssemble::add(msg.payload, sizeof(msg.payload), "rotSyncError", "\"%.2f s\"", p->rotationOutOfSyncTime / 1000.);
        JsonAssemble::add(msg.payload, sizeof(msg.payload), "greenwichLag", "\"%.2ld degrees\"", p->greenwichLag);      // already in degrees
    }
    else {
        JsonAssemble::add(msg.payload, sizeof(msg.payload), "rotSyncError", "\"--.-- \"s");
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

    MsgToMQTT msg{};
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
    Serial.print("second cue: MQTT msg is "); Serial.print(msg.topic); Serial.print(", "); Serial.print(msg.payload); Serial.print(", "); Serial.println(msg.retain);
    _sharedContext.queueToMQTT.push(msg);
}

void WireSlaveMessages::convertGlobeSettingsToMQTT(I2C_m_globeSettings* pGlobeIn) {
    MsgToMQTT msg{};
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
    MsgToMQTT msg{};
    // JSON
    snprintf(msg.topic, sizeof(msg.topic), TOPIC_PID_SETTINGS);
    JsonAssemble::startJson(msg.payload, sizeof(msg.payload));
    JsonAssemble::add(msg.payload, sizeof(msg.payload), "gainAdjust", "%1d", pPIDIn->gainAdjustSteps);              // gain adjustment
    JsonAssemble::add(msg.payload, sizeof(msg.payload), "difTimeAdjust", "%1d", pPIDIn->difTimeCstAdjustSteps);     // diff. time constant adjustment
    JsonAssemble::add(msg.payload, sizeof(msg.payload), "intTimeAdjust", "%1d", pPIDIn->intTimeCstAdjustSteps);     // int. time constant adjustment
    JsonAssemble::closeJson(msg.payload, sizeof(msg.payload));
    msg.retain = true;
    _sharedContext.queueToMQTT.push(msg);
};


// ============================================================================================
// Reply to wire master with an 'ACK' message
// If an MQTT message is pending and NO MQTT message is committed to be sent to wire master:
//   MQTT message - PHASE 2: commit pending message and lag 'message is committed' 
//   (next phase: phase 3: answer to wire master requesting data)
// ============================================================================================
void WireSlaveMessages::replyAndFlagSlaveDataAvailable() {

    MsgType msgType{ M_MSG_NONE };        // init: no message req

    if (_sharedContext.pendingGlobeSettings.slaveHasData && !_sharedContext.committedGlobeSettings.slaveHasData) {
        _sharedContext.committedGlobeSettings = _sharedContext.pendingGlobeSettings;        // also sets '.hasSlaveData' to '1'
        _sharedContext.pendingGlobeSettings.slaveHasData = 0;       // was just committed
        msgType = MsgType::M_MSG_GLOBE_SETTINGS_REQ;        // inform master: should request globe settings
        Serial.print("Phase 2: MQTT globe settings now committed: set rot time index: ");Serial.println(_sharedContext.committedGlobeSettings.rotationPeriodIndex);
        Serial.print("         has slave data ?"); Serial.println((bool)_sharedContext.committedGlobeSettings.slaveHasData);
    }

    /* later ////
    else if (_sharedContext.pendingGlobeSettingsFull && !_sharedContext.committedGlobeSettingsFull) {
        _sharedContext.committedGlobeSettings = _sharedContext.pendingGlobeSettings;
        _sharedContext.pendingGlobeSettingsFull = false;
        _sharedContext.committedGlobeSettingsFull = true;
        msgType = ;
    }

    else if (_sharedContext.pendingGlobeSettingsFull && !_sharedContext.committedGlobeSettingsFull) {
        _sharedContext.committedGlobeSettings = _sharedContext.pendingGlobeSettings;
        _sharedContext.pendingGlobeSettingsFull = false;
        _sharedContext.committedGlobeSettingsFull = true;
        msgType = ;
    }

    else if (_sharedContext.pendingGlobeSettingsFull && !_sharedContext.committedGlobeSettingsFull) {
        _sharedContext.committedGlobeSettings = _sharedContext.pendingGlobeSettings;
        _sharedContext.pendingGlobeSettingsFull = false;
        _sharedContext.committedGlobeSettingsFull = true;
        msgType = ;
    }
    */

    I2C_s_ack p;
    p.ack = 0x0;        // not used
    p.requestMasterMsgType = msgType;
    if(p.requestMasterMsgType == MsgType::M_MSG_GLOBE_SETTINGS_REQ) {Serial.print("(start)  MQTT globe settings now committed: "); Serial.println((bool)_sharedContext.committedGlobeSettings.slaveHasData);}
    wireSlave.pushOutgoingWireMsg(MsgType::S_MSG_ACK, &p, sizeof(p));
    if (p.requestMasterMsgType == MsgType::M_MSG_GLOBE_SETTINGS_REQ) { Serial.print("(end)    MQTT globe settings now committed: "); Serial.println((bool)_sharedContext.committedGlobeSettings.slaveHasData); }
}




