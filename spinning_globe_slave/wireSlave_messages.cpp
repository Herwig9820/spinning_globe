#include "wireSlave_messages.h"
#include "json_helpers.h"

WireSlaveMessages::WireSlaveMessages(SharedContext& sharedContext) : _sharedContext(sharedContext) {
};

bool WireSlaveMessages::loop() {

    uint8_t messageTypeIn{};          // as received
    uint8_t payloadSizeIn{};          // as received
    uint8_t payloadIn[WireSlave::PAYLOAD_IN_MAX];

    bool msgAvailable = wireSlave.dequeueRx(messageTypeIn, payloadIn, payloadSizeIn);
    if (!msgAvailable) { return false; }

    Serial.print("message: "); Serial.println(messageTypeIn, HEX);// for now

    switch (messageTypeIn)
    {
        case MsgType::M_MSG_HELLO:  // not yet implemented
        {
            wireSlave.enqueueTx(MsgType::S_MSG_HELLO_ACK, nullptr, 0);             // no payload
        }
        break;


       // ========== message types in: RECEIVE DATA from wire master after an event ==========

        case MsgType::M_MSG_STATUS:
        {
            I2C_m_status* pStatus = reinterpret_cast<I2C_m_status*>(payloadIn);
            convertGlobeStatusToMQTT(pStatus);

            I2C_s_ack p;
            p.ack = 0x0;
            p.requestMasterMsgType = MsgType::M_MSG_NONE;
            wireSlave.enqueueTx(MsgType::S_MSG_ACK, &p, sizeof(p));
        }
        break;

        case MsgType::M_MSG_GREENWICH:
        {
            I2C_m_greenwich* pGreenwich = reinterpret_cast<I2C_m_greenwich*>(payloadIn);
            convertGlobeGreenwichCueToMQTT(pGreenwich);

            I2C_s_ack p;
            p.ack = 0x0;
            p.requestMasterMsgType = MsgType::M_MSG_NONE;
            wireSlave.enqueueTx(MsgType::S_MSG_ACK, &p, sizeof(p));
        }
        break;

        case MsgType::M_MSG_SECOND:
        {
            I2C_m_secondCue* pSecondCue = reinterpret_cast<I2C_m_secondCue*>(payloadIn);
            convertSecondCueToMQTT(pSecondCue);

            I2C_s_ack p;
            p.ack = 0x0;
            p.requestMasterMsgType = MsgType::M_MSG_NONE;
            wireSlave.enqueueTx(MsgType::S_MSG_ACK, &p, sizeof(p));
        }
        break;


        // ========== message types in: RECEIVE DATA from wire master when requested by slave ==========

        // receive wire master library tx stats 
        case MsgType::M_MSG_SEND_STATS:
        {
            I2C_s_ack p;
            p.ack = 0x0;
            p.requestMasterMsgType = MsgType::M_MSG_NONE;
            wireSlave.enqueueTx(MsgType::S_MSG_ACK, &p, sizeof(p));
        }
        break;

        // receive wire master library rx stats
        case MsgType::M_MSG_RECEIVE_STATS:
        {
            I2C_s_ack p;
            p.ack = 0x0;
            p.requestMasterMsgType = MsgType::M_MSG_NONE;
            wireSlave.enqueueTx(MsgType::S_MSG_ACK, &p, sizeof(p));
        }
        break;

        // receive wire master message library stats
        case MsgType::M_MSG_MESSAGE_STATS:
        {
            I2C_s_ack p;
            p.ack = 0x0;
            p.requestMasterMsgType = MsgType::M_MSG_NONE;
            wireSlave.enqueueTx(MsgType::S_MSG_ACK, &p, sizeof(p));
        }
        break;

        // receive spinning globe event stats
        case MsgType::M_MSG_GLOBE_STATS:
        {

            I2C_s_ack p;
            p.ack = 0x0;
            p.requestMasterMsgType = MsgType::M_MSG_NONE;
            wireSlave.enqueueTx(MsgType::S_MSG_ACK, &p, sizeof(p));
        }
        break;

        // receive globe settings (changeable globe attributes)
        case MsgType::M_MSG_GLOBE_SETTINGS:
        {
            I2C_s_globeSettings* globeIn = reinterpret_cast<I2C_s_globeSettings*>(payloadIn);
            if (payloadSizeIn != sizeof(*globeIn)) { Serial.println("PAYLOAD SIZE <<<<<<"); return false; }               // inconsistency between master and slave: forget message
            Serial.println("Globe settings: adjust steps (received):");
            Serial.print("    rotation time   "); Serial.println(globeIn->userSet_rotationPeriod);
            Serial.print("    led effect      "); Serial.println(globeIn->userSet_ledEffect);
            Serial.print("    led cycle speed "); Serial.println(globeIn->userSet_ledCycleSpeed);

            I2C_s_ack p;
            p.ack = 0x0;
            p.requestMasterMsgType = MsgType::M_MSG_NONE;
            wireSlave.enqueueTx(MsgType::S_MSG_ACK, &p, sizeof(p));
        }
        break;

        // receive PID controller settings
        case MsgType::M_MSG_PID_SETTINGS:
        {
            I2C_s_PIDsettings* PIDin = reinterpret_cast<I2C_s_PIDsettings*>(payloadIn);
            if (payloadSizeIn != sizeof(*PIDin)) { Serial.println("PAYLOAD SIZE <<<<<<"); return false; }               // inconsistency between master and slave: forget message
            Serial.println("PID controller: adjust steps (received):");
            Serial.print("    gain          "); Serial.println(PIDin->gainAdjustSteps);
            Serial.print("    int. time cst "); Serial.println(PIDin->intTimeCstAdjustSteps);
            Serial.print("    dif. time cst "); Serial.println(PIDin->difTimeCstAdjustSteps);

            I2C_s_ack p;
            p.ack = 0x0;
            p.requestMasterMsgType = MsgType::M_MSG_NONE;
            wireSlave.enqueueTx(MsgType::S_MSG_ACK, &p, sizeof(p));
        }
        break;

        // receive vertical position setpoint (mV)
        case MsgType::M_MSG_VERT_POS_SETPOINT:
        {
            I2C_s_vertPosSetpoint* vertPosIn = reinterpret_cast<I2C_s_vertPosSetpoint*>(payloadIn);
            if (payloadSizeIn != sizeof(*vertPosIn)) { Serial.println("PAYLOAD SIZE <<<<<<"); return false; }               // inconsistency between master and slave: forget message
            Serial.print("vert. pos. setpoint "); Serial.println(vertPosIn->userSet_vertPosIndex);

            I2C_s_ack p;
            p.ack = 0x0;
            p.requestMasterMsgType = MsgType::M_MSG_NONE;
            wireSlave.enqueueTx(MsgType::S_MSG_ACK, &p, sizeof(p));
        }
        break;

        // receive coil phase adjust (degrees)
        case MsgType::M_MSG_COIL_PHASE_ADJUST:
        {
            I2C_s_coilPhaseAdjust* coilPhaseIn = reinterpret_cast<I2C_s_coilPhaseAdjust*>(payloadIn);
            if (payloadSizeIn != sizeof(*coilPhaseIn)) { Serial.println("PAYLOAD SIZE <<<<<<"); return false; }               // inconsistency between master and slave: forget message
            Serial.print("coil phase adjust: "); Serial.println(coilPhaseIn->userSet_coilPhaseAdjust);

            I2C_s_ack p;
            p.ack = 0x0;
            p.requestMasterMsgType = MsgType::M_MSG_NONE;
            wireSlave.enqueueTx(MsgType::S_MSG_ACK, &p, sizeof(p));
        }
        break;

        // ========== message types in: RECEIVE REQUEST from master to send data ==========

        case MsgType::M_MSG_REQ_GLOBE_SETTINGS:
        {
            I2C_s_globeSettings p{};
            p.userSet_rotationPeriod = 5;      // test
            p.userSet_ledEffect = 5;
            p.userSet_ledCycleSpeed = 0;
            // but master needs to observe a delay between this message it sent and the response expected, because this response will only be enqueued now
            // and must be ready in the queue when 'handleRequest()' is triggered  
            wireSlave.enqueueTx(MsgType::S_MSG_GLOBE_SETTINGS, &p, sizeof(p));
        }
        break;

        case MsgType::M_MSG_REQ_PID_SETTINGS:
        {
            I2C_s_PIDsettings p{};
            p.gainAdjustSteps = 16;      // test
            p.intTimeCstAdjustSteps = 16;
            p.difTimeCstAdjustSteps = 16;
            wireSlave.enqueueTx(S_MSG_PID_SETTINGS, &p, sizeof(p));
        }
        break;

        case MsgType::M_MSG_REQ_VERT_POS_SETPOINT:
        {
            I2C_s_vertPosSetpoint p{};
            p.userSet_vertPosIndex = 0;      // test
            wireSlave.enqueueTx(S_MSG_VERT_POS_SETPOINT, &p, sizeof(p));
        }
        break;

        case MsgType::M_MSG_REQ_COIL_PHASE_ADJUST:
        {
            I2C_s_coilPhaseAdjust p{};
            p.userSet_coilPhaseAdjust = 0;      // test
            wireSlave.enqueueTx(S_MSG_COIL_PHASE_ADJUST, &p, sizeof(p));
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

    _sharedContext.queueToMQTT.push(msg);

    snprintf(msg.topic, sizeof(msg.topic), TOPIC_GREENWICH);
    JsonUtil::startJson(msg.payload, sizeof(msg.payload));
    JsonUtil::add(msg.payload, sizeof(msg.payload), "actRotTime", "--.-- s");
    JsonUtil::add(msg.payload, sizeof(msg.payload), "rotSyncError", "--.-- s");
    JsonUtil::add(msg.payload, sizeof(msg.payload), "greenwichLag", "--- degrees");
    JsonUtil::closeJson(msg.payload, sizeof(msg.payload));

    _sharedContext.queueToMQTT.push(msg);
};

void WireSlaveMessages::convertGlobeGreenwichCueToMQTT(I2C_m_greenwich* p) {

    MsgToMQTT msg{};

    // JSON serialization into fixed buffer
    snprintf(msg.topic, sizeof(msg.topic), TOPIC_GREENWICH);
    JsonUtil::startJson(msg.payload, sizeof(msg.payload));
    JsonUtil::add(msg.payload, sizeof(msg.payload), "actRotTime", "%.2f s", p->actualRotationTime/1000.);
    if (p->status == rotLocked) {
        JsonUtil::add(msg.payload, sizeof(msg.payload), "rotSyncError", "%.2f s", p->rotationOutOfSyncTime/1000.);
        JsonUtil::add(msg.payload, sizeof(msg.payload), "greenwichLag", "%.2ld degrees", p->greenwichLag);
    }
    else {
        JsonUtil::add(msg.payload, sizeof(msg.payload), "rotSyncError", "--.-- s");
        JsonUtil::add(msg.payload, sizeof(msg.payload), "greenwichLag", "--- degrees");
    }
    JsonUtil::closeJson(msg.payload, sizeof(msg.payload));

    _sharedContext.queueToMQTT.push(msg);
};

void WireSlaveMessages::convertSecondCueToMQTT(I2C_m_secondCue* p) {
    float tempC = p->tempSmooth / 100.0f;
    float magnetDutyCycle = (p->magnetOnCyclesSmooth / spinningGlobeNano_fastDataRateSamplingPeriods) * 100.0f / (float)spinningGlobeNano_timer1Top;
    float isrDuration = p->ISRdurationSmooth / spinningGlobeNano_fastDataRateSamplingPeriods;
    float load = ((1000.0f * spinningGlobeNano_fastDataRateSamplingPeriods - p->idleLoopMicrosSmooth)
        * 100.0f / (1000.0f * spinningGlobeNano_fastDataRateSamplingPeriods));
    float vertPosAvgError = (p->errSignalMagnitudeSmooth / spinningGlobeNano_fastDataRateSamplingPeriods) * spinningGlobeNano_ADCmVperStep;

    MsgToMQTT msg{};

    // JSON serialization into fixed buffer
    snprintf(msg.topic, sizeof(msg.topic), TOPIC_TELEMETRY);
    JsonUtil::startJson(msg.payload, sizeof(msg.payload));
    JsonUtil::add(msg.payload, sizeof(msg.payload), "temp", "%.1f deg.C", tempC);
    JsonUtil::add(msg.payload, sizeof(msg.payload), "magnetDutyCycle", "%.1f%%", magnetDutyCycle);
    JsonUtil::add(msg.payload, sizeof(msg.payload), "ISRduration", "%.0f us", isrDuration);
    JsonUtil::add(msg.payload, sizeof(msg.payload), "load", "%.1f%%", load);
    JsonUtil::add(msg.payload, sizeof(msg.payload), "vertPosError", "%.1f mV", vertPosAvgError);
    JsonUtil::closeJson(msg.payload, sizeof(msg.payload));

    _sharedContext.queueToMQTT.push(msg);
}




