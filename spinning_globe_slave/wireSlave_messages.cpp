#include "wireSlave_messages.h"
#include "json_helpers.h"

WireSlaveMessages::WireSlaveMessages(SharedContext& sharedContext) : _sharedContext(sharedContext) {
};

bool WireSlaveMessages::loop() {

    uint8_t messageTypeIn{};          // as received
    uint8_t payloadSizeIn{};          // as received
    uint8_t plIn[WireSlave::PAYLOAD_IN_MAX];

    bool msgAvailable = wireSlave.dequeueRx(messageTypeIn, plIn, payloadSizeIn);
    if (!msgAvailable) { return false; }

    Serial.print("message: "); Serial.println(messageTypeIn, HEX);// for now

    switch (messageTypeIn)
    {
        case MsgType::M_MSG_HELLO:
        {
            Serial.println("\n\nhello received");
            wireSlave.enqueueTx(MsgType::S_MSG_HELLO_ACK, nullptr, 0);             // no payload
        }
        break;


       // ========== message types in: RECEIVE DATA from master after an event ==========

        case MsgType::M_MSG_GREENWICH:
        {
            Serial.println("\n\nGreenwich received");

            I2C_s_ack p;
            p.ack = 0x0; //// 0; //dummy
            p.requestMasterMsgType = MsgType::M_MSG_NONE;
            wireSlave.enqueueTx(MsgType::S_MSG_ACK, &p, sizeof(p));
        }
        break;

        case MsgType::M_MSG_STATUS:
        {
            Serial.println("\n\nStatus received");

            I2C_s_ack p;
            p.ack = 0x0; //// 0; //dummy
            p.requestMasterMsgType = MsgType::M_MSG_NONE;
            wireSlave.enqueueTx(MsgType::S_MSG_ACK, &p, sizeof(p));
        }
        break;

        case MsgType::M_MSG_SECOND:
        {
            Serial.println("\n\rSecond received");
            I2C_m_secondCue* pSecondCue = reinterpret_cast<I2C_m_secondCue*>(plIn);
            convertSecondCueToMQTT(pSecondCue);
            Serial.println("Second converted to MQTT");

            I2C_s_ack p;
            p.ack = 0x0; //// seq low byte //// 0; //dummy
            p.requestMasterMsgType = MsgType::M_MSG_NONE; //// MsgType::M_MSG_REQ_GLOBE_SETTINGS; ////
            wireSlave.enqueueTx(MsgType::S_MSG_ACK, &p, sizeof(p));
        }
        break;


        // ========== message types in: RECEIVE DATA from master when requested by slave ==========

        // receive wire master library tx stats 
        case MsgType::M_MSG_SEND_STATS:
        {
            I2C_s_ack p;
            p.ack = 0x0; //// 0; //dummy
            p.requestMasterMsgType = MsgType::M_MSG_NONE;
            wireSlave.enqueueTx(MsgType::S_MSG_ACK, &p, sizeof(p));
        }
        break;

        // receive wire master library rx stats
        case MsgType::M_MSG_RECEIVE_STATS:
        {
            I2C_s_ack p;
            p.ack = 0x0; //// 0; //dummy
            p.requestMasterMsgType = MsgType::M_MSG_NONE;
            wireSlave.enqueueTx(MsgType::S_MSG_ACK, &p, sizeof(p));
        }
        break;

        // receive wire master message library stats
        case MsgType::M_MSG_MESSAGE_STATS:
        {
            I2C_s_ack p;
            p.ack = 0x0; //// 0; //dummy
            p.requestMasterMsgType = MsgType::M_MSG_NONE;
            wireSlave.enqueueTx(MsgType::S_MSG_ACK, &p, sizeof(p));
        }
        break;

        // receive spinning globe event stats
        case MsgType::M_MSG_GLOBE_STATS:
        {

            I2C_s_ack p;
            p.ack = 0x0; //// 0; //dummy
            p.requestMasterMsgType = MsgType::M_MSG_NONE;
            wireSlave.enqueueTx(MsgType::S_MSG_ACK, &p, sizeof(p));
        }
        break;

        // receive globe settings (changeable globe attributes)
        case MsgType::M_MSG_GLOBE_SETTINGS:
        {
            I2C_s_globeSettings* globeIn = reinterpret_cast<I2C_s_globeSettings*>(plIn);
            if (payloadSizeIn != sizeof(*globeIn)) { Serial.println("PAYLOAD SIZE <<<<<<"); return false; }               // inconsistency between master and slave: forget message
            Serial.println("Globe settings: adjust steps (received):");
            Serial.print("    rotation time   "); Serial.println(globeIn->userSet_rotationPeriod);
            Serial.print("    led effect      "); Serial.println(globeIn->userSet_ledEffect);
            Serial.print("    led cycle speed "); Serial.println(globeIn->userSet_ledCycleSpeed);

            I2C_s_ack p;
            p.ack = 0x0; //// 0; //dummy
            p.requestMasterMsgType = MsgType::M_MSG_NONE;
            wireSlave.enqueueTx(MsgType::S_MSG_ACK, &p, sizeof(p));
        }
        break;

        // receive PID controller settings
        case MsgType::M_MSG_PID_SETTINGS:
        {
            I2C_s_PIDsettings* PIDin = reinterpret_cast<I2C_s_PIDsettings*>(plIn);
            if (payloadSizeIn != sizeof(*PIDin)) { Serial.println("PAYLOAD SIZE <<<<<<"); return false; }               // inconsistency between master and slave: forget message
            Serial.println("PID controller: adjust steps (received):");
            Serial.print("    gain          "); Serial.println(PIDin->gainAdjustSteps);
            Serial.print("    int. time cst "); Serial.println(PIDin->intTimeCstAdjustSteps);
            Serial.print("    dif. time cst "); Serial.println(PIDin->difTimeCstAdjustSteps);

            I2C_s_ack p;
            p.ack = 0x0; //// 0; //dummy
            p.requestMasterMsgType = MsgType::M_MSG_NONE;
            wireSlave.enqueueTx(MsgType::S_MSG_ACK, &p, sizeof(p));
        }
        break;

        // receive vertical position setpoint (mV)
        case MsgType::M_MSG_VERT_POS_SETPOINT:
        {
            I2C_s_vertPosSetpoint* vertPosIn = reinterpret_cast<I2C_s_vertPosSetpoint*>(plIn);
            if (payloadSizeIn != sizeof(*vertPosIn)) { Serial.println("PAYLOAD SIZE <<<<<<"); return false; }               // inconsistency between master and slave: forget message
            Serial.print("vert. pos. setpoint "); Serial.println(vertPosIn->userSet_vertPosIndex);

            I2C_s_ack p;
            p.ack = 0x0; //// 0; //dummy
            p.requestMasterMsgType = MsgType::M_MSG_NONE;
            wireSlave.enqueueTx(MsgType::S_MSG_ACK, &p, sizeof(p));
        }
        break;

        // receive coil phase adjust (degrees)
        case MsgType::M_MSG_COIL_PHASE_ADJUST:
        {
            I2C_s_coilPhaseAdjust* coilPhaseIn = reinterpret_cast<I2C_s_coilPhaseAdjust*>(plIn);
            if (payloadSizeIn != sizeof(*coilPhaseIn)) { Serial.println("PAYLOAD SIZE <<<<<<"); return false; }               // inconsistency between master and slave: forget message
            Serial.print("coil phase adjust: "); Serial.println(coilPhaseIn->userSet_coilPhaseAdjust);

            I2C_s_ack p;
            p.ack = 0x0; //// 0; //dummy
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




/*
// ============================================================================
// SAFE FLOAT FORMATTER (no dtostrf, no snprintf)
// ============================================================================
// ---------------------------------------------------------------------------
// Safe, robust formatter for floats with rounding and no %f dependency.
// Writes into 'out' with size 'outSize'.
// Example: formatFloat(out, 3.14159, 2) -> "3.14"
// ---------------------------------------------------------------------------
// Writes the formatted float into `out`, guaranteed null-terminated.
// outSize must be >= 4 (for "-0.0\0").
// decimals: number of digits after decimal point (0..6).
void WireSlaveMessages::formatFloat(char* out, size_t outSize, float value, uint8_t decimals)
{
    if (outSize == 0) return;          // no room
    out[0] = '\0';                     // always produce valid string

    if (decimals > 6) decimals = 6;    // safe limit

    bool neg = (value < 0);
    if (neg) value = -value;

    // multiplier = 10^decimals
    uint32_t mul = 1;
    for (uint8_t i = 0; i < decimals; i++)
        mul *= 10U;

    // scaled and rounded integer representation
    uint32_t scaled = (uint32_t)(value * mul + 0.5f);

    uint32_t whole = scaled / mul;
    uint32_t frac = scaled % mul;

    // --- Build the string manually ---
    char temp[32];      // local build buffer — safe on ARM
    size_t idx = 0;

    // sign
    if (neg) temp[idx++] = '-';

    // whole part
    idx += snprintf(temp + idx, sizeof(temp) - idx, "%lu", (unsigned long)whole);

    // fractional part
    if (decimals > 0) {
        temp[idx++] = '.';

        // Leading zeros for fractional part
        uint32_t t = mul / 10;
        while (t > 1 && frac < t) {
            temp[idx++] = '0';
            t /= 10;
        }

        // actual fractional value
        idx += snprintf(temp + idx, sizeof(temp) - idx, "%lu", (unsigned long)frac);
    }

    temp[idx] = '\0';

    // Copy to user buffer safely
    strncpy(out, temp, outSize - 1);
    out[outSize - 1] = '\0';
}
*/

