#include "floatingGlobeState.h"
#include "messageHandling.h"

MessageHandling::MessageHandling(GreenwichData& greenwichData, StatusData& statusData, SecondData& secondData,
    SmoothedMeasurements& smoothedMeasurements, PIDsettings& pidSettings, ParamSettings& paramSettings) :
    _greenwichData(greenwichData), _statusData(statusData), _secondData(secondData),
    _smoothedMeasurements(smoothedMeasurements), _pidSettings(pidSettings), _paramSettings(paramSettings) {
};

MessageHandling::~MessageHandling() {
};

uint8_t MessageHandling::transmit() {
    return (uint8_t)wireMaster.sendAndReceiveMessage();         // return master or slave message in error
}

// ========== enqueue data to send buffer ==========

void MessageHandling::enqueueI2CmessageToSlave(uint8_t& msgTypeOut) {

    static long sequence{ 0 };////

    if (msgTypeOut == MsgType::M_MSG_NONE) { return; }

    switch (msgTypeOut) {

        // ========== message type requesting a PING back from slave ==========

        case MsgType::M_MSG_PING:
        {
            wireMaster.enqueueTx(M_MSG_PING, 0, nullptr, S_MSG_PING, 0);         // no payload
            Serial.println(F("ping sent"));
        }
        break;


        // ========== message types SENDING DATA to slave after an event ==========

        case  MsgType::M_MSG_SECOND:
        {
            I2C_m_secondCue p{};
            p.tempSmooth = _smoothedMeasurements.tempSmooth;                                  // raw input for wire slave
            p.magnetOnCyclesSmooth = _smoothedMeasurements.magnetOnCyclesSmooth;
            p.ISRdurationSmooth = _smoothedMeasurements.ISRdurationSmooth;
            p.idleLoopMicrosSmooth = _smoothedMeasurements.idleLoopMicrosSmooth;
            p.errSignalMagnitudeSmooth = _smoothedMeasurements.errSignalMagnitudeSmooth;
            wireMaster.enqueueTx(M_MSG_SECOND, sizeof(p), &p, S_MSG_ACK, sizeof(I2C_s_ack));
        }
        break;

        case  MsgType::M_MSG_GREENWICH:
        {
            I2C_m_greenwich p{};
            p.actualRotationTime = _greenwichData.globeRotationTime;
            p.rotationOutOfSyncTime = _greenwichData.rotationOutOfSyncTime;
            p.greenwichLag = _greenwichData.greenwichLag;
            wireMaster.enqueueTx(M_MSG_GREENWICH, sizeof(p), &p, S_MSG_ACK, sizeof(I2C_s_ack));
        }
        break;

        case MsgType::M_MSG_STATUS:
        {
            I2C_m_status p{};
            p.status = (int8_t)((_statusData.errorCondition == errNoError) ? _statusData.rotationStatus : (_statusData.errorCondition | 0x10));
            wireMaster.enqueueTx(M_MSG_STATUS, sizeof(p), &p, S_MSG_ACK, sizeof(I2C_s_ack));
        }
        break;

    #if 1 ////

            // ========== message types SENDING DATA to slave when requested ==========

        // send master send stats to wire slave
        case MsgType::M_MSG_SEND_STATS:
        {
            I2C_m_masterSendStats p{};
            wireMaster.getSendStats(p);
            wireMaster.enqueueTx(M_MSG_SEND_STATS, sizeof(p), &p, S_MSG_ACK, sizeof(I2C_s_ack));
        }
        break;

        // send master receive stats to wire slave
        case MsgType::M_MSG_RECEIVE_STATS:
        {
            I2C_m_masterReceiveStats p{};
            wireMaster.getReceiveStats(p);
            wireMaster.enqueueTx(M_MSG_RECEIVE_STATS, sizeof(p), &p, S_MSG_ACK, sizeof(I2C_s_ack));
        }
        break;


        // send user settings to wire slave
        case MsgType::M_MSG_USER_SETTINGS:
        {
        ////
        }
        break;

        // send PID settings to wire slave
        case MsgType::M_MSG_PID_SETTINGS:
        {
            I2C_m_PIDsettings p{};
            p.gainAdjustSteps = _pidSettings.gainAdjustSteps;
            p.intTimeCstAdjustSteps = _pidSettings.intTimeCstAdjustSteps;
            p.difTimeCstAdjustSteps = _pidSettings.difTimeCstAdjustSteps;
            wireMaster.enqueueTx(M_MSG_PID_SETTINGS, sizeof(p), &p, S_MSG_ACK, sizeof(I2C_s_ack));
            Serial.println("PID controller adjust steps (sent):");
            Serial.print("    gain          "); Serial.println(p.gainAdjustSteps);
            Serial.print("    int. time cst "); Serial.println(p.intTimeCstAdjustSteps);
            Serial.print("    dif. time cst "); Serial.println(p.difTimeCstAdjustSteps);
        }
        break;


        // send globe vertical position setpoint to wire slave
        case MsgType::M_MSG_VERT_POS_SETPOINT:
        {
        ////
        }
        break;

        // send coil phase adjustment setting to wire slave
        case MsgType::M_MSG_COIL_PHASE_ADJUST:
        {
        ////
        }
        break;


        // ========== message types REQUESTING DATA from slave ==========

        case MsgType::M_MSG_REQ_USER_SETTINGS:
        {
            wireMaster.enqueueTx(M_MSG_REQ_USER_SETTINGS, 0, nullptr, sizeof(I2C_s_userSettings));
        }
        break;

        case MsgType::M_MSG_REQ_PID_SETTINGS:
        {
            wireMaster.enqueueTx(M_MSG_REQ_PID_SETTINGS, 0, nullptr, sizeof(I2C_s_PIDsettings));
        }
        break;

        case MsgType::M_MSG_REQ_VERT_POS_SETPOINT:
        {   //// ??
            wireMaster.enqueueTx(M_MSG_REQ_VERT_POS_SETPOINT, 0, nullptr, S_MSG_VERT_POS_SETPOINT, sizeof(I2C_s_vertPosSetpoint));
        }
        break;

        case MsgType::M_MSG_REQ_COIL_PHASE_ADJUST:
        {   //// ??
            wireMaster.enqueueTx(M_MSG_REQ_COIL_PHASE_ADJUST, 0, nullptr, S_MSG_COIL_PHASE_ADJUST, sizeof(I2C_s_coilPhaseAdjust));
        }
        break;
    #endif

    }
    msgTypeOut = MsgType::M_MSG_NONE;
}

// ========== process Wire slave reply ==========

void MessageHandling::dequeueI2CmessageFromSlave(uint8_t& nextMsgTypeOut) {
    //// na een delay ? (slave tijd geven om data te prepareren)
    uint8_t msgTypeIn{ MsgType::M_MSG_NONE };              // message type received from slave
    uint8_t i2cPayloadSizeIn{ 0 };              // payload size as reported by slave
    uint8_t plIn[WireMaster::PAYLOAD_IN_MAX];

    bool msgAvailable = wireMaster.dequeueRx(msgTypeIn, i2cPayloadSizeIn, &plIn);
    if (!msgAvailable) {
        nextMsgTypeOut = M_MSG_NONE;
        return;
    }

    switch (msgTypeIn) {

        case S_MSG_PING:
        {
            nextMsgTypeOut = MsgType::M_MSG_NONE;
            Serial.println(F("ping received"));
        }
        break;

        case S_MSG_ACK:
        {
            I2C_s_ack* p = reinterpret_cast<I2C_s_ack*>(plIn);
            if (i2cPayloadSizeIn != sizeof(I2C_s_ack)) { break; }               // inconsistency between master and slave: forget message //// reeds gecheckt in lib ?
            // next requested message type - allowed message types:     
            // M_MSG_SEND_STATS, M_MSG_RECEIVE_STATS, M_MSG_USER_SETTINGS, M_MSG_PID_SETTINGS, M_MSG_VERT_POS_SETPOINT, M_MSG_COIL_PHASE_ADJUST
            nextMsgTypeOut = p->requestMasterMsgType;
        }
        break;

        // receive user settings from wire slave
        case S_MSG_USER_SETTINGS: //// use of volatile vars: ok ????
        {
            nextMsgTypeOut = MsgType::M_MSG_NONE;

            I2C_s_userSettings* p = reinterpret_cast<I2C_s_userSettings*>(plIn);
            if (i2cPayloadSizeIn != sizeof(I2C_s_userSettings)) { break; }               // inconsistency between master and slave: forget message //// reeds gecheckt in lib ?

            Serial.print("rot. time idx "); Serial.print(p->userSet_rotationPeriod);
            Serial.print(", leds "); Serial.print(p->userSet_ledEffect);
            Serial.print(", "); Serial.println(p->userSet_ledCycleSpeed);
            break;      //// temp

            // rotation period
            int cnt = paramValueCounts[paramNo_rotTimes];
            if ((p->userSet_rotationPeriod >= 0) && (p->userSet_rotationPeriod < cnt)) {
                _paramSettings.paramNo = paramNo_rotTimes;
                _paramSettings.paramValueOrIndex = p->userSet_rotationPeriod;
                saveAndUseParam();
            }

            // digital LED settings
            bool valid = ((p->userSet_ledEffect >= cLedstripOff) && (p->userSet_ledEffect <= cRedGreenBlue)
                && (p->userSet_ledCycleSpeed >= cLedstripVeryFast) && (p->userSet_ledCycleSpeed <= cLedStripVerySlow));
            if (valid) { setColorCycle(p->userSet_ledEffect, p->userSet_ledCycleSpeed); }
        }
        break;

        // receive PID settings from wire slave
        case S_MSG_PID_SETTINGS: //// use of volatile vars: ok ????
        {
            nextMsgTypeOut = MsgType::M_MSG_NONE;

            I2C_s_PIDsettings* p = reinterpret_cast<I2C_s_PIDsettings*>(plIn);
            if (i2cPayloadSizeIn != sizeof(I2C_s_userSettings)) { break; }               // inconsistency between master and slave: forget message //// reeds gecheckt in lib ?

            Serial.print("PID gain "); Serial.print(p->gainAdjustSteps);
            Serial.print(", int.cst "); Serial.print(p->intTimeCstAdjustSteps);
            Serial.print(", dif.cst "); Serial.println(p->difTimeCstAdjustSteps);
            break;      //// temp

            // setting steps: positive values (preset value = mid point) 
            _pidSettings.gainAdjustSteps = (uint8_t)p->gainAdjustSteps;                  // preset gain corresponds to gainAdjustSteps mid value   
            _pidSettings.intTimeCstAdjustSteps = (uint8_t)p->intTimeCstAdjustSteps;
            _pidSettings.difTimeCstAdjustSteps = (uint8_t)p->difTimeCstAdjustSteps;

            if (_pidSettings.gainAdjustSteps >= settingSteps) { _pidSettings.gainAdjustSteps = settingSteps - 1; }
            if (_pidSettings.intTimeCstAdjustSteps >= settingSteps) { _pidSettings.intTimeCstAdjustSteps = settingSteps - 1; }
            if (_pidSettings.difTimeCstAdjustSteps >= settingSteps) { _pidSettings.difTimeCstAdjustSteps = settingSteps - 1; }
            
            _paramSettings.pParamsSelectedValuesOrIndexes[paramNo_gainAdjust] = _pidSettings.gainAdjustSteps;
            _paramSettings.pParamsSelectedValuesOrIndexes[paramNo_intTimeConstAdjust] = _pidSettings.intTimeCstAdjustSteps;
            _paramSettings.pParamsSelectedValuesOrIndexes[paramNo_difTimeConstAdjust] = _pidSettings.difTimeCstAdjustSteps;

            saveAndUseParam();  //// alle 3 de waarden updated ? of slechts één ? ik vrees het
        }
        break;

        // receive globe vertical position setpoint from wire slave
        case S_MSG_VERT_POS_SETPOINT:
        {
            nextMsgTypeOut = MsgType::M_MSG_NONE;

            I2C_s_vertPosSetpoint* p = reinterpret_cast<I2C_s_vertPosSetpoint*>(plIn);
            if (i2cPayloadSizeIn != sizeof(I2C_s_vertPosSetpoint)) { break; }               // inconsistency between master and slave: forget message //// reeds gecheckt in lib ?

            // if not valid, ignore
            int cnt = paramValueCounts[paramNo_hallmVoltRefs];
            if ((p->userSet_vertPosIndex >= 0) && (p->userSet_vertPosIndex < cnt)) {
                _paramSettings.paramNo = paramNo_hallmVoltRefs;
                _paramSettings.paramValueOrIndex = p->userSet_vertPosIndex;
                saveAndUseParam();
            }
        }
        break;

        // receive coil phase adjustment setting from wire slave
        case  S_MSG_COIL_PHASE_ADJUST:
        {
            nextMsgTypeOut = MsgType::M_MSG_NONE;

            I2C_s_coilPhaseAdjust* p = reinterpret_cast<I2C_s_coilPhaseAdjust*>(plIn);
            if (i2cPayloadSizeIn != sizeof(I2C_s_coilPhaseAdjust)) { break; }               // inconsistency between master and slave: forget message //// reeds gecheckt in lib ?

            if (p->userSet_coilPhaseAdjust > 179) { p->userSet_coilPhaseAdjust = 179; }     // phase adjustment in 2-degree increments (0 to 358 degrees)                                 
            _paramSettings.paramNo = paramNo_phaseAdjust;
            _paramSettings.paramValueOrIndex = p->userSet_coilPhaseAdjust;
            saveAndUseParam;


        }
        break;

        default:
        {
            nextMsgTypeOut = MsgType::M_MSG_NONE;
        }
        break;

    }
}


void MessageHandling::getWireStats() {

    static uint32_t lastTime{ 0 };
    if (lastTime + 10UL < _secondData.eventSecond) {
        lastTime = _secondData.eventSecond;
        WireMaster::I2C_MasterSendStats sendStats; WireMaster::I2C_MasterReceiveStats receiveStats;
        wireMaster.getSendStats(sendStats);
        wireMaster.getReceiveStats(receiveStats);

    /*
        Serial.println(F("======== STATS ========"));
        Serial.print(F("sent       ")); Serial.println(masterSendStats.I_stats_sent);
        Serial.print(F("tx_retry   ")); Serial.println(masterSendStats.W_stats_tx_retrying);
        Serial.print(F("tx_xmitErr ")); Serial.println(masterSendStats.E_stats_tx_wireXmitError);
        Serial.print(F("tx_full    ")); Serial.println(masterSendStats.E_stats_tx_full);
        Serial.print(F("received   ")); Serial.println(masterReceiveStats.I_stats_received);
        Serial.print(F("rx_ch.sum  ")); Serial.println(masterReceiveStats.E_stats_rx_checksum);
        Serial.print(F("rx_timeout ")); Serial.println(masterReceiveStats.E_stats_rx_timeOut);
        Serial.print(F("rx_full    ")); Serial.println(masterReceiveStats.E_stats_rx_full);
        Serial.println();
    */
    }
}



