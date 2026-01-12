#ifndef _MESSAGEHANDLING_h
#define _MESSAGEHANDLING_h

#include "floatingGlobeState.h"
#include "messages_common.h"

#include "roles/wireMaster.h"               // or master slave 

class MessageHandling {

    SmoothedMeasurements& _smoothedMeasurements;
    PIDsettings& _pidSettings;
    ParamSettings& _paramSettings;

    GreenwichData& _greenwichData;
    StatusData& _statusData;
    SecondData& _secondData;

    WireMaster wireMaster;

    using I2C_m_masterSendStats = WireMaster::I2C_MasterSendStats;
    using I2C_m_masterReceiveStats = WireMaster::I2C_MasterReceiveStats;

        
    /* ========== METHODS ========== */

public:
    MessageHandling(GreenwichData& greenwichData, StatusData& statusData, SecondData& secondData,
        SmoothedMeasurements& smoothedMeasurements, PIDsettings& pidSettings, ParamSettings& paramSettings);
    ~MessageHandling();

    uint8_t transmit();
    void enqueueI2CmessageToSlave(uint8_t& msgTypeOut);
    void dequeueI2CmessageFromSlave(uint8_t& nextMsgTypeOut);
    void getWireStats();
};

#endif

