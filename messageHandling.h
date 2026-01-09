#ifndef _MESSAGEHANDLING_h
#define _MESSAGEHANDLING_h

#include "messages_common.h"
#include "wireMaster.h"
#include "floatingGlobeState.h"

class MessageHandling {

    SmoothedMeasurements& _smoothedMeasurements;
    PIDsettings& _pidSettings;
    ParamSettings& _paramSettings;

    GreenwichData& _greenwichData;
    StatusData& _statusData;
    SecondData& _secondData;

    WireMaster wireMaster;

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

