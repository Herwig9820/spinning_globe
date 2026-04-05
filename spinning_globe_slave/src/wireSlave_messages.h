#ifndef FG_SLAVE_MESSAGES_h
#define FG_SLAVE_MESSAGES_h

#include "bridge_context.h"
#include "wireSlave_transport.h"


// MASTER send & receive stats, received as message. Sole purpose: receive wire stats from master 

class WireSlaveMessages {

    WireSlave wireSlave;

    SharedContext& _sharedContext;

    void convertGlobeStatusToMQTT(I2C_m_status* p);
    void convertGlobeGreenwichCueToMQTT(I2C_m_greenwich* p);
    void convertTelemetryToMQTT(I2C_m_telemetry* p);
    void convertTelemetryExtraToMQTT(I2C_m_telemetry_extra* p);
    void convertGlobeSettingsToMQTT(I2C_m_globeSettings* p);
    void convertPIDsettingsToMQTT(I2C_m_PIDsettings* p);
    void convertVertPosSetpointToMQTT(I2C_m_vertPosSetpoint* p);
    void convertCoilPhaseAdjustmentToMQTT(I2C_m_coilPhaseAdjust* p);

    void replyAndFlagSlaveDataAvailable();             

public:
    WireSlaveMessages(SharedContext& sharedContext);
    bool loop();


};


#endif

