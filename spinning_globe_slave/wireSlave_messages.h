#ifndef _WIRE_SLAVE_MESSAGES_h
#define _WIRE_SLAVE_MESSAGES_h

#include "sharedContext.h"
#include "wireSlave_transport.h"


// MASTER send & receive stats, received as message. Sole purpose: receive wire stats from master 

class WireSlaveMessages {

    WireSlave wireSlave;

    struct I2C_m_masterSendStats {////
        uint32_t I_stats_sent = 0;                             // info: counts     
        uint32_t W_stats_tx_retrying = 0;                      // warning: count 
        uint32_t E_stats_tx_wireXmitError = 0;
        uint32_t E_stats_tx_full = 0;                          // errors: counts
    };

    struct I2C_m_masterReceiveStats {////
        uint32_t I_stats_received = 0;
        uint32_t E_stats_rx_checksum = 0;
        uint32_t E_stats_rx_timeOut = 0;
        uint32_t E_stats_rx_full = 0;
    };

    SharedContext& _sharedContext;

    void convertGlobeStatusToMQTT(I2C_m_status* p);
    void convertGlobeGreenwichCueToMQTT(I2C_m_greenwich* p);
    void convertSecondCueToMQTT(I2C_m_secondCue* p);
    void convertGlobeSettingsToMQTT(I2C_m_globeSettings* p);
    void convertPIDsettingsToMQTT(I2C_m_PIDsettings* p);

    void replyAndFlagSlaveDataAvailable();             // naming

    //// ???
    //bool processMsgIn_second(uint8_t payloadSizeIn, void* ppl);
    //void formatFloat(char* out, size_t outSize, float value, uint8_t decimals);

public:
    WireSlaveMessages(SharedContext& sharedContext);
    bool  loop();


};


#endif

