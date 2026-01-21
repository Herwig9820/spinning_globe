// messageHandling.h

#ifndef _MESSAGEHANDLING_h
#define _MESSAGEHANDLING_h

#include "messages_common.h"

#include "wireSlave.h"


// MASTER send & receive stats, received as message. Sole purpose: receive wire stats from master 

struct I2C_m_masterSendStats{
    uint32_t I_stats_sent = 0;                             // info: counts     
    uint32_t W_stats_tx_retrying = 0;                      // warning: count 
    uint32_t E_stats_tx_wireXmitError = 0;
    uint32_t E_stats_tx_full = 0;                          // errors: counts
};

struct I2C_m_masterReceiveStats{
    uint32_t I_stats_received = 0;
    uint32_t E_stats_rx_checksum = 0;
    uint32_t E_stats_rx_timeOut = 0;
    uint32_t E_stats_rx_full = 0;
};




#endif

