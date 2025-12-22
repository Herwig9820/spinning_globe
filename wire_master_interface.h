#ifndef _WIRE_MASTER_INTERFACE.h
#define _WIRE_MASTER_INTERFACE.h

#include "arduino.h"
#include <util/atomic.h>                                            // atomic operations

class  Wire_master_interface {

/* ================= CONSTANTS ================= */

public:
    static constexpr uint8_t PAYLOAD_IN_MAX = 10;                   // max. payload sizes in bytes 
    static constexpr uint8_t PAYLOAD_OUT_MAX = 20;

private:
    static constexpr  uint8_t I2C_SLAVE_ADDR = 9;
    static constexpr uint32_t I2C_CLOCK = 100000UL;                 // 100 or 400 kHz

    // Packet (message) sizing
    static constexpr uint8_t HEADER_SIZE = 2;                       // message type and length

    // Ring queue
    static constexpr uint8_t TX_QUEUE_SIZE = 4;                     // queue depths     
    static constexpr uint8_t RX_QUEUE_SIZE = 4;

    // Timing/backoff
    static constexpr unsigned long CYCLE_PERIOD_MS = 10;            // spacing between send or receive operations

    // Retries
    static constexpr uint8_t MAX_RETRIES_PER_PACKET = 3;            // max. retries allowed per message (send)


/* ================= TX: RING BUFFER, RX: RING BUFFER ================= */

private:
    volatile uint8_t txExpReplyMsgSize[TX_QUEUE_SIZE];              // paired with txQueue slot index

    volatile uint8_t rxQueue[RX_QUEUE_SIZE][HEADER_SIZE + PAYLOAD_IN_MAX + 1];  // message type + payload size + payload + checksum
    volatile uint8_t txQueue[TX_QUEUE_SIZE][HEADER_SIZE + PAYLOAD_OUT_MAX + 1];

    // producer owned (SPSC)
    volatile uint8_t rxHead = 0;                                    // next free slot (main or ISR will write here)
    volatile uint8_t txHead = 0;                                    // next free slot (main or ISR will write here)

    // consumer owned (SPSC)
    volatile uint8_t txTail = 0;                                    // oldest queued (main reads here)
    volatile uint8_t rxTail = 0;                                    // oldest queued (main reads here)


    /* ================= STATE + WORKING COPIES ================= */

    enum MasterState { MS_IDLE, MS_SEND, MS_RECEIVE };
    MasterState state = MS_IDLE;

    uint8_t rxInBuffer[HEADER_SIZE + PAYLOAD_IN_MAX + 1];
    uint8_t txOutBuffer[HEADER_SIZE + PAYLOAD_OUT_MAX + 1];

    unsigned long lastCycleTime = 0;                                // inter-packet delay scheduling
    uint8_t sendMaxTries = 0;
    uint8_t msgType = 0;

    volatile uint32_t I_stats_sent = 0;                             // info: counts     
    volatile uint32_t W_stats_tx_retrying = 0;                      // warning: count 
    volatile uint32_t E_stats_tx_wireXmitError = 0;
    volatile uint32_t E_stats_tx_full = 0;                          // errors: counts

    volatile uint32_t I_stats_received = 0;
    volatile uint32_t E_stats_rx_checksum = 0;
    volatile uint32_t E_stats_rx_timeOut = 0;
    volatile uint32_t E_stats_rx_full = 0;

public:
    enum class WireStatus : uint8_t {
        I_idle,
        I_waitForCue,                                               // counting time until next cue
        I_xmitOK,                                                   // sent/receive went OK
        E_tx_WireXmitError,
        E_rx_checksum,
        E_rx_timeOut,
        E_rx_full
    };

    Wire_master_interface();
    ~Wire_master_interface();

    // safe to call from ISR
    // enqueue with expReplyPayloadSize = 0xff: no return message requested, = 0x00: requested, without payload
    bool enqueueTx(uint8_t type, uint8_t payloadSize, const void* payload, uint8_t expReplyPayloadSize = 0xff);
    bool dequeueRx(uint8_t& type, uint8_t& payloadSize, void* payload);

    // should be called frequently from application main loop
    WireStatus sendAndReceiveMessage(uint8_t* msgType = nullptr);

    void getStats(uint32_t& sent, uint32_t& tx_retries, uint32_t& tx_wireXmitError, uint32_t& tx_full,
        uint32_t& received, uint32_t& rx_checksum, uint32_t& rx_timeOut, uint32_t& rx_full);

private:
    // helpers: called from sendAndReceiveMessage()
    bool copyTXqueueTailToOut(uint8_t* const out, uint8_t& expReplyMsgSize);
    bool i2cWriteMessage(const uint8_t* p);
    bool i2cReadMessage(uint8_t* p, uint8_t expReplyMsgSize);
    Wire_master_interface::WireStatus copyInToRXqueueHead(uint8_t* const in, uint8_t expReplyMsgSize);
};

#endif

