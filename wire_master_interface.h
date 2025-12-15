#ifndef _WIRE_MASTER_INTERFACE.h
#define _WIRE_MASTER_INTERFACE.h

#include "arduino.h"
#include <util/atomic.h>                                    // atomic operations

class  Wire_master_interface {


/* ================= CONFIGURATION ================= */

    static constexpr  uint8_t I2C_SLAVE_ADDR = 9;
    static constexpr uint32_t I2C_CLOCK = 100000UL;  // 100 kHz////

    // Packet sizing
    static constexpr uint8_t HEADER_SIZE = 2;      // message type and length
    static constexpr uint8_t PAYLOAD_OUT_MAX = 20;////
    static constexpr uint8_t PACKET_OUT_MAX = HEADER_SIZE + PAYLOAD_OUT_MAX + 1; // type + len + payload + checksum: length max. 32

    // Ring queue
    static constexpr uint8_t TX_QUEUE_SIZE = 4;      // small bounded queue

    // Timing/backoff
    static constexpr unsigned long CYCLE_PERIOD_MS = 10;   // spacing after successful action
    static constexpr unsigned long BACKOFF_DELAY_MS = 2;

    // Retries
    static constexpr uint8_t MAX_RETRIES_PER_PACKET = 3;

////private:
public:
/* ================= TX: RING BUFFER, RX: SINGLE BUFFER ================= */

uint8_t oldHead =0; //// test
    // ISR safe

    volatile uint8_t expectedRxMsgLength[TX_QUEUE_SIZE];
    
    volatile uint8_t rxQueue[HEADER_SIZE + PAYLOAD_OUT_MAX + 1];                    // only one deep
    volatile uint8_t txQueue[TX_QUEUE_SIZE][HEADER_SIZE + PAYLOAD_OUT_MAX + 1];  // type + len + payload + checksum

    volatile uint8_t txHead = 0; // next free slot (main or ISR will write here)
    volatile uint8_t txTail = 0; // oldest queued (main reads here)
    
    volatile bool rxAvailable {false};                          // instead of head and tail (rx queue only one deep)

    volatile bool txPending = false;


    /* ================= STATE + WORKING COPIES ================= */

    enum MasterState { MS_IDLE, MS_SEND, MS_RECEIVE };
    MasterState state = MS_IDLE;

    uint8_t txWorking[HEADER_SIZE + PAYLOAD_OUT_MAX + 1];
    uint8_t rxWorking[HEADER_SIZE + PAYLOAD_OUT_MAX + 1];

    unsigned long previousCycleTime = 0; ////
    unsigned long nextCycleTime = 0;   // inter-packet delay scheduling
    
    uint8_t rxMsgLength = 0;;

    uint8_t sendRetryCount = 0;

    uint32_t stats_sent = 0;
    uint32_t stats_dropped = 0;
    uint32_t stats_errors = 0;



public:
    Wire_master_interface();
    ~Wire_master_interface();

    // safe to call from ISR
    bool enqueueTx(uint8_t type, const void* payload, uint8_t payloadSize, uint8_t replyPayloadSize);
    bool dequeueRx(uint8_t &type, void* payload, uint8_t &payloadSize);

    // should be called frequently from application main loop
    void sendAndReceiveMessage();

    // called from sendAndReceiveMessage()
    bool copyTXqueueTail(uint8_t* const out, uint8_t& indexAtTail);
    bool i2cWriteMessage(const uint8_t* p);
    uint8_t i2cReadMessage(uint8_t* p);
    bool copyRXqueue(uint8_t* const in);
    
    void popTXtailAtomic();

};

#endif

