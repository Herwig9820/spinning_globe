#ifndef _WIRE_MASTER_INTERFACE-.h
#define _WIRE_MASTER_INTERFACE-.h

#include "arduino.h"

class  Wire_master_interface {

/* ================= CONFIGURATION ================= */

    static constexpr  uint8_t I2C_SLAVE_ADDR = 9;
    static constexpr uint32_t I2C_CLOCK = 400000UL;  // 100 kHz////

    // Packet sizing
    static constexpr uint8_t PAYLOAD_MAX = 29;
    static constexpr uint8_t PACKET_MAX = 2 + PAYLOAD_MAX + 1; // id + len + payload + checksum: length max. 32

    // Ring queue
    static constexpr uint8_t TX_QUEUE_SIZE = 4;      // small bounded queue

    // Timing/backoff
    static constexpr unsigned long CYCLE_MS = 10;   // spacing after successful action
    static constexpr unsigned long RETRY_BACKOFF_MS = 10;   // backoff after failed send
    static constexpr unsigned long REPLY_TIMEOUT_MS = 3000UL;

    // Retries
    static constexpr uint8_t MAX_RETRIES_PER_PACKET = 4;


public:

/* ================= MESSAGE TYPES ================= */

    enum MsgID : uint8_t {
        MSG_PING = 0x01,
        MSG_GET_SETTINGS = 0x02,
        MSG_GREENWICH_EVENT = 0x03,
        MSG_STATUS_EVENT = 0x04,
        MSG_SECONDS_EVENT = 0x05,
        MSG_ERROR = 0xFF
    };

    /* ================= PACKET STRUCTS ================= */

    struct PacketTX {
        uint8_t id;
        uint8_t len;                 // payload length (0..PAYLOAD_MAX)
        uint8_t payload[PAYLOAD_MAX];
        uint8_t checksum;
    };

    struct PacketRX {
        uint8_t id;
        uint8_t len;
        uint8_t payload[PAYLOAD_MAX];
        uint8_t checksum;
    };

private:

/* ================= RING BUFFER (ISR-safe enqueue) ================= */

    PacketTX txQueue[TX_QUEUE_SIZE];
    volatile uint8_t txHead = 0; // next free slot (ISR will write here)
    volatile uint8_t txTail = 0; // oldest queued (main reads here)
    uint8_t retryCounts[TX_QUEUE_SIZE]; // per-slot retry count (main only)


    /* ================= STATE + WORKING COPIES ================= */

    enum MasterState { MS_IDLE, MS_SEND, MS_RECEIVE };
    MasterState state = MS_IDLE;

    PacketTX txWorking; // working copy of the packet we attempt to send (peeked from queue)
    PacketRX rxWorking;

    unsigned long nextActionMillis = 0;   // inter-packet delay scheduling
    unsigned long nextRetryMillis = 0;   // global backoff timer
    uint32_t stats_sent = 0;
    uint32_t stats_dropped = 0;
    uint32_t stats_errors = 0;



public:
    Wire_master_interface();
    ~Wire_master_interface();

    bool enqueueTxISR(const PacketTX& p);
    bool enqueueTxISR(uint8_t type, const void* payload, uint8_t length);
    uint8_t calcChecksum(PacketTX& p, uint8_t len);
    uint8_t calcChecksum(PacketRX& p, uint8_t len);
    bool i2cWritePacket(const PacketTX& p);
    uint8_t i2cMasterReadWithTimeout(uint8_t len, PacketRX p, unsigned long reply_timeout_ms);
    bool peekQueueTail(PacketTX& out, uint8_t& indexAtTail);
    void popTailAtomic();
    void processQueueAndSend();

    uint32_t incr_stats_sent();
    uint32_t incr_stats_dropped();
    uint32_t incr_stats_errors();
};

#endif

