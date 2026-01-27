#ifndef _SHARED_DATA_h
#define _SHARED_DATA_h

// ----- shared with spinning globe master ----- //// moeten identiek zijn, maar zijn nu niet shared

constexpr long spinningGlobeNano_timer1ClockFreq{ 2000000L };
constexpr long spinningGlobeNano_timer1PWMfreq{ 1000L };
constexpr long spinningGlobeNano_timer1Top{ spinningGlobeNano_timer1ClockFreq / spinningGlobeNano_timer1PWMfreq / 2 };
constexpr float spinningGlobeNano_ADCmVperStep = 5000. / 1024.;
constexpr long spinningGlobeNano_fastDataRateSamplingPeriods{ 1 << 7 };

enum rotStatus :uint8_t {
    rotNoPosSync, rotFreeRunning, rotMeasuring, rotUnlocked, rotLocked, // rotNoPosSync: also if rotation OFF or not floating   
    errDroppedGlobe = 0x11, errStickyGlobe, errMagnetLoad, errTemp
};
//// eBlink, eSpareNoDataEvent1: cue only (no data) events. additional time cues can be added
////enum events :uint8_t { eNoEvent = 0, eGreenwich, eStatusChange, eFastRateData, eLedstripData, eStepResponseData, eSecond, eBlink, eSpareNoDataEvent1 };
enum colorCycles :uint8_t { cLedstripOff = 0, cCstBrightWhite, cCstBrightMagenta, cCstBrightBlue, cWhiteBlue, cRedGreenBlue };      // led strip color cycle 
enum colorTiming :uint8_t { cLedstripVeryFast = 0, cLedstripFast, cLedstripSlow, cLedStripVerySlow };                               // led strip color cycle 


constexpr const char str_rotationOff[] = "rotation off";
constexpr const char str_freeRunning[] = "free running";
constexpr const char str_noPosSync[] = "wait for pos sync";
constexpr const char str_measuring[] = "measuring";
constexpr const char str_notLocked[] = "not locked";
constexpr const char str_locked[] = "locked";
constexpr const char str_notFloating[] = "not floating";
constexpr const char str_ErrDroppedGlobe[] = "E! dropped globe";
constexpr const char str_ErrStickyGlobe[] = "E! sticky globe";
constexpr const char str_ErrOverload[] = "E! overload";
constexpr const char str_ErrTemp[] = "E! temp too high";




#include <stddef.h>

constexpr const char* TOPIC_TELEMETRY = "telemetry";


template<typename T, size_t N>
class SPSCQueue {
public:
    bool push(const T& item) {
        size_t next = (head + 1) % N;
        if (next == tail) {
            return false; // full
        }
        buffer[head] = item;
        head = next;
        return true;
    }

    bool pop(T& out) {
        if (tail == head) {
            return false; // empty
        }
        out = buffer[tail];
        tail = (tail + 1) % N;
        return true;
    }

    bool empty() const {
        return head == tail;
    }

    bool full() const {
        return ((head + 1) % N) == tail;
    }

private:
    T buffer[N];
    size_t head = 0;
    size_t tail = 0;
}; 


struct MsgToMQTT {
    char topic[48];
    char payload[128];
    //uint8_t qos;
    //bool retain;
};

struct MsgToWire {
    char msgType;
    char msgSize;
    char payload[128];
};


struct SharedContext {
    // Queues
    SPSCQueue<MsgToMQTT, 16> queueToMQTT;
    SPSCQueue<MsgToWire, 16> queueToWire;

    // Optional: shared counters
    uint32_t mqttMessagesSent = 0;
    uint32_t wireMessagesSent = 0;

    // Optional: shared configuration
    bool verboseLogging = false;

    // Optional: timestamps
    uint32_t lastMQTTpublish = 0;
    uint32_t lastWireActivity = 0;
};


#endif

