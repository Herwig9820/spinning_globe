#ifndef FG_SLAVE_CONTEXT_h
#define FG_SLAVE_CONTEXT_h

#include "wire_protocol.h"
#include <stddef.h>
#include <pins_arduino.h>

constexpr int WIRE_RECEIVE_LED{ LED_BUILTIN };              // pin 13 (receive only)

enum rotStatus :uint8_t {
    rotNoPosSync, rotFreeRunning, rotMeasuring, rotUnlocked, rotLocked, // rotNoPosSync: also if rotation OFF or not floating   
    errDroppedGlobe = 0x11, errStickyGlobe, errMagnetLoad, errTemp
};
//// eBlink, eSpareNoDataEvent1: cue only (no data) events. additional time cues can be added
////enum events :uint8_t { eNoEvent = 0, eGreenwich, eStatusChange, eFastRateData, eLedstripData, eStepResponseData, eSecond, eBlink, eSpareNoDataEvent1 };
enum colorCycles :uint8_t { cLedstripOff = 0, cCstBrightWhite, cCstBrightMagenta, cCstBrightBlue, cWhiteBlue, cRedGreenBlue };      // led strip color cycle 
enum colorTiming :uint8_t { cLedstripVeryFast = 0, cLedstripFast, cLedstripSlow, cLedStripVerySlow };                               // led strip color cycle 


// wire to MQTT
constexpr const char* TOPIC_STATUS = "globe/status";
constexpr const char* TOPIC_GREENWICH = "globe/greenwich";
constexpr const char* TOPIC_TELEMETRY = "globe/telemetry";

// wire to MQTT
constexpr const char* TOPIC_GLOBE_SETTINGS = "globe/settings";
constexpr const char* TOPIC_PID_SETTINGS = "globe/PIDsettings";
constexpr const char* TOPIC_VERT_POS_SETPOINT = "globe/vertPosSetpoint";
constexpr const char* TOPIC_COIL_PHASE_ADJUST = "globe/coilPhaseAdjust";

// MQTT to Wire: settings changed by MQTT client (e.g., node-red)
constexpr const char* TOPIC_GLOBE_SETTINGS_SET = "globe/settings/set";
constexpr const char* TOPIC_PID_SETTINGS_SET = "globe/PIDsettings/set";
constexpr const char* TOPIC_VERT_POS_SETPOINT_SET = "globe/vertPosSetpoint/set";
constexpr const char* TOPIC_COIL_PHASE_ADJUST_SET = "globe/coilPhaseAdjust/set";

// MQTT to Wire: MQTT client requests data from wire master (spinning globe) 
constexpr const char* TOPIC_GLOBE_SETTINGS_REQUEST = "globe/settings/request";
constexpr const char* TOPIC_PID_SETTINGS_REQUEST = "globe/PIDsettings/request";
constexpr const char* TOPIC_VERT_POS_SETPOINT_REQUEST = "globe/vertPosSetpoint/request";
constexpr const char* TOPIC_COIL_PHASE_ADJUST_REQUEST = "globe/coilPhaseAdjust/request";


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

    T* front() {
        return &buffer[tail];
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


struct MQTTmsgFromWire {
    char topic[48];
    char payload[160];
    bool retain{ true };
};

struct MQTTmsgToWire {
    char topic[48];
    char payload[160];
};

struct SharedContext {

    // ---------- Queues ----------

    // queues with inbound and outbound MQTT messages
    SPSCQueue<MQTTmsgFromWire, 16> queueToMQTT;
    SPSCQueue<MQTTmsgToWire, 16> queueToWire;

    // MQTT to wire flows: holding queue 
    SPSCQueue<MsgType, 8> requestDataFromMaster;                // message types to be sent by master to send data to wire slave

    // Optional: shared counters
    uint32_t mqttMessagesSent = 0;
    uint32_t wireMessagesSent = 0;

    // Optional: timestamps
    uint32_t lastMQTTpublish = 0;
    uint32_t lastWireActivity = 0;


    // ---------- wire slave has data for wire master ----------

    I2C_s_globeSettings_set pendingGlobeSettings;
    I2C_s_globeSettings_set committedGlobeSettings;

    I2C_s_PIDsettings_set pendingPIDsettings;
    I2C_s_PIDsettings_set committedPIDsettings;

    I2C_s_coilPhaseAdjust_set pendingCoilPhaseAdjust;
    I2C_s_coilPhaseAdjust_set committedCoilPhaseAdjust;

    I2C_s_vertPosSetpoint_set pendingVertPosSetpoint;
    I2C_s_vertPosSetpoint_set committedVertPosSetpoint;
};

#endif

