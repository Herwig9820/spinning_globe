#ifndef FG_SLAVE_CONTEXT_h
#define FG_SLAVE_CONTEXT_h

#include "shared/wire_protocol.h"
#include <stddef.h>
#include <pins_arduino.h>


enum rotStatus :uint8_t {
    rotNoPosSync, rotFreeRunning, rotMeasuring, rotUnlocked, rotLocked, // rotNoPosSync: also if rotation OFF or not floating   
    errDroppedGlobe = 0x11, errStickyGlobe, errMagnetLoad, errTemp
};
//// eBlink, eSpareNoDataEvent1: cue only (no data) events. additional time cues can be added
////enum events :uint8_t { eNoEvent = 0, eGreenwich, eStatusChange, eFastRateData, eLedstripData, eStepResponseData, eSecond, eBlink, eSpareNoDataEvent1 };
enum colorCycles :uint8_t { cLedstripOff = 0, cCstBrightWhite, cCstBrightMagenta, cCstBrightBlue, cWhiteBlue, cRedGreenBlue };      // led strip color cycle 
enum colorTiming :uint8_t { cLedstripVeryFast = 0, cLedstripFast, cLedstripSlow, cLedStripVerySlow };                               // led strip color cycle 


// ========== MQTT topics ==========

// ---------- publish ----------
constexpr const char* TOPIC_STATUS = "globe/status";                                        // publish spinning globe status and settings
constexpr const char* TOPIC_GREENWICH = "globe/greenwich";
constexpr const char* TOPIC_TELEMETRY = "globe/telemetry";

constexpr const char* TOPIC_GLOBE_SETTINGS = "globe/settings";
constexpr const char* TOPIC_PID_SETTINGS = "globe/PIDsettings";
constexpr const char* TOPIC_VERT_POS_SETPOINT = "globe/vertPosSetpoint";
constexpr const char* TOPIC_COIL_PHASE_ADJUST = "globe/coilPhaseAdjust";

// ---------- subscribed to ----------
constexpr const char* TOPIC_GLOBE_SETTINGS_SET = "globe/settings/set";                      // spinning globe settings published by node-red or other dashboard
constexpr const char* TOPIC_PID_SETTINGS_SET = "globe/PIDsettings/set";
constexpr const char* TOPIC_VERT_POS_SETPOINT_SET = "globe/vertPosSetpoint/set";
constexpr const char* TOPIC_COIL_PHASE_ADJUST_SET = "globe/coilPhaseAdjust/set";

constexpr const char* TOPIC_GLOBE_SETTINGS_REQUEST = "globe/settings/request";              // node-red (or other dashboard) request to publish spinning globe settings
constexpr const char* TOPIC_PID_SETTINGS_REQUEST = "globe/PIDsettings/request";
constexpr const char* TOPIC_VERT_POS_SETPOINT_REQUEST = "globe/vertPosSetpoint/request";
constexpr const char* TOPIC_COIL_PHASE_ADJUST_REQUEST = "globe/coilPhaseAdjust/request";

constexpr const char* TOPIC_WIRE_STATS_REQUEST = "globe/wireStats/request";////

constexpr const char* TOPIC_RING_REQUEST = "globe/ring/request";                            // node-red (or other dashboard) request for 'visual ring' action


// ========== MQTT payloads: JSON payload field keys ==========

// ---------- settings ----------
constexpr const char* PAYLOAD_SECRET_TOKEN = "token";

constexpr const char* PL_KEY_SET_ROT_TIME = "setRotTime";
constexpr const char* PL_KEY_SET_LED_EFFECT = "setLedEffect";
constexpr const char* PL_KEY_SET_LED_EFFECT_SPEED = "setLedEffectSpeed";
constexpr const char* PL_KEY_SET_GAIN_ADJUST = "setGainAdjust";
constexpr const char* PL_KEY_SET_DIFF_TIME_ADJUST = "setDifTimeAdjust";
constexpr const char* PL_KEY_SET_INTEGR_TIME_ADJUST = "setIntTimeAdjust";
constexpr const char* PL_KEY_SET_VERT_POS_SETPOINT = "setVertPosSetpoint";
constexpr const char* PL_KEY_SET_COIL_PHASE_ADJUST = "setCoilPhaseAdjust";

// ---------- actual values, measurements ----------
constexpr const char* PL_KEY_ACT_TIME = "actRotTime";
constexpr const char* PL_KEY_ROT_SYNC_ERROR = "rotSyncError";
constexpr const char* PL_KEY_GREENWICH_LAG = "greenwichLag";
constexpr const char* PL_KEY_HEATSINK_TEMP = "heatsinkTemp";
constexpr const char* PL_KEY_MAGNET_DUTY_CYCLE = "magnetDutyCycle";
constexpr const char* PL_KEY_ISR_DURATION = "ISRduration";
constexpr const char* PL_KEY_PROC_LOAD = "load";
constexpr const char* PL_KEY_VERT_POS_ERROR = "vertPosError";
constexpr const char* PL_KEY_TTT_INTEGR_TERM = "TTTintegrTerm";


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
    SPSCQueue<AckPayload, 8> holdAckResponses;                // message types to be sent by master to send data to wire slave

    volatile bool triggerWireCommLed{false};

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

