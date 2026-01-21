#include "messageHandling.h"
#include "wireSlave.h"

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

const char* root_ca = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";


// WireSlave class is not aware of payload details
// note: message types and message definitions (structures, below) MUST BE EQUAL in Wire master and slave setup 


long targetGlobeRotationTime{ 12000 };//// variable



// WiFi / MQTT credentials
const char* ssid = "Mathurot";
const char* password = "ikbennietgeborenop9augustus";

const char* mqtt_server = "e303fcc2b2f344838220e08b26e702b3.s1.eu.hivemq.cloud";
const int   mqtt_port = 8883;
const char* mqtt_user = "arduino";
const char* mqtt_pass = "TestPass123";

constexpr long oldNano_timer1ClockFreq{ 2000000L };
constexpr long oldNano_timer1PWMfreq{ 1000L };
constexpr long oldNano_timer1Top{ oldNano_timer1ClockFreq / oldNano_timer1PWMfreq / 2 };
constexpr float oldNano_ADCmVperStep = 5000. / 1024.;
constexpr long oldNano_fastDataRateSamplingPeriods{ 1 << 7 };

enum rotStatus :uint8_t {
    rotNoPosSync, rotFreeRunning, rotMeasuring, rotUnlocked, rotLocked, // rotNoPosSync: also if rotation OFF or not floating   
    errDroppedGlobe = 0x11, errStickyGlobe, errMagnetLoad, errTemp
};
//// eBlink, eSpareNoDataEvent1: cue only (no data) events. additional time cues can be added
////enum events :uint8_t { eNoEvent = 0, eGreenwich, eStatusChange, eFastRateData, eLedstripData, eStepResponseData, eSecond, eBlink, eSpareNoDataEvent1 };
enum colorCycles :uint8_t { cLedstripOff = 0, cCstBrightWhite, cCstBrightMagenta, cCstBrightBlue, cWhiteBlue, cRedGreenBlue };      // led strip color cycle 
enum colorTiming :uint8_t { cLedstripVeryFast = 0, cLedstripFast, cLedstripSlow, cLedStripVerySlow };                               // led strip color cycle 


const char str_rotationOff[] = "rotation off";
const char str_freeRunning[] = "free running";
const char str_noPosSync[] = "wait for pos sync";
const char str_measuring[] = "measuring";
const char str_notLocked[] = "not locked";
const char str_locked[] = "locked";
const char str_notFloating[] = "not floating";
const char str_ErrDroppedGlobe[] = "E! dropped globe";
const char str_ErrStickyGlobe[] = "E! sticky globe";
const char str_ErrOverload[] = "E! overload";
const char str_ErrTemp[] = "E! temp too high";


// ============================================================================
// GLOBALS: MQTT
// ============================================================================
WiFiClientSecure espClient;
PubSubClient  client(espClient);


WireSlave wire_slave_interface;


// ============================================================================
// SETUP
// ============================================================================
void setup()
{
    Serial.begin(115200);
    delay(5000);

    pinMode(13, OUTPUT);

    // WiFi
    Serial.print("Connecting to WiFi ");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(300);
    }
    Serial.println("\nWiFi connected.");
    Serial.print("IP = ");
    Serial.println(WiFi.localIP());

    // MQTT
    /*
    espClient.setCACert(root_ca);                   // Set the Root CA for the secure client
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(MQTTmessageReceived);
    reconnectMQTT();
    */
}

// ============================================================================
// LOOP
// ============================================================================
void loop()
{
    /*
    if (!client.connected())
        reconnectMQTT();
    client.loop();
    */

    if (!processI2CmessageFromMaster()) {};
}


bool processI2CmessageFromMaster() {
    uint8_t messageTypeIn{};          // as received
    uint8_t payloadSizeIn{};          // as received
    uint8_t plIn[WireSlave::PAYLOAD_IN_MAX];

    bool msgAvailable = wire_slave_interface.dequeueRx(messageTypeIn, plIn, payloadSizeIn);
    if (!msgAvailable) { return false; }

    Serial.print("message: "); Serial.println(messageTypeIn, HEX);

    switch (messageTypeIn)
    {
        case MsgType::M_MSG_HELLO:
        {
            Serial.println("\n\nhello received");
            wire_slave_interface.enqueueTx(MsgType::S_MSG_HELLO_ACK, nullptr, 0);             // no payload
         }
         break;


        // ========== message types in: RECEIVE DATA from master after an event ==========

        case MsgType::M_MSG_GREENWICH:
        {
            Serial.println("\n\nGreenwich received");
            
            I2C_s_ack p;
            p.ack = 0x0; //// 0; //dummy
            p.requestMasterMsgType = MsgType::M_MSG_NONE;
            wire_slave_interface.enqueueTx(MsgType::S_MSG_ACK, &p, sizeof(p));
        }
        break;

        case MsgType::M_MSG_STATUS:
        {
            Serial.println("\n\nStatus received");
            
            I2C_s_ack p;
            p.ack = 0x0; //// 0; //dummy
            p.requestMasterMsgType = MsgType::M_MSG_NONE;
            wire_slave_interface.enqueueTx(MsgType::S_MSG_ACK, &p, sizeof(p));
        }
        break;

        case MsgType::M_MSG_SECOND:
        {
            Serial.println("\n\Second received");
            bool success = processMsgIn_second(payloadSizeIn, plIn);
            if (!success) { Serial.println("msg in ERROR <<<<"); }

            I2C_s_ack p;
            p.ack = 0x0; //// seq low byte //// 0; //dummy
            p.requestMasterMsgType =  MsgType::M_MSG_NONE; //// MsgType::M_MSG_REQ_GLOBE_SETTINGS; ////
            wire_slave_interface.enqueueTx(MsgType::S_MSG_ACK, &p, sizeof(p));
        }
        break;


        // ========== message types in: RECEIVE DATA from master when requested by slave ==========

        // receive wire master library tx stats 
        case MsgType::M_MSG_SEND_STATS:
        {
            I2C_s_ack p;
            p.ack = 0x0; //// 0; //dummy
            p.requestMasterMsgType = MsgType::M_MSG_NONE;
            wire_slave_interface.enqueueTx(MsgType::S_MSG_ACK, &p, sizeof(p));
        }
        break;

        // receive wire master library rx stats
        case MsgType::M_MSG_RECEIVE_STATS:
        {
            I2C_s_ack p;
            p.ack = 0x0; //// 0; //dummy
            p.requestMasterMsgType = MsgType::M_MSG_NONE;
            wire_slave_interface.enqueueTx(MsgType::S_MSG_ACK, &p, sizeof(p));
        }
        break;

        // receive wire master message library stats
        case MsgType::M_MSG_MESSAGE_STATS:
        {
            I2C_s_ack p;
            p.ack = 0x0; //// 0; //dummy
            p.requestMasterMsgType = MsgType::M_MSG_NONE;
            wire_slave_interface.enqueueTx(MsgType::S_MSG_ACK, &p, sizeof(p));
        }
        break;

        // receive spinning globe event stats
        case MsgType::M_MSG_GLOBE_STATS:
        {

            I2C_s_ack p;
            p.ack = 0x0; //// 0; //dummy
            p.requestMasterMsgType = MsgType::M_MSG_NONE;
            wire_slave_interface.enqueueTx(MsgType::S_MSG_ACK, &p, sizeof(p));
        }
        break;

        // receive globe settings (changeable globe attributes)
        case MsgType::M_MSG_GLOBE_SETTINGS:
        {
            I2C_s_globeSettings* globeIn = reinterpret_cast<I2C_s_globeSettings*>(plIn);
            if (payloadSizeIn != sizeof(*globeIn)) { Serial.println("PAYLOAD SIZE <<<<<<"); return false; }               // inconsistency between master and slave: forget message
            Serial.println("Globe settings: adjust steps (received):");
            Serial.print("    rotation time   "); Serial.println(globeIn->userSet_rotationPeriod);
            Serial.print("    led effect      "); Serial.println(globeIn->userSet_ledEffect);
            Serial.print("    led cycle speed "); Serial.println(globeIn->userSet_ledCycleSpeed);

            I2C_s_ack p;
            p.ack = 0x0; //// 0; //dummy
            p.requestMasterMsgType = MsgType::M_MSG_NONE;
            wire_slave_interface.enqueueTx(MsgType::S_MSG_ACK, &p, sizeof(p));
        }
        break;

        // receive PID controller settings
        case MsgType::M_MSG_PID_SETTINGS:
        {
            I2C_s_PIDsettings* PIDin = reinterpret_cast<I2C_s_PIDsettings*>(plIn);
            if (payloadSizeIn != sizeof(*PIDin)) { Serial.println("PAYLOAD SIZE <<<<<<"); return false; }               // inconsistency between master and slave: forget message
            Serial.println("PID controller: adjust steps (received):");
            Serial.print("    gain          "); Serial.println(PIDin->gainAdjustSteps);
            Serial.print("    int. time cst "); Serial.println(PIDin->intTimeCstAdjustSteps);
            Serial.print("    dif. time cst "); Serial.println(PIDin->difTimeCstAdjustSteps);

            I2C_s_ack p;
            p.ack = 0x0; //// 0; //dummy
            p.requestMasterMsgType = MsgType::M_MSG_NONE;
            wire_slave_interface.enqueueTx(MsgType::S_MSG_ACK, &p, sizeof(p));
        }
        break;

        // receive vertical position setpoint (mV)
        case MsgType::M_MSG_VERT_POS_SETPOINT:
        {
            I2C_s_vertPosSetpoint* vertPosIn = reinterpret_cast<I2C_s_vertPosSetpoint*>(plIn);
            if (payloadSizeIn != sizeof(*vertPosIn)) { Serial.println("PAYLOAD SIZE <<<<<<"); return false; }               // inconsistency between master and slave: forget message
            Serial.print("vert. pos. setpoint "); Serial.println(vertPosIn->userSet_vertPosIndex);

            I2C_s_ack p;
            p.ack = 0x0; //// 0; //dummy
            p.requestMasterMsgType = MsgType::M_MSG_NONE;
            wire_slave_interface.enqueueTx(MsgType::S_MSG_ACK, &p, sizeof(p));
        }
        break;

        // receive coil phase adjust (degrees)
        case MsgType::M_MSG_COIL_PHASE_ADJUST:
        {
            I2C_s_coilPhaseAdjust* coilPhaseIn = reinterpret_cast<I2C_s_coilPhaseAdjust*>(plIn);
            if (payloadSizeIn != sizeof(*coilPhaseIn)) { Serial.println("PAYLOAD SIZE <<<<<<"); return false; }               // inconsistency between master and slave: forget message
            Serial.print("coil phase adjust: "); Serial.println(coilPhaseIn->userSet_coilPhaseAdjust);

            I2C_s_ack p;
            p.ack = 0x0; //// 0; //dummy
            p.requestMasterMsgType = MsgType::M_MSG_NONE;
            wire_slave_interface.enqueueTx(MsgType::S_MSG_ACK, &p, sizeof(p));
        }
        break;

        // ========== message types in: RECEIVE REQUEST from master to send data ==========

        case MsgType::M_MSG_REQ_GLOBE_SETTINGS:
        {
            I2C_s_globeSettings p{};
            p.userSet_rotationPeriod = 5;      // test
            p.userSet_ledEffect = 5;
            p.userSet_ledCycleSpeed = 0;
            // but master needs to observe a delay between this message it sent and the response expected, because this response will only be enqueued now
            // and must be ready in the queue when 'handleRequest()' is triggered  
            wire_slave_interface.enqueueTx(MsgType::S_MSG_GLOBE_SETTINGS, &p, sizeof(p));
        }
        break;

        case MsgType::M_MSG_REQ_PID_SETTINGS:
        {
            I2C_s_PIDsettings p{};
            p.gainAdjustSteps = 16;      // test
            p.intTimeCstAdjustSteps = 16;
            p.difTimeCstAdjustSteps = 16;
            wire_slave_interface.enqueueTx(S_MSG_PID_SETTINGS, &p, sizeof(p));
        }
        break;

        case MsgType::M_MSG_REQ_VERT_POS_SETPOINT:
        {
            I2C_s_vertPosSetpoint p{};
            p.userSet_vertPosIndex = 0;      // test
            wire_slave_interface.enqueueTx(S_MSG_VERT_POS_SETPOINT, &p, sizeof(p));
        }
        break;

        case MsgType::M_MSG_REQ_COIL_PHASE_ADJUST:
        {
            I2C_s_coilPhaseAdjust p{};
            p.userSet_coilPhaseAdjust = 0;      // test
            wire_slave_interface.enqueueTx(S_MSG_COIL_PHASE_ADJUST, &p, sizeof(p));
        }
        break;

        default:
        {}  // unknown message type: do nothing (master will time out)
        break;
         
    }
    return true;
}


bool processMsgIn_second(uint8_t payloadSizeIn, void* ppl) {
    long tempSmoothed;
    float magnetDutyCycle, isrDuration, idle, vertPosAvgError;

    I2C_m_secondCue* p = reinterpret_cast<I2C_m_secondCue*>(ppl);
    if (payloadSizeIn != sizeof(*p)) { Serial.println("PAYLOAD SIZE <<<<<<"); return false; }               // inconsistency between master and slave: forget message

    tempSmoothed = p->tempSmooth;//// doe het zo
    magnetDutyCycle = p->magnetOnCyclesSmooth;
    isrDuration = p->ISRdurationSmooth;
    idle = p->idleLoopMicrosSmooth;
    vertPosAvgError = p->errSignalMagnitudeSmooth;

    float tempC = tempSmoothed / 100.0f;
    magnetDutyCycle = (magnetDutyCycle / oldNano_fastDataRateSamplingPeriods) * 100.0f / (float)oldNano_timer1Top;
    isrDuration /= oldNano_fastDataRateSamplingPeriods;
    float load = ((1000.0f * oldNano_fastDataRateSamplingPeriods - idle)
        * 100.0f / (1000.0f * oldNano_fastDataRateSamplingPeriods));
    vertPosAvgError = (vertPosAvgError / oldNano_fastDataRateSamplingPeriods) * oldNano_ADCmVperStep;


/*    char s[32];
    formatFloat(s, sizeof(s), tempC, 1);   Serial.print("T = "); strcat(s, " °C"); Serial.println(s);
    ////client.publish("globe/temperature", s);
    formatFloat(s, sizeof(s), magnetDutyCycle, 1);  Serial.print("M = "); strcat(s, "%"); Serial.println(s);
    ////client.publish("globe/magnetDutyCycle", s);
    formatFloat(s, sizeof(s), isrDuration, 0);  Serial.print("ISR = "); strcat(s, " us"); Serial.println(s);
    ////client.publish("globe/ISR duration", s);
    formatFloat(s, sizeof(s), load, 1);    Serial.print("Load = "); strcat(s, "%"); Serial.println(s);
    ////client.publish("globe/processor load", s);

    Serial.print("Vpos err ");
    strcpy(s, " ---mV");                        // init
    //if ((statusData.isFloating) {                         // from latest fast rate update event before write
    formatFloat(s, sizeof(s), vertPosAvgError, 1); Serial.print(" = "); strcat(s, "mV"); Serial.println(s);
*/

//}
////client.publish("globe/????????", s);

    return true;    // success
}


// ============================================================================
// SAFE FLOAT FORMATTER (no dtostrf, no snprintf)
// ============================================================================
// ---------------------------------------------------------------------------
// Safe, robust formatter for floats with rounding and no %f dependency.
// Writes into 'out' with size 'outSize'.
// Example: formatFloat(out, 3.14159, 2) -> "3.14"
// ---------------------------------------------------------------------------
// Writes the formatted float into `out`, guaranteed null-terminated.
// outSize must be >= 4 (for "-0.0\0").
// decimals: number of digits after decimal point (0..6).
void formatFloat(char* out, size_t outSize, float value, uint8_t decimals)
{
    if (outSize == 0) return;          // no room
    out[0] = '\0';                     // always produce valid string

    if (decimals > 6) decimals = 6;    // safe limit

    bool neg = (value < 0);
    if (neg) value = -value;

    // multiplier = 10^decimals
    uint32_t mul = 1;
    for (uint8_t i = 0; i < decimals; i++)
        mul *= 10U;

    // scaled and rounded integer representation
    uint32_t scaled = (uint32_t)(value * mul + 0.5f);

    uint32_t whole = scaled / mul;
    uint32_t frac = scaled % mul;

    // --- Build the string manually ---
    char temp[32];      // local build buffer — safe on ARM
    size_t idx = 0;

    // sign
    if (neg) temp[idx++] = '-';

    // whole part
    idx += snprintf(temp + idx, sizeof(temp) - idx, "%lu", (unsigned long)whole);

    // fractional part
    if (decimals > 0) {
        temp[idx++] = '.';

        // Leading zeros for fractional part
        uint32_t t = mul / 10;
        while (t > 1 && frac < t) {
            temp[idx++] = '0';
            t /= 10;
        }

        // actual fractional value
        idx += snprintf(temp + idx, sizeof(temp) - idx, "%lu", (unsigned long)frac);
    }

    temp[idx] = '\0';

    // Copy to user buffer safely
    strncpy(out, temp, outSize - 1);
    out[outSize - 1] = '\0';
}


// ============================================================================
// MQTT CALLBACK
// ============================================================================
void MQTTmessageReceived(char* topic, byte* payload, unsigned int length)
{
    String msg;
    for (unsigned int i = 0; i < length; i++)
        msg += (char)payload[i];

    Serial.print("MQTT received: ");
    Serial.println(msg);
}

// ============================================================================
// MQTT RECONNECT
// ============================================================================
void reconnectMQTT() {

    const char* clientId = "ESP32Client1";  // Your unique client ID

    Serial.print("Connecting to MQTT...");
    while (!client.connected()) {
       // Use the connect method with clientId, username, and password
        if (client.connect(clientId, mqtt_user, mqtt_pass)) {
            Serial.println("connected.");
            // Subscribe or publish here if needed
            client.subscribe("globe/led");

        }
        else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" retrying in 2 seconds");
            delay(2000);
        }
    }
}




#if 0 

void processWireMessage(uint8_t messageType) {


    switch (rxWorking.id) {
        case MSG_SECONDS_EVENT:
        {
            long tempSmoothed;
            float magnetDutyCycle, isrDuration, idle, vertPosAvgError;
            /*
            memcpy(&tempSmoothed, rxWorking.payload + 0, 4);
            memcpy(&magnetDutyCycle, rxWorking.payload + 4, 4);
            memcpy(&isrDuration, rxWorking.payload + 8, 4);
            memcpy(&idle, rxWorking.payload + 12, 4);
            memcpy(&vertPosAvgError, rxWorking.payload + 16, 4);
            */
            float tempC = tempSmoothed / 100.0f;
            magnetDutyCycle = (magnetDutyCycle / oldNano_fastDataRateSamplingPeriods) * 100.0f / (float)oldNano_timer1Top;
            isrDuration /= oldNano_fastDataRateSamplingPeriods;
            float load = ((1000.0f * oldNano_fastDataRateSamplingPeriods - idle)
                * 100.0f / (1000.0f * oldNano_fastDataRateSamplingPeriods));
            vertPosAvgError = (vertPosAvgError / oldNano_fastDataRateSamplingPeriods) * oldNano_ADCmVperStep;

            char s[32];
            formatFloat(s, sizeof(s), tempC, 1);   Serial.print("T = "); strcat(s, " °C"); Serial.println(s);
            ////client.publish("globe/temperature", s);
            formatFloat(s, sizeof(s), magnetDutyCycle, 1);  Serial.print("M = "); strcat(s, "%"); Serial.println(s);
            ////client.publish("globe/magnetDutyCycle", s);
            formatFloat(s, sizeof(s), isrDuration, 0);  Serial.print("ISR = "); strcat(s, " us"); Serial.println(s);
            ////client.publish("globe/ISR duration", s);
            formatFloat(s, sizeof(s), load, 1);    Serial.print("Load = "); strcat(s, "%"); Serial.println(s);
            ////client.publish("globe/processor load", s);
        }
        break;

        case MSG_GREENWICH_EVENT:
        {
            bool isFloating{};
            char rotationStatus{};      // status or (b4 set) error condition
            long lastRotationTime{}, rotationOutOfSyncTime{}, greenwichLag{};
             //// status, error status, ..., rotation time

            rotationStatus = rxWorking.payload[0];       // bit 4: flag 'is error condition'
            isFloating = (bool)rxWorking.payload[1];
            memcpy(&lastRotationTime, rxWorking.payload + 2, 4);
            memcpy(&rotationOutOfSyncTime, rxWorking.payload + 6, 4);
            memcpy(&greenwichLag, rxWorking.payload + 10, 4);

            float rotTime = lastRotationTime / 1000.f;      // seconds
            float rotOutOfSyncTime = rotationOutOfSyncTime / 1000.f;
            int angle = (greenwichLag * 360) / targetGlobeRotationTime;


            char s[32];
            char mqttStatus[2]; mqttStatus[1] = '\0';

            // determine status text
            switch (rotationStatus) {
                case rotNoPosSync:
                    strcpy(s, isFloating ? ((targetGlobeRotationTime == 0) ? str_rotationOff : str_noPosSync) : str_notFloating);
                    mqttStatus[0] = (isFloating ? ((targetGlobeRotationTime == 0) ? '0' : '2') : '6');
                    break;
                case rotFreeRunning: strcpy(s, str_freeRunning); mqttStatus[0] = '1'; break;
                case rotMeasuring: strcpy(s, str_measuring); mqttStatus[0] = '3'; break;
                case rotUnlocked: strcpy(s, str_notLocked); mqttStatus[0] = '4'; break;
                case rotLocked: strcpy(s, str_locked); mqttStatus[0] = '5'; break;
                case errDroppedGlobe: strcpy(s, str_ErrDroppedGlobe); mqttStatus[0] = 'A'; break;
                case errStickyGlobe: strcpy(s, str_ErrStickyGlobe); mqttStatus[0] = 'B';  break;
                case errMagnetLoad: strcpy(s, str_ErrOverload); mqttStatus[0] = 'C'; break;
                case errTemp: strcpy(s, str_ErrTemp); mqttStatus[0] = 'D'; break;
            }

            ////client.publish("globe/status", mqttStatus);
            Serial.println(s);

            if (rotationStatus >= rotUnlocked) { formatFloat(s, sizeof(s), rotTime, 2); strcat(s, " s"); }
            else { strcpy(s, "-.-- s"); }
            ////client.publish("globe/lastRotationTime", s);
            Serial.print("Last rotation time = "); Serial.println(s);

            if (rotationStatus == rotLocked) { formatFloat(s, sizeof(s), rotOutOfSyncTime, 2); }
            else { strcpy(s, "-.-- s"); }
            ////client.publish("globe/rotationOutOfSyncTime", s);
            Serial.print("Rotation sync error = "); strcat(s, " s"); Serial.println(s);

            snprintf(s, 32, "%+1.1d", angle); strcat(s, "°");
            ////client.publish("globe/greenwichLagAngle", s);
            Serial.print("Greenwich lag = "); Serial.println(s);
        }
    }
// continue draining any further packets
}

#endif