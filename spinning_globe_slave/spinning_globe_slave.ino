#include <esp_task_wdt.h>
#include "src/wireSlave_messages.h"
#include "src/MQTTmessages.h"
#include "src/time_helpers.h"

constexpr uint32_t WDT_TIMEOUT = 60;        // 60 seconds

SharedContext sharedContext;
WireSlaveMessages wireSlaveMessages(sharedContext);
WiFiConnection* pWiFiConnection = nullptr;
MQTTmessages* pMqttMessages = nullptr;

void setWiFiLeds(WiFiConnection::ConnectionState, MQTTmessages::ConnectionState);

// ============================================================================
// SETUP
// ============================================================================
void setup()
{
    Serial.begin(115200);
    delay(5000);

    Serial.println("=== BUILD DATE AND TIME ===");
    Serial.print(__DATE__); Serial.print(' '); Serial.println(__TIME__);

    // do not define before setup() runs
    pWiFiConnection = new WiFiConnection;
    pMqttMessages = new MQTTmessages(sharedContext);

    pinMode(LED_BUILTIN, OUTPUT);

    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);

    // Initialize Task Watchdog Timer (enable, set timeout) and add current task (loop) to WDT
    esp_task_wdt_init(WDT_TIMEOUT, true);
    esp_task_wdt_add(NULL);
}


// ============================================================================
// LOOP
// ============================================================================
void loop()
{
    // maintain both the wire slave and MQTT connections and send messages back and forth
    WiFiConnection::ConnectionState wifiConnectionState{ WiFiConnection::WiFi_notConnected };
    MQTTmessages::ConnectionState mqttConnectionState{ MQTTmessages::MQTT_notConnected };

    // that's all there is (here): this nano esp32 acts as a bridge between MQTT and the spinning globe classic nano
    if (pWiFiConnection) { wifiConnectionState = pWiFiConnection->maintainWiFi(); }
    if (pMqttMessages) { mqttConnectionState = pMqttMessages->loop(wifiConnectionState == WiFiConnection::WiFi_connected); }
    wireSlaveMessages.loop();

    setWiFiLeds(wifiConnectionState, mqttConnectionState);

    esp_task_wdt_reset();           // reset the watchdog

    if (DEBUG) {
        static bool timeIsPrinted{ false };
        char ntpTimeBuf[32];
        if (!timeIsPrinted) {
            if (timeHelpers::getLocalTimeString(ntpTimeBuf, sizeof(ntpTimeBuf))) {
                char s[80]; snprintf(s, sizeof(s), "-- at %13.3fs : time is now set to %s", millis() / 1000., ntpTimeBuf);
                timeIsPrinted = true;
                DEBUG_PRINTLN(s);
            }
        }
    }
}


// ============================================================================
// Use D13 led to indicate wire transmissions are active
// Use RGB led to display current WiFi / MQTT state
// ============================================================================
void setWiFiLeds(WiFiConnection::ConnectionState wifiState, MQTTmessages::ConnectionState mqttState) {

    uint32_t now = millis();

    // ---------- WiFi and MQTT state ----------

    constexpr int32_t LED_CONN_BLINK_PERIOD = 0x100;    // led blink control while WiFi / MQTT are connecting: led on if (millis() & LED_CONN_BLINK_PERIOD): 256 ms ON + 256 ms OFF = 512 ms blink period
    constexpr int32_t LED_NEW_MSG_PERIOD = 0x64;        // new message flash: change color for 100 ms if MQTT message is sent to, or received from broker

    constexpr int32_t LED_DIM_MASK = 0xF;               // led brightness control: reduce brightness to 1 / 16; fastest led state change = 1 ms; T = 16 ms(62.5 Hz: no flicker visible)

    enum COMM_STATE { LED_WiFiNotConnected, LED_WiFiConnecting, LED_WiFiConnected, LED_MQTTconnecting, LED_MQTTconnectedIdle, LED_MQTTconnectedNewMessage };    //  small state machine

    static bool red{}, green{}, blue{};
    static bool lastRed{ !red }, lastGreen{ !green }, lastBlue{ !blue };

    static uint32_t mqttTxmit_startTime{ now };
    static bool newMQTTmessage{ false };
    static COMM_STATE comm_state{};

    // 1. determine whether an MQTT message was recently transmitted, or is transmitted, just now
    if (mqttState != MQTTmessages::MQTT_connected) { newMQTTmessage = false; }                              // MQTT even not connected: no new message
    else if (pMqttMessages->newMQTTmessage()) { mqttTxmit_startTime = now; newMQTTmessage = true; }         // new MQTT message transmitted just now: start timer
    else if (now - mqttTxmit_startTime > LED_NEW_MSG_PERIOD) { newMQTTmessage = false; }                    // no recent and  no new MQTT message

    // 2. calculate state to report (RGB led)
    comm_state = (wifiState == WiFiConnection::WiFi_notConnected) ? LED_WiFiNotConnected :
        (wifiState == WiFiConnection::WiFi_waitForConnecton) ? LED_WiFiConnecting :
        (mqttState == MQTTmessages::MQTT_notConnected) ? LED_WiFiConnected :
        (mqttState == MQTTmessages::MQTT_waitForConnecton) ? LED_MQTTconnecting :
        newMQTTmessage ? LED_MQTTconnectedNewMessage : LED_MQTTconnectedIdle;

    // 3. set RGB led according to state calculated above
    switch (comm_state) {
        case LED_WiFiNotConnected:
            red = green = blue = false;                             // RGB led: off
            break;

        case LED_WiFiConnecting:
            red = green = (bool)(now & LED_CONN_BLINK_PERIOD);          // RGB led: yellow, blinking
            blue = false;
            break;

        case LED_WiFiConnected:                                     // RGB led: yellow 
            red = green = true;
            blue = false;
            break;

        case LED_MQTTconnecting:
            green = (bool)(now & LED_CONN_BLINK_PERIOD);                // RGB led: green, blinking
            red = blue = false;
            break;

        case LED_MQTTconnectedIdle:                                 // RGB led: green                               
            green = true;
            red = blue = false;
            break;

        case LED_MQTTconnectedNewMessage:                           // RGB led: magenta
            green = false;
            red = blue = true;
            break;
    }

    // 4. reduce brightness 
    bool dimmedRed{}, dimmedGreen{}, dimmedBlue{};

    dimmedRed = red && !((now & LED_DIM_MASK));
    dimmedGreen = green && !((now & LED_DIM_MASK));
    dimmedBlue = blue && !((now & LED_DIM_MASK));

    // 5. only write to pins when pin output changes
    if (dimmedRed != lastRed) { lastRed = dimmedRed; digitalWrite(LED_RED, !dimmedRed); }             // note that RGB led is ON when pin output is LOW          
    if (dimmedGreen != lastGreen) { lastGreen = dimmedGreen; digitalWrite(LED_GREEN, !dimmedGreen); }
    if (dimmedBlue != lastBlue) { lastBlue = dimmedBlue; digitalWrite(LED_BLUE, !dimmedBlue); }


    // ---------- wire transmission ----------

    constexpr int32_t WIRE_LED_NEW_MSG_PERIOD = 0x64;    // new message flash: change color for 100 ms if MQTT message is sent to, or received from broker

    constexpr int32_t WIRE_LED_DIM_MASK = 0xF;           // led brightness control: reduce brightness to 1 / 16; fastest led state change = 1 ms; T = 16 ms(62.5 Hz: no flicker visible)

    static bool ledState{ false }, lastLedState{ false };
    static uint32_t wireCommStartTime{};

    if (sharedContext.triggerWireCommLed) { wireCommStartTime = now; ledState = true; sharedContext.triggerWireCommLed = false; }
    if ((ledState) && (now - wireCommStartTime > WIRE_LED_NEW_MSG_PERIOD)) { ledState = false; }

    // led ON (1/16 brightness) for a full second each time a trigger occurs (prevents blinking)
    bool dimmedLedState = ledState && !((now & WIRE_LED_DIM_MASK));
    if (dimmedLedState != lastLedState) { lastLedState = dimmedLedState; digitalWrite(LED_BUILTIN, dimmedLedState); }

}





