#include <esp_task_wdt.h>
#include "wireSlave_messages.h"
#include "MQTTmessages.h"

constexpr uint32_t WDT_TIMEOUT = 5;

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
    pinMode(WIRE_RECEIVE_LED, OUTPUT);

    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);

    // Initialize Task Watchdog Timer (enable, 5s timeout) and add current task (loop) to WDT
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
}


// ============================================================================
// LOOP
// ============================================================================
void setWiFiLeds(WiFiConnection::ConnectionState wifiState, MQTTmessages::ConnectionState mqttState) {

    uint32_t now = millis();
    bool dimmedRed{}, dimmedGreen{}, dimmedBlue{};

    static bool red{}, green{}, blue{};
    static bool lastRed{ !red }, lastGreen{ !green }, lastBlue{ !blue };

    static uint32_t indicateMQTTxmit_start{ now };
    static bool MQTTtransmitFlag{ false };

    // no MQTT transmission underway: show connection status
    if (!MQTTtransmitFlag) {
        if (pMqttMessages->getMQTTtransmitFlag()) {     // and clear the flag 
            MQTTtransmitFlag = true;
            indicateMQTTxmit_start = now;
            red = blue = true;                                     // 1/16 duty cycle
            green = false;
        }
    }


    if (MQTTtransmitFlag) {
        if (millis() - indicateMQTTxmit_start > 100) {                   // stay on for a number of ms after last data was received
            green = true;
            red = blue = false;
            MQTTtransmitFlag = false;
        }
    }
    
    if(!MQTTtransmitFlag){
        bool wifiIndicatorOn = (wifiState == WiFiConnection::WiFi_notConnected) ? false : (wifiState == WiFiConnection::WiFi_connected) ? true :
            (bool)(millis() & 0x00200);                       //1/2 duty cycle, lower frequency to show as blinking (T = 1 s)
        bool mqttIndicatorOn = (mqttState == MQTTmessages::MQTT_notConnected) ? false : (mqttState == MQTTmessages::MQTT_connected) ? true :
            (bool)(millis() & 0x00200);                       //1/2 duty cycle, lower frequency to show as blinking (T = 1 s)

        red = (wifiIndicatorOn && !mqttIndicatorOn) ; 
        green = (wifiIndicatorOn || mqttIndicatorOn); 
        blue = false ;                                
    }

    // reduce brightness to 1/8; fastest led state change = 2 ms; T = 16 ms (no flicker visible)
    dimmedRed = red && !((millis() & 0x000e));
    dimmedGreen = green && !((millis() & 0x000e));
    dimmedBlue = blue && !((millis() & 0x000e));

    if (dimmedRed != lastRed) { lastRed = dimmedRed; digitalWrite(LED_RED, !dimmedRed); }             // negative logic          
    if (dimmedGreen != lastGreen) { lastGreen = dimmedGreen; digitalWrite(LED_GREEN, !dimmedGreen); }
    if (dimmedBlue != lastBlue) { lastBlue = dimmedBlue; digitalWrite(LED_BLUE, !dimmedBlue); }
}





