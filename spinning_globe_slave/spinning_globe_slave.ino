/*
==================================================================================================
Spinning globe extension: using the Wire interface to exchange messages with an Arduino nano esp32.
The nano esp32 acts as a bridge between MQTT and the spinning globe nano (I2C).
over WiFi, e.g. using MQTT.
---------------------------------------------------------------------------------------------------
Copyright 2026 Herwig Taveirne

Program written and tested for Arduino Nano esp32.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

See GitHub for more information and documentation: https://github.com/Herwig9820/spinning_globe

A complete description of the project can be found here:
https://www.instructables.com/Floating-and-Spinning-Earth-Globe/

===============================================================================================
*/


#include <esp_task_wdt.h>
#include "src/wireSlave_messages.h"
#include "src/MQTTmessages.h"
#include "src/time_helpers.h"

#define SPINNING_GLOBE_BRIDGE_VERSION 1.00

constexpr uint32_t WDT_TIMEOUT = 60;        // watchdog timeout 60 seconds

SharedContext sharedContext;
WireSlaveMessages wireSlaveMessages(sharedContext);
WiFiConnection* pWiFiConnection = nullptr;
MQTTmessages* pMqttMessages = nullptr;

void setCommLeds(WiFiConnection::ConnectionState, MQTTmessages::ConnectionState);

// ============================================================================
// SETUP
// ============================================================================
void setup()
{
    Serial.begin(115200);
    delay(5000);
    printWelcome();

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

    // use on-board leds to indicate wire and WiFi/MQTT state
    setCommLeds(wifiConnectionState, mqttConnectionState);

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
void setCommLeds(WiFiConnection::ConnectionState wifiState, MQTTmessages::ConnectionState mqttState) {

    uint32_t now = millis();

    // ---------- WiFi and MQTT state ----------

    constexpr int32_t LED_CONN_BLINK_PERIOD = 0x100;    // led blink control while WiFi / MQTT are connecting: led on if (millis() & LED_CONN_BLINK_PERIOD): 256 ms ON + 256 ms OFF = 512 ms blink period
    constexpr int32_t LED_NEW_MSG_PERIOD = 0xC8;        // new message flash: change color for 200 ms if MQTT message is sent to, or received from broker
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
            red = green = (bool)(now & LED_CONN_BLINK_PERIOD);      // RGB led: yellow, blinking
            blue = false;
            break;

        case LED_WiFiConnected:                                     // RGB led: yellow 
            red = green = true;
            blue = false;
            break;

        case LED_MQTTconnecting:
            green = (bool)(now & LED_CONN_BLINK_PERIOD);            // RGB led: green, blinking
            red = blue = false;
            break;

        case LED_MQTTconnectedIdle:                                 // RGB led: green                               
            green = true;
            red = blue = false;
            break;

        case LED_MQTTconnectedNewMessage:                           // RGB led: white
            red = green = blue = true;
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


// ============================================================================
// Print a welcome message to Serial
// ============================================================================
void printWelcome() {
    Serial.println();
    // string literals On AVR targets (classic nano,...): 
    // if char string used once: use 'F("...")'. Otherwise define strName "..." with PROGMEM and use '(__FlashStringHelper*)strName' 

    for (int i = 0; i < 45; i++) { Serial.print('*'); }; Serial.println();

    Serial.print("Spinning Globe Wire Slave and MQTT bridge - v "); Serial.print(SPINNING_GLOBE_BRIDGE_VERSION); Serial.println();

#if MQTT_BROKER_HIVEMQ
    Serial.println("MQTT broker: HiveMQ Serverless Cloud Cluster");
#else
    Serial.println("MQTT broker: Mosquitto on Raspberry Pi 5");
#endif

    Serial.print("Copyright 2026 "); Serial.println("Herwig Taveirne");

    Serial.print("Build date and time:    "); Serial.print(__DATE__); Serial.print(F("  ")); Serial.print(__TIME__); Serial.println();

    for (int i = 0; i < 45; i++) { Serial.print('*'); } Serial.println("\r\n");
}