#include "MQTTmessages.h"
#include "secrets.h"
#include "debug.h"

MQTTmessages* MQTTmessages::_instance = nullptr;

MQTTmessages::MQTTmessages(SharedContext& sharedContext) :_client(_espClient), _sharedContext(sharedContext) {
    _instance = this;

    _espClient.setCACert(ROOT_CA);                   // Set the Root CA for the secure client
    _client.setServer(MQTT_SERVER, MQTT_PORT);
    _client.setCallback(mqttCallback);
}


void MQTTmessages::loop() {
    // first, maintain WiFi and MQTT connections
    bool WiFiConnected = _wifiConnection.maintainWiFi();
    bool MQTTconnected = maintainMQTT(WiFiConnected);
    
    // .publish(...) builds an MQTT PUBLISH packet, writes it into the underlying TCP client, returns immediately(success or failure)
    if (MQTTconnected) {
        MsgToMQTT* pMsg{};
        if (!_sharedContext.queueToMQTT.empty()) {
            pMsg = _sharedContext.queueToMQTT.front();
            _client.publish(pMsg->topic, pMsg->payload);
            _sharedContext.queueToMQTT.pop(*pMsg);
        }
    }

   // handles MQTT keep-alive, incoming MQTT packets; detects broken connections, calls the MQTT user callback  
    _client.loop(); // placed at the end, after all states machines ran, after an eventual publish that was just queued                                    
}


// ============================================================================
// MQTT CALLBACK
// ============================================================================

// --- static trampoline ---

void MQTTmessages::mqttCallback(char* topic, byte* payload, unsigned int length)
{
    if (_instance) {
        _instance->handleMQTTmessage(topic, payload, length);
    }
}
void MQTTmessages::handleMQTTmessage(char* topic, byte* payload, unsigned int length)
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
bool MQTTmessages::maintainMQTT(bool WiFiConnected) {

    // Variable '_mqttState' maintains the state of the MQTT connection('state machine').If this maintained state
    // (e.g., 'MQTT connected') does not correspond to the actual state(e.g., MQTT connection was lost), action is taken

    uint32_t now = millis();

    if (!WiFiConnected) { _mqttState = MQTT_notConnected; return false; }

    switch (_mqttState) {

        // state: MQTT is currently not connected
        // --------------------------------------
        case MQTT_notConnected:
        {
            // time for a next MQTT connection attempt ?
            if (now - _lastMqttMaintenanceTime > MQTT_UP_CHECK_INTERVAL) {
                _client.connect(MQTT_DEVICE_ID, MQTT_USER, MQTT_PASS);
                _mqttState = MQTT_waitForConnecton;

                if (DEBUG) {
                    char s[100]; sprintf(s, "-- at %11.3fs: %s", now / 1000., "Trying to connect to MQTT...");
                    DEBUG_PRINTLN(s);
                }

                // remember time of this MQTT connection attempt; this is also the time of the last debug print (if enabled)
                _lastMqttWaitReportedAt = _lastMqttMaintenanceTime = now;
            }
        }
        break;


        // state: currently waiting for MQTT connection
        // --------------------------------------------
        case MQTT_waitForConnecton:
        {
            // time for a next MQTT connection check ?
            if (now - _lastMqttMaintenanceTime > MQTT_UP_CHECK_INTERVAL) {
                if (_client.connected()) {                                        // MQTT is now connected ?
                    _mqttState = MQTT_connected;
                    if (DEBUG) {
                        char s[120]; sprintf(s, "-- at %11.3fs: MQTT connected", now / 1000.);
                        DEBUG_PRINTLN(s);
                    }
                }

                else {                                                                      // MQTT is not yet connected
                    // regularly report status ('still trying...' etc.)
                    if (DEBUG) {
                        if (now - _lastMqttWaitReportedAt > MQTT_REPORT_INTERVAL) {
                            _lastMqttWaitReportedAt = now;                              // for printing only
                            DEBUG_PRINT(".");
                        }
                    }
                }
                _lastMqttMaintenanceTime = now;                                        // remember time of last MQTT maintenance 
            }
        }
        break;


        // state: MQTT is connected
        // ------------------------
        case MQTT_connected:
        {
            //  prepare for reconnection if connection is lost OR per user program request 
            if (!_client.connected()) {
                _mqttState = MQTT_notConnected;

                if (DEBUG) {
                    char s[100]; sprintf(s, "-- at %11.3fs: %s%d", now / 1000., "MQTT disconnected, client state = ", _client.state());
                    DEBUG_PRINTLN(s);
                }

                _lastMqttMaintenanceTime = now;                                        // remember time of last MQTT maintenance 
            }
        }
        break;
    }

    return (_mqttState == MQTT_connected);

}

