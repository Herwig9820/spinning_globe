#include "MQTTmessages.h"
#include "secrets.h"
#include "debug.h"
#include "json_helpers.h"

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
    unsigned int tmp;

    // .publish(...) builds an MQTT PUBLISH packet, writes it into the underlying TCP client, returns immediately(success or failure)
    if (MQTTconnected) {
        MsgToMQTT* pMsgToMQTT{};
        if (!_sharedContext.queueToMQTT.empty()) {
            pMsgToMQTT = _sharedContext.queueToMQTT.front();
            _client.publish(pMsgToMQTT->topic, pMsgToMQTT->payload);
            _sharedContext.queueToMQTT.pop(*pMsgToMQTT);
        }

        MQTTmsgToWire* pMsgToWire{};
        if (!_sharedContext.queueToWire.empty()) {
            pMsgToWire = _sharedContext.queueToWire.front();

            // switch() requires integral type

            // MQTT has data available: OVERWRITE pending settings slot for that particular data
            if (strcmp(pMsgToWire->topic, TOPIC_GLOBE_SETTINGS_SET) == 0) {
                convertMQTTtoGlobeSettings(pMsgToWire);
            }

            else if (strcmp(pMsgToWire->topic, TOPIC_GLOBE_SETTINGS_REQUEST) == 0) {
                // MSTT requests globe settings
            }

            _sharedContext.queueToWire.pop(*pMsgToWire);
        }
    }

   // handles MQTT keep-alive, incoming MQTT packets; detects broken connections, calls the MQTT user callback  
    _client.loop(); // placed at the end, after all states machines ran, after an eventual publish that was just queued                                    
}

// ============================================================================================
// MQTT globe settings message - PHASE 1: parse payload into 'pending' globe settings structure
// [Phases 2 and 3: in wireSlaveMessages.loop()]
// ============================================================================================
bool MQTTmessages::convertMQTTtoGlobeSettings(MQTTmsgToWire* pMsgToWire) {
    bool ok{ false };
    unsigned int tmp{};

    uint8_t& rotationPeriodIndex = _sharedContext.pendingGlobeSettings.rotationPeriodIndex;
    ok = JsonParse::getUInt(pMsgToWire->payload, "setRotTime", &tmp);
    _sharedContext.pendingGlobeSettings.rotationPeriodIndex = tmp;
    ok &= JsonParse::getUInt(pMsgToWire->payload, "setLedEffect", &tmp);
    _sharedContext.pendingGlobeSettings.ledEffect = tmp;
    ok &= JsonParse::getUInt(pMsgToWire->payload, "setLedEffectSpeed", &tmp);
    _sharedContext.pendingGlobeSettings.ledCycleSpeed = tmp;
    _sharedContext.pendingGlobeSettings.slaveHasData = (uint8_t)ok;       // overwrite previous if not committed in time
    Serial.print("PHASE 1: MQTT convert to wire: set rot time index: "); Serial.println(_sharedContext.pendingGlobeSettings.rotationPeriodIndex);
    return ok;
}

// ============================================================================
// MQTT CALLBACK
// ============================================================================

// --- static trampoline ---

void MQTTmessages::mqttCallback(char* topic, byte* payload, unsigned int length)
{
    if (_instance) {
        _instance->pushIncomingMQTTmsg(topic, payload, length);
    }
}


void MQTTmessages::pushIncomingMQTTmsg(char* topic, byte* payload, unsigned int length)
{
    // do not loose time here decoding the topic to wire msgType en msgLen

    if (_sharedContext.queueToWire.full()) { return; }
    MQTTmsgToWire msg;
    strlcpy(msg.topic, topic, sizeof(msg.topic));
    if(length < sizeof(msg.payload)){
        strlcpy((char*)msg.payload, (const char*)payload, length+1);
    }
    _sharedContext.queueToWire.push(msg);
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
                    _client.subscribe("globe/settings/set");
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

