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
    _client.setKeepAlive(60);     // seconds; HiveMQ free tier disconnects at 60s idle
    _client.setSocketTimeout(10); // seconds; don't hang forever on a dead TCP socket
}


MQTTmessages::ConnectionState MQTTmessages::loop(bool WiFiConnected) {
    // first, maintain WiFi and MQTT connections
    maintainMQTT(WiFiConnected);

    // .publish(...) builds an MQTT PUBLISH packet, writes it into the underlying TCP client, returns immediately(success or failure)
    if (_mqttState == MQTT_connected) {
        MQTTmsgFromWire* pMsgToMQTT{};
        if (!_sharedContext.queueToMQTT.empty()) {
            _MQTTtransmitFlag = true;
            pMsgToMQTT = _sharedContext.queueToMQTT.front();
            _client.publish(pMsgToMQTT->topic, pMsgToMQTT->payload, pMsgToMQTT->retain);
            _sharedContext.lastMQTTpublish = millis();
            _sharedContext.queueToMQTT.pop(*pMsgToMQTT);
        }

        MQTTmsgToWire* pMsgToWire{};
        if (!_sharedContext.queueToWire.empty()) {
            _MQTTtransmitFlag = true;
            pMsgToWire = _sharedContext.queueToWire.front();

            Serial.print("**** topic received: "); Serial.print(pMsgToWire->topic); Serial.print(", payload: "); Serial.println(pMsgToWire->payload);
            // ------------------------------------------------------------------------------
            // MQTT has data available (typically a setting) to send to wire master ? 
            // convert to wire message and save in temporary buffer for this msg type.
            // ------------------------------------------------------------------------------

            if (strcmp(pMsgToWire->topic, TOPIC_GLOBE_SETTINGS_SET) == 0) {                 // MQTT has globe settings available ?
                convertMQTTtoGlobeSettings(pMsgToWire);
            }
            else if (strcmp(pMsgToWire->topic, TOPIC_PID_SETTINGS_SET) == 0) {             // MQTT has PID settings available ?            
                convertMQTTtoPIDsettings(pMsgToWire);
            }

            else if (strcmp(pMsgToWire->topic, TOPIC_VERT_POS_SETPOINT_SET) == 0) {        // MQTT has vertical pos. setpoint available ?  
                convertMQTTtoVertPosSetpoint(pMsgToWire);
            }
            else if (strcmp(pMsgToWire->topic, TOPIC_COIL_PHASE_ADJUST_SET) == 0) {        // MQTT has coil phase adjustment available ?   
                convertMQTTtoCoilPhaseAdjust(pMsgToWire);
            }

            else if (true) { Serial.print("WRONG TOPIC: "); Serial.println(pMsgToWire->payload); }//// weg

            // ------------------------------------------------------------------------------
            // MQTT REQUESTS wire master (spinning globe) to send message:
            // push the requested message type in a temporary queue
            // ------------------------------------------------------------------------------

            // currently not implemented (no need for it)
            else if (strcmp(pMsgToWire->topic, TOPIC_GLOBE_SETTINGS_REQUEST) == 0) {}       // MQTT requests globe settings ?            
            else if (strcmp(pMsgToWire->topic, TOPIC_PID_SETTINGS_REQUEST) == 0) {}         // MQTT requests PID settings ? 
            else if (strcmp(pMsgToWire->topic, TOPIC_VERT_POS_SETPOINT_REQUEST) == 0) {}    // MQTT requests vertical pos. setpoint ? 
            else if (strcmp(pMsgToWire->topic, TOPIC_COIL_PHASE_ADJUST_REQUEST) == 0) {}    // MSTT requests coil phase adjustment ?


            _sharedContext.queueToWire.pop(*pMsgToWire);
        }
    }

    // handles MQTT keep-alive, incoming MQTT packets; detects broken connections, calls the MQTT user callback  
    _client.loop(); // placed at the end, after all states machines ran, after an eventual publish that was just queued  
    
    return _mqttState;
}

// ============================================================================================
// MQTT globe settings message - PHASE 1: parse payload into 'pending' globe settings structure
// [Phases 2 and 3: in wireSlaveMessages.loop()]
// ============================================================================================

bool MQTTmessages::convertMQTTtoGlobeSettings(MQTTmsgToWire* pMsgToWire) {
    Serial.print("mqtt to wire. Topic (settings) "); Serial.print(pMsgToWire->topic), Serial.print(", payload "); Serial.println(pMsgToWire->payload);

    bool ok{ false };
    unsigned int tmp{};

    ok = JsonParse::getUInt(pMsgToWire->payload, "setRotTime", &tmp);
    _sharedContext.pendingGlobeSettings.rotationPeriodIndex = tmp;
    ok &= JsonParse::getUInt(pMsgToWire->payload, "setLedEffect", &tmp);
    _sharedContext.pendingGlobeSettings.ledEffect = tmp;
    ok &= JsonParse::getUInt(pMsgToWire->payload, "setLedEffectSpeed", &tmp);
    _sharedContext.pendingGlobeSettings.ledCycleSpeed = tmp;
    _sharedContext.pendingGlobeSettings.slaveHasData = (uint8_t)ok;       // overwrite previous if not committed in time
    return ok;
}

bool MQTTmessages::convertMQTTtoPIDsettings(MQTTmsgToWire* pMsgToWire) {

    // Token check: only accept PID settings from local network
    char token[32] = "";
    JsonParse::getString(pMsgToWire->payload, "token", token, sizeof(token));
    if (strcmp(token, SECRET_TOKEN) != 0) {
        return false;   // wrong or missing token: silently ignore
    }

    bool ok{ false };
    unsigned int tmp{};

    ok = JsonParse::getUInt(pMsgToWire->payload, "setGainAdjust", &tmp);
    _sharedContext.pendingPIDsettings.gainAdjustSteps = tmp;
    ok &= JsonParse::getUInt(pMsgToWire->payload, "setDifTimeAdjust", &tmp);
    _sharedContext.pendingPIDsettings.difTimeCstAdjustSteps = tmp;
    ok &= JsonParse::getUInt(pMsgToWire->payload, "setIntTimeAdjust", &tmp);
    _sharedContext.pendingPIDsettings.intTimeCstAdjustSteps = tmp;
    _sharedContext.pendingPIDsettings.slaveHasData = (uint8_t)ok;       // overwrite previous if not committed in time
    return ok;
}

bool MQTTmessages::convertMQTTtoVertPosSetpoint(MQTTmsgToWire* pMsgToWire) {
    Serial.print("mqtt to wire. Topic (vert pos) "); Serial.print(pMsgToWire->topic), Serial.print(", payload "); Serial.println(pMsgToWire->payload);
    // Token check: only accept vertical position setpoint from local network
    char token[32] = "";
    JsonParse::getString(pMsgToWire->payload, "token", token, sizeof(token));
    if (strcmp(token, SECRET_TOKEN) != 0) {
        Serial.println("token not valid");
        return false;   // wrong or missing token: silently ignore
    }

    bool ok{ false };
    unsigned int tmp{};

    ok = JsonParse::getUInt(pMsgToWire->payload, "setVertPosSetpoint", &tmp);
    _sharedContext.pendingVertPosSetpoint.vertPosIndex = tmp;
    _sharedContext.pendingVertPosSetpoint.slaveHasData = (uint8_t)ok;       // overwrite previous if not committed in time

    Serial.print("OK: "); Serial.print(ok);
    Serial.print(", vertical position index: "); Serial.print(_sharedContext.pendingVertPosSetpoint.slaveHasData); Serial.print(", "); Serial.println(_sharedContext.pendingVertPosSetpoint.vertPosIndex);
    return ok;
}

bool MQTTmessages::convertMQTTtoCoilPhaseAdjust(MQTTmsgToWire* pMsgToWire) {
    Serial.print("mqtt to wire. Topic (coil phase) "); Serial.print(pMsgToWire->topic), Serial.print(", payload "); Serial.println(pMsgToWire->payload);
    // Token check: only accept coil phase adjustment from local network
    char token[32] = "";
    JsonParse::getString(pMsgToWire->payload, "token", token, sizeof(token));
    if (strcmp(token, SECRET_TOKEN) != 0) {
        Serial.println("token not valid");
        return false;   // wrong or missing token: silently ignore
    }

    bool ok{ false };
    unsigned int tmp{};

    ok = JsonParse::getUInt(pMsgToWire->payload, "setCoilPhaseAdjust", &tmp);
    _sharedContext.pendingCoilPhaseAdjust.coilPhaseAdjust = tmp;
    _sharedContext.pendingCoilPhaseAdjust.slaveHasData = (uint8_t)ok;       // overwrite previous if not committed in time

    Serial.print("OK: "); Serial.print(ok);
    Serial.print(", coil phase adjust: "); Serial.print(_sharedContext.pendingCoilPhaseAdjust.slaveHasData);
    Serial.print(", "); Serial.println(_sharedContext.pendingCoilPhaseAdjust.coilPhaseAdjust);
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
    if (length < sizeof(msg.payload)) {
        strlcpy((char*)msg.payload, (const char*)payload, length + 1);
    }
    _sharedContext.queueToWire.push(msg);
}


// ============================================================================
// MQTT RECONNECT
// ============================================================================
MQTTmessages::ConnectionState MQTTmessages::maintainMQTT(bool WiFiConnected) {

    // Variable '_mqttState' maintains the state of the MQTT connection('state machine').If this maintained state
    // (e.g., 'MQTT connected') does not correspond to the actual state(e.g., MQTT connection was lost), action is taken

    uint32_t now = millis();

    if (!WiFiConnected) { _mqttState = MQTT_notConnected; return _mqttState; }

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
                    _client.subscribe(TOPIC_GLOBE_SETTINGS_SET);
                    _client.subscribe(TOPIC_PID_SETTINGS_SET);
                    _client.subscribe(TOPIC_VERT_POS_SETPOINT_SET);
                    _client.subscribe(TOPIC_COIL_PHASE_ADJUST_SET);
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
            bool connectionLost = !_client.connected();
            bool publishStalled = _sharedContext.lastMQTTpublish > 0 &&
                (millis() - _sharedContext.lastMQTTpublish > MQTT_PUBLISH_TIMEOUT);

            if (connectionLost or publishStalled) {
                _client.disconnect();
                _mqttState = MQTT_notConnected;
                _lastMqttMaintenanceTime = now;                                        // remember time of last MQTT maintenance 
                if (DEBUG) {
                    char s[100]; sprintf(s, "-- at %11.3fs: %s%d", now / 1000., "MQTT disconnected, client state = ", _client.state());
                    DEBUG_PRINTLN(s);
                }
            }
        }
        break;
    }

    return _mqttState;

}

