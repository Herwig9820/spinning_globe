#include "MQTTmessages.h"
#include "secrets.h"
#include "debug.h"
#include "json_helpers.h"
#include "time_helpers.h"

MQTTmessages* MQTTmessages::_instance = nullptr;

MQTTmessages::MQTTmessages(SharedContext& sharedContext) :
    _MQTTclient(_tlsSocket), _sharedContext(sharedContext) {
    
    _instance = this;

    _tlsSocket.setCACert(ROOT_CA);                   // Set the Root CA for the secure client
    _MQTTclient.setServer(MQTT_SERVER, MQTT_PORT);
    _MQTTclient.setCallback(mqttCallback);
    _MQTTclient.setKeepAlive(60);     // seconds; HiveMQ free tier disconnects after this idle time
}


MQTTmessages::ConnectionState MQTTmessages::loop(bool WiFiConnected) {
    // first, maintain WiFi and MQTT connections
    maintainMQTT(WiFiConnected);

    if (_mqttState == MQTT_connected) {

        // ---------------------------
        // data available to publish ?
        // ---------------------------
        if (!_sharedContext.queueToMQTT.empty()) {
            MQTTmsgFromWire* pMsgToMQTT{};
            _MQTTnewMessageFlag = true;                                                     // cleared by inline .newMQTTmessage() method
            pMsgToMQTT = _sharedContext.queueToMQTT.front();
            // .publish(...) builds an MQTT PUBLISH packet, writes it into the underlying TCP client, returns immediately(success or failure)
            _MQTTclient.publish(pMsgToMQTT->topic, pMsgToMQTT->payload, pMsgToMQTT->retain);
            _sharedContext.lastMQTTpublish = millis();
            _sharedContext.queueToMQTT.pop(*pMsgToMQTT);
        }

        // -----------------------------------------------
        // topics available the nano esp32 subscribed to ? 
        // -----------------------------------------------
        if (!_sharedContext.queueToWire.empty()) {
            MQTTmsgToWire* pMsgToWire{};
            _MQTTnewMessageFlag = true;
            pMsgToWire = _sharedContext.queueToWire.front();

            // ---------- message (settings, ...) available for wire master, NOT fitting in a wire slave 'ack' response ----------

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


            // ---------- message (request for action, or data, from master) available for wire master, fitting in a wire slave 'ack' response ----------

            // requests for wire master action:
            else if (strcmp(pMsgToWire->topic, TOPIC_RING_REQUEST) == 0) {                      // request wire master RING action   
                holdRequestForWireMasterAction(Action::M_ACTION_RING);
            }

            // requests for wire master message types:       
            else if (strcmp(pMsgToWire->topic, TOPIC_GLOBE_SETTINGS_REQUEST) == 0) {        // request wire master globe settings
                holdRequestForWireMasterMsgType(MsgType::M_MSG_GLOBE_SETTINGS);
            }
            else if (strcmp(pMsgToWire->topic, TOPIC_PID_SETTINGS_REQUEST) == 0) {        // request wire master globe settings
                holdRequestForWireMasterMsgType(MsgType::M_MSG_PID_SETTINGS);
            }
            else if (strcmp(pMsgToWire->topic, TOPIC_VERT_POS_SETPOINT_REQUEST) == 0) {        // request wire master globe settings
                holdRequestForWireMasterMsgType(MsgType::M_MSG_VERT_POS_SETPOINT);
            }
            else if (strcmp(pMsgToWire->topic, TOPIC_COIL_PHASE_ADJUST_REQUEST) == 0) {        // request wire master globe settings
                holdRequestForWireMasterMsgType(MsgType::M_MSG_COIL_PHASE_ADJUST);
            }

            // remove entry in MQTT in queue
            _sharedContext.queueToWire.pop(*pMsgToWire);
        }
    }

    // handles MQTT keep-alive, incoming MQTT packets; detects broken connections, calls the MQTT user callback  
    _MQTTclient.loop(); // placed at the end, after all states machines ran, after an eventual publish that was just queued  

    return _mqttState;
}

// ============================================================================================
// process MQTT message to wire master - STEP 1: parse MQTT payload into 'pending' globe settings structure
// [STEP 2: in wireSlaveMessages.loop()]
// ============================================================================================

bool MQTTmessages::convertMQTTtoGlobeSettings(MQTTmsgToWire* pMsgToWire) {
    bool ok{ false };
    unsigned int tmp{};

    ok = JsonParse::getUInt(pMsgToWire->payload, PL_KEY_SET_ROT_TIME, &tmp);
    _sharedContext.pendingGlobeSettings.rotationPeriodIndex = tmp;
    ok &= JsonParse::getUInt(pMsgToWire->payload, PL_KEY_SET_LED_EFFECT, &tmp);
    _sharedContext.pendingGlobeSettings.ledEffect = tmp;
    ok &= JsonParse::getUInt(pMsgToWire->payload, PL_KEY_SET_LED_EFFECT_SPEED, &tmp);
    _sharedContext.pendingGlobeSettings.ledCycleSpeed = tmp;
    _sharedContext.pendingGlobeSettings.slaveHasData = (uint8_t)ok;       // overwrite previous if not committed in time
    return ok;
}

bool MQTTmessages::convertMQTTtoPIDsettings(MQTTmsgToWire* pMsgToWire) {

    // Token check: only accept PID settings from local network
    char token[32] = "";
    JsonParse::getString(pMsgToWire->payload, PAYLOAD_SECRET_TOKEN, token, sizeof(token));
    if (strcmp(token, SECRET_TOKEN) != 0) { return false; }  // wrong or missing token: silently ignore

    bool ok{ false };
    unsigned int tmp{};

    ok = JsonParse::getUInt(pMsgToWire->payload, PL_KEY_SET_GAIN_ADJUST, &tmp);
    _sharedContext.pendingPIDsettings.gainAdjustSteps = tmp;
    ok &= JsonParse::getUInt(pMsgToWire->payload, PL_KEY_SET_DIFF_TIME_ADJUST, &tmp);
    _sharedContext.pendingPIDsettings.difTimeCstAdjustSteps = tmp;
    ok &= JsonParse::getUInt(pMsgToWire->payload, PL_KEY_SET_INTEGR_TIME_ADJUST, &tmp);
    _sharedContext.pendingPIDsettings.intTimeCstAdjustSteps = tmp;
    _sharedContext.pendingPIDsettings.slaveHasData = (uint8_t)ok;       // overwrite previous if not committed in time
    return ok;
}

bool MQTTmessages::convertMQTTtoVertPosSetpoint(MQTTmsgToWire* pMsgToWire) {
    // Token check: only accept vertical position setpoint from local network
    char token[32] = "";
    JsonParse::getString(pMsgToWire->payload, PAYLOAD_SECRET_TOKEN, token, sizeof(token));
    if (strcmp(token, SECRET_TOKEN) != 0) { return false; }   // wrong or missing token: silently ignore

    bool ok{ false };
    unsigned int tmp{};

    ok = JsonParse::getUInt(pMsgToWire->payload, PL_KEY_SET_VERT_POS_SETPOINT, &tmp);
    _sharedContext.pendingVertPosSetpoint.vertPosIndex = tmp;
    _sharedContext.pendingVertPosSetpoint.slaveHasData = (uint8_t)ok;       // overwrite previous if not committed in time
    return ok;
}

bool MQTTmessages::convertMQTTtoCoilPhaseAdjust(MQTTmsgToWire* pMsgToWire) {
    // Token check: only accept coil phase adjustment from local network
    char token[32] = "";
    JsonParse::getString(pMsgToWire->payload, PAYLOAD_SECRET_TOKEN, token, sizeof(token));
    if (strcmp(token, SECRET_TOKEN) != 0) { return false; }   // wrong or missing token: silently ignore

    bool ok{ false };
    unsigned int tmp{};

    ok = JsonParse::getUInt(pMsgToWire->payload, PL_KEY_SET_COIL_PHASE_ADJUST, &tmp);
    _sharedContext.pendingCoilPhaseAdjust.coilPhaseAdjust = tmp;
    _sharedContext.pendingCoilPhaseAdjust.slaveHasData = (uint8_t)ok;       // overwrite previous if not committed in time
    return ok;
}

void MQTTmessages::holdRequestForWireMasterMsgType(MsgType m_msgType) {
    AckPayload ackResponse{};
    ackResponse.msgType = m_msgType;
    ackResponse.action = Action::M_ACTION_NONE;
    _sharedContext.holdAckResponses.push(ackResponse);
}

void MQTTmessages::holdRequestForWireMasterAction(Action m_action) {
    AckPayload ackResponse{};
    ackResponse.msgType = MsgType::M_MSG_NONE;
    ackResponse.action = m_action;
    _sharedContext.holdAckResponses.push(ackResponse);

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
                _tlsSocket.stop();  // force-close any lingering socket before new attempt                
                char clientId[48];
                snprintf(clientId, sizeof(clientId), "spinning-globe-esp32-%s", WiFi.macAddress().c_str());
                _MQTTclient.connect(clientId, MQTT_USER, MQTT_PASS, TOPIC_STATUS, 0, true, "99", true);       // last will: status "99" means 'offline'
                _mqttState = MQTT_waitForConnecton;

                if (DEBUG) {
                    char timeBuf[32]; if (!timeHelpers::getLocalTimeString(timeBuf, sizeof(timeBuf))) {
                        snprintf(timeBuf, sizeof(timeBuf), "at %13.3fs", now / 1000.);
                    } 
                    char s[80]; snprintf(s, sizeof(s), "-- %s : Trying to connect to MQTT...", timeBuf);
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
                if (_MQTTclient.connected()) {                                        // MQTT is now connected ?
                    _mqttFailCount = 0;
                    _mqttState = MQTT_connected;

                    _MQTTclient.subscribe(TOPIC_GLOBE_SETTINGS_SET);                // MQTT publishing settings etc. 
                    _MQTTclient.subscribe(TOPIC_PID_SETTINGS_SET);
                    _MQTTclient.subscribe(TOPIC_VERT_POS_SETPOINT_SET);
                    _MQTTclient.subscribe(TOPIC_COIL_PHASE_ADJUST_SET);

                    _MQTTclient.subscribe(TOPIC_GLOBE_SETTINGS_REQUEST);            // MQTT publishing a request for spinning globe to send sends out settings
                    _MQTTclient.subscribe(TOPIC_PID_SETTINGS_REQUEST);
                    _MQTTclient.subscribe(TOPIC_VERT_POS_SETPOINT_REQUEST);
                    _MQTTclient.subscribe(TOPIC_COIL_PHASE_ADJUST_REQUEST);

                    _MQTTclient.subscribe(TOPIC_RING_REQUEST);                      // MQTT requesting that nano esp32 performs an action        

                    if (DEBUG) {
                        char timeBuf[32]; if (!timeHelpers::getLocalTimeString(timeBuf, sizeof(timeBuf))) {
                            snprintf(timeBuf, sizeof(timeBuf), "at %13.3fs", now / 1000.);
                        }
                        char s[80]; snprintf(s, sizeof(s), "-- %s : MQTT connected", timeBuf);
                        DEBUG_PRINTLN(s);
                    }
                }

                else {                                                                      // MQTT is not yet connected
                    // after a number of MQTT connection attempts, also disconnect WiFi and start all over
                    if (++_mqttFailCount >= MQTT_MAX_FAIL_COUNT) {
                        _mqttFailCount = 0;
                        _mqttState = MQTT_notConnected;
                        // WiFi state machine will detect WL_DISCONNECTED  and trigger a full reconnect including fresh TLS stack
                        WiFi.disconnect();
                    }

                    if (DEBUG) {                                                        // regularly report status ('still trying...' etc.)
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
            bool connectionLost = !_MQTTclient.connected();
            bool publishStalled = _sharedContext.lastMQTTpublish > 0 &&
                (millis() - _sharedContext.lastMQTTpublish > MQTT_PUBLISH_TIMEOUT);

            if (connectionLost or publishStalled) {
                _MQTTclient.disconnect();
                _mqttState = MQTT_notConnected;
                _MQTTnewMessageFlag = false;
                _lastMqttMaintenanceTime = now;                                        // remember time of last MQTT maintenance 
                if (DEBUG) {
                    char timeBuf[32]; if (!timeHelpers::getLocalTimeString(timeBuf, sizeof(timeBuf))) {
                        snprintf(timeBuf, sizeof(timeBuf), "at %13.3fs", now / 1000.);
                    }
                    char s[80]; snprintf(s, sizeof(s), "-- %s : MQTT disconnected, client state = %d",timeBuf,  _MQTTclient.state());
                    DEBUG_PRINTLN(s);
                }
            }
        }
        break;
    }

    return _mqttState;

}

