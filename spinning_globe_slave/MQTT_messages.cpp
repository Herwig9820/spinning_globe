#include "MQTT_messages.h"
#include "secrets.h"
#include "debug.h"

MQTTmessages* MQTTmessages::_instance = nullptr;

MQTTmessages::MQTTmessages(SharedContext& sharedContext):_client(_espClient), _sharedContext(sharedContext){

    // WiFi
    Serial.print(F("Connecting to WiFi ")); delay(1000);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(300);
    }
    DEBUG_PRINTLN(F("\nWiFi connected."));
    Serial.print(F("IP = "));
    Serial.println(WiFi.localIP());

    _instance = this;

    _espClient.setCACert(ROOT_CA);                   // Set the Root CA for the secure client
    _client.setServer(MQTT_SERVER, MQTT_PORT);
    _client.setCallback(mqttCallback);
    reconnectMQTT();
}

void MQTTmessages::loop(){
    if(!_client.connected()){reconnectMQTT(); }

    
    _client.loop();
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
void MQTTmessages::reconnectMQTT() {

    const char* clientId = "ESP32Client1";  // Your unique client ID

    Serial.print("Connecting to MQTT...");
    while (!_client.connected()) {
       // Use the connect method with clientId, username, and password
        if (_client.connect(clientId, MQTT_USER, MQTT_PASS)) {
            Serial.println("connected.");
            // Subscribe or publish here if needed
            _client.subscribe("globe/led");
            
        }
        else {
            Serial.print("failed, rc=");
            Serial.print(_client.state());
            Serial.println(" retrying in 2 seconds");
            delay(2000);
        }
    }
}

