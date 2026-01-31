#include "WiFiConnection.h"
#include "secrets.h"

// wire slave 
#include "wireSlave_messages.h"
#include "wireSlave_transport.h"

// WiFi and MQTT
#include "MQTTmessages.h"

// debugging 
#include "debug.h"

SharedContext sharedContext;
WireSlaveMessages wireSlaveMessages(sharedContext);
//MQTTmessages mqttMessages(sharedContext);
MQTTmessages* mqttMessages = nullptr;



// ============================================================================
// SETUP
// ============================================================================
void setup()
{
    Serial.begin(115200);
    delay(5000);

    Serial.println("=== BUILD DATE AND TIME ===");
    Serial.println(__DATE__);
    Serial.println(__TIME__);

    mqttMessages = new MQTTmessages(sharedContext);

    pinMode(13, OUTPUT);

}

// ============================================================================
// LOOP
// ============================================================================
void loop()
{
    if (mqttMessages)
        mqttMessages->loop();

    wireSlaveMessages.loop();
}



