//#include "shared.h"
#include "secrets.h"

// wire slave 
#include "wireSlave_messages.h"
#include "wireSlave_transport.h"

// WiFi and MQTT
#include "MQTT_messages.h"

// debugging 
#include "debug.h"


SharedContext sharedContext;
MQTTmessages mqttMessages(sharedContext);
WireSlaveMessages wireSlaveMessages(sharedContext);


// ============================================================================
// SETUP
// ============================================================================
void setup()
{
    Serial.begin(115200);
    delay(5000);

    Serial.println("=== NEW BUILD ===");
    Serial.println(__DATE__);
    Serial.println(__TIME__);

    pinMode(13, OUTPUT);

}

// ============================================================================
// LOOP
// ============================================================================
void loop()
{
    mqttMessages.loop();
    wireSlaveMessages.loop();
}



