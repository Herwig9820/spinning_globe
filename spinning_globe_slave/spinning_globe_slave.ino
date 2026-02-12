#include "wireSlave_messages.h"
#include "MQTTmessages.h"

SharedContext sharedContext;
WireSlaveMessages wireSlaveMessages(sharedContext);
MQTTmessages* mqttMessages = nullptr;



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
    mqttMessages = new MQTTmessages(sharedContext);

    pinMode(13, OUTPUT);

}

// ============================================================================
// LOOP
// ============================================================================
void loop()
{
    // that's all there is (here): this nano esp32 acts as a bridge between MQTT and the spinning globe classic nano
    
    // maintain both the wire slave and MQTT connections and send messages back and forth
    if (mqttMessages){mqttMessages->loop();}
    wireSlaveMessages.loop();
}



