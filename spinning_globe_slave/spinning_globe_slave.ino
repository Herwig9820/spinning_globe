#include <esp_task_wdt.h>
#include "slave_messages.h"
#include "MQTTmessages.h"

constexpr uint32_t WDT_TIMEOUT = 1;

SharedContext sharedContext;
WireSlaveMessages wireSlaveMessages(sharedContext);

MQTTmessages* pMqttMessages = nullptr;



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
    pMqttMessages = new MQTTmessages(sharedContext);
    pinMode(WIRE_RECEIVE_LED, OUTPUT);

    // Initialize Task Watchdog Timer (enable, 5s timeout) and add current task (loop) to WDT
    esp_task_wdt_init(WDT_TIMEOUT, true);
    esp_task_wdt_add(NULL);
}

// ============================================================================
// LOOP
// ============================================================================
void loop()
{
    // that's all there is (here): this nano esp32 acts as a bridge between MQTT and the spinning globe classic nano
    
    // maintain both the wire slave and MQTT connections and send messages back and forth
    if (pMqttMessages){pMqttMessages->loop();}
    wireSlaveMessages.loop();

    esp_task_wdt_reset();           // reset the watchdog
}



