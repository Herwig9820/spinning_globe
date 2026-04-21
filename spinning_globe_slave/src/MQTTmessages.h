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


#ifndef FG_MQTT_MESSAGES_h
#define FG_MQTT_MESSAGES_h

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "bridge_context.h"
#include "WiFiConnection.h"

class MQTTmessages {

    const char* const ROOT_CA = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";

    static constexpr size_t MQTT_CLIENT_BUF_SIZE = 256; ////1024;        // passed to _MQTTclient.setBufferSize();
    
    static constexpr uint32_t MQTT_UP_CHECK_INTERVAL = 2000;
    static constexpr uint32_t MQTT_REPORT_INTERVAL = 4000;
    static constexpr uint32_t MQTT_PUBLISH_TIMEOUT = 5UL * 60 * 1000; // 5 min
    static constexpr uint8_t MQTT_MAX_FAIL_COUNT = 6;           // before also disconnecting WiFi and starting all over

public:
    enum ConnectionState {
        MQTT_notConnected,                                       // MQTT not yet connected
        MQTT_waitForConnecton,                                   // waiting for MQTT to connect
        MQTT_connected                                           // MQTT connected
    };

private:
    bool _MQTTnewMessageFlag{false};
    uint8_t _mqttFailCount{ 0 };

    ConnectionState _mqttState{ MQTT_notConnected };

    uint32_t _lastMqttMaintenanceTime{ millis() };
    uint32_t _lastMqttWaitReportedAt{ _lastMqttMaintenanceTime };

    SharedContext& _sharedContext;

    WiFiConnection * _pWiFiConnection;
    WiFiClientSecure _tlsSocket;
    PubSubClient     _MQTTclient;

    
    // ========== methods ==========
    ConnectionState maintainMQTT(bool WiFiConnected);

    // --- Callback plumbing ---
    static MQTTmessages* _instance;
    static void mqttCallback(char* topic, byte* payload, unsigned int length);

    // --- Real handler ---
    void pushIncomingMQTTmsg(char* topic, byte* payload, unsigned int length);

public:

    MQTTmessages(SharedContext& sharedContext);
    ConnectionState loop(bool WiFiConnected);
    
    // get flag and clear
    bool inline newMQTTmessage(){bool flag = _MQTTnewMessageFlag; _MQTTnewMessageFlag = false; return flag;}

    bool convertMQTTtoGlobeSettings(MQTTmsgToWire* pMsgToWire);
    bool convertMQTTtoPIDsettings(MQTTmsgToWire* pMsgToWire);
    bool convertMQTTtoVertPosSetpoint(MQTTmsgToWire* pMsgToWire);
    bool convertMQTTtoCoilPhaseAdjust(MQTTmsgToWire* pMsgToWire);
    void holdRequestForWireMaster(MsgType m_msgtype, Action m_action = M_ACTION_NONE);
    void holdRequestForWireMaster(Action m_action);

};

#endif

