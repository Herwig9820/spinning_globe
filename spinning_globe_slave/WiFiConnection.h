#ifndef FG_WIFI_CONNECTION_h
#define FG_WIFI_CONNECTION_h

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "secrets.h"
#include "debug.h"

const IPAddress clientAddress(192, 168, 0, 95);     // STATIC client IP (LAN)
const IPAddress gatewayAddress(192, 168, 0, 1);
const IPAddress subnetMask(255, 255, 255, 0);
const IPAddress DNSaddress(195, 130, 130, 5);

class WiFiConnection {

    static constexpr uint32_t WIFI_UP_CHECK_INTERVAL = 3000;
    static constexpr uint32_t WIFI_REPORT_INTERVAL = 4000;

    enum ConnectionState {
        WiFi_notConnected,                                       // WiFi not yet connected
        WiFi_waitForConnecton,                                   // waiting for WiFi to connect
        WiFi_connected,                                          // WiFi connected
    };

    ConnectionState _wifiState{WiFi_notConnected};

    uint32_t _lastWiFiMaintenanceTime{millis()};
    uint32_t _lastWiFiWaitReportedAt{ _lastWiFiMaintenanceTime };
    
public:
    void loop();
    bool maintainWiFi();
};

#endif

