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


#ifndef FG_WIFI_CONNECTION_h
#define FG_WIFI_CONNECTION_h

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <esp_sntp.h> // needed for sntp_get_sync_status()
#include "../secrets.h"
#include "bridge_context.h"
#include "debug.h"

// Note that next 4 definitions will only be used if using a static IP address.
// If using DHCP instead (with or without a static lease), outcomment line 
// "WiFi.config(clientAddress, gatewayAddress, subnetMask, DNSaddress)"  in WiFiConnection.cpp 
const IPAddress clientAddress(192, 168, 0, 95);     // STATIC client IP (LAN)
const IPAddress gatewayAddress(192, 168, 0, 1);
const IPAddress subnetMask(255, 255, 255, 0);
const IPAddress DNSaddress(195, 130, 130, 5);

class WiFiConnection {

    static constexpr uint32_t WIFI_UP_CHECK_INTERVAL = 3000;
    static constexpr uint32_t WIFI_REPORT_INTERVAL = 4000;

public:
    enum ConnectionState {
        WiFi_notConnected,                                       // WiFi not yet connected
        WiFi_waitForConnecton,                                   // waiting for WiFi to connect
        WiFi_connected,                                          // WiFi connected
    };

private:
    ConnectionState _wifiState{WiFi_notConnected};

    uint32_t _lastWiFiMaintenanceTime{millis()};
    uint32_t _lastWiFiWaitReportedAt{ _lastWiFiMaintenanceTime };
    
public:
    void loop();
    ConnectionState maintainWiFi();
};

#endif

