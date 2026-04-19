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


#include <time.h>
#include "WiFiConnection.h"
#include "time_helpers.h"

// ============================================================================
// WiFi MAINTENANCE: state machine (call regularly from main loop() )
// ============================================================================
WiFiConnection::ConnectionState WiFiConnection::maintainWiFi() {


    uint32_t now = millis();

    switch (_wifiState) {

        // state: WiFi is currently not connected
        // --------------------------------------
        case WiFi_notConnected:
        {
            // time for a next WiFi connection attempt ?
            if (now - _lastWiFiMaintenanceTime > WIFI_UP_CHECK_INTERVAL) {

                // If DHCP reservation (or 'static lease') instead of static IP address: IP for specific nano esp32 MAC address is set in the router.
                // The nano esp32 still goes through the normal DHCP process — it asks the router "give me an IP" — 
                // but the router always hands out the same one based on the MAC address. From the ESP32's perspective it's just using DHCP normally.
                // Static IP breaks silently when the network assumptions don't match (different gateway, subnet, DNS, or an IP conflict).
                // Also, the ESP32's network stack appears to be more sensitive to static IP mismatches than the Nano 33 IoT's NINA chip

                // WiFi.config(...) is only needed if using a static IP address (not if DHCP, whether you use DHCP reservation ('static lease') or not).
                // => Outcomment next line if using DHCP (with or without a static lease) instead of assigning a static IP.
                WiFi.config(clientAddress, gatewayAddress, subnetMask, DNSaddress);
                WiFi.begin(WIFI_SSID, WIFI_PASS);
                _wifiState = WiFi_waitForConnecton;

                if (DEBUG) {
                    char timeBuf[32]; if (!timeHelpers::getLocalTimeString(timeBuf, sizeof(timeBuf))) {
                        snprintf(timeBuf, sizeof(timeBuf), "at %13.3fs", now / 1000.);
                    }
                    char s[80]; snprintf(s, sizeof(s), "-- %s : Trying to connect to WiFi. MAC is %s...", timeBuf, WiFi.macAddress().c_str());
                    DEBUG_PRINTLN(s);
                }

                // remember time of this WiFi connection attempt; this is also the time of the last debug print (if enabled)
                _lastWiFiWaitReportedAt = _lastWiFiMaintenanceTime = now;
            }
        }
        break;


        // state: currently waiting for WiFi connection
        // --------------------------------------------
        case WiFi_waitForConnecton:
        {
            // WiFi is enabled AND it's time for a next WiFi connection check ?
            if (now - _lastWiFiMaintenanceTime > WIFI_UP_CHECK_INTERVAL) {
                if (WiFi.status() == WL_CONNECTED) {                                    // WiFi is now connected ?
                    _wifiState = WiFi_connected;

                    // CET: Central European Time; -1: 1 hour east of ETC (UTC+1); CEST: Central European Summer Time (Automatic daylight saving)
                    // CEST = CET+1 = UTC+2
                    // last Sunday of March, at 2:00 AM clocks go forward to 3:00 AM; last Sunday of October, at 3:00 AM clocks go back to 2:00 AM
                    configTzTime("CET-1CEST,M3.5.0/2,M10.5.0/3", "pool.ntp.org");

                    if (DEBUG) {
                        char timeBuf[32]; if (!timeHelpers::getLocalTimeString(timeBuf, sizeof(timeBuf))) {
                            snprintf(timeBuf, sizeof(timeBuf), "at %13.3fs", now / 1000.);
                        }
                        IPAddress IP = WiFi.localIP();
                        char s[80]; snprintf(s, sizeof(s), "-- %s : WiFi connected, Local IP %d.%d.%d.%d(%d dBm)",
                            timeBuf, IP[0], IP[1], IP[2], IP[3], WiFi.RSSI());
                        DEBUG_PRINTLN(s);
                    }
                }

                else {                                                                  // WiFi is not yet connected
                    // regularly report status ('still trying...' etc.)
                    if (DEBUG) {
                        if (now - _lastWiFiWaitReportedAt > WIFI_REPORT_INTERVAL) {
                            _lastWiFiWaitReportedAt = now;                              // for printing only
                            DEBUG_PRINT(".");
                        }
                    }
                }
                _lastWiFiMaintenanceTime = now;                                         // remember time of last WiFi maintenance 
            }
        }
        break;


        // state: WiFi is connected
        // ------------------------
        case WiFi_connected:
        {
            //  prepare for reconnection if connection is lost OR per user program request 
            if (WiFi.status() != WL_CONNECTED) {
                _wifiState = WiFi_notConnected;

                if (DEBUG) {
                    char timeBuf[32]; if (!timeHelpers::getLocalTimeString(timeBuf, sizeof(timeBuf))) {
                        snprintf(timeBuf, sizeof(timeBuf), "at %13.3fs", now / 1000.);
                    }
                    char s[80]; snprintf(s, sizeof(s), "-- %s : WiFi disconnected", timeBuf);
                    DEBUG_PRINTLN(s);
                }
                WiFi.disconnect();
            #if !defined ARDUINO_ARCH_ESP32
                WiFi.end();
            #endif
                _lastWiFiMaintenanceTime = now;                                         // remember time of last WiFi maintenance 
            }
        }
        break;
    }

    return (_wifiState);
}


