#include "WiFiConnection.h"


WiFiConnection::ConnectionState WiFiConnection::maintainWiFi() {

    // Variable '_wifiState' maintains the state of the WiFi connection('state machine').If this maintained state
    // (e.g., 'WiFi connected') does not correspond to the actual state(e.g., WiFi connection was lost), action is taken

    uint32_t now = millis();

    switch (_wifiState) {

        // state: WiFi is currently not connected
        // --------------------------------------
        case WiFi_notConnected:
        {
            // time for a next WiFi connection attempt ?
            if (now - _lastWiFiMaintenanceTime > WIFI_UP_CHECK_INTERVAL) {

                // DHCP reservation (or 'static lease') instead of static IP address: IP for specific nano esp32 MAC address is set in the router.
                // The nano esp32 still goes through the normal DHCP process — it asks the router "give me an IP" — 
                // but the router always hands out the same one based on the MAC address. From the ESP32's perspective it's just using DHCP normally.
                // Static IP breaks silently when the network assumptions don't match (different gateway, subnet, DNS, or an IP conflict).
                // Also, the ESP32's network stack appears to be more sensitive to static IP mismatches than the Nano 33 IoT's NINA chip
                
                //WiFi.config(clientAddress, gatewayAddress, subnetMask, DNSaddress);
                WiFi.begin(WIFI_SSID, WIFI_PASS);
                _wifiState = WiFi_waitForConnecton;

                if (DEBUG) {
                    char s[100]; sprintf(s, "-- at %11.3fs: %s", now / 1000., "Trying to connect to WiFi...");
                    DEBUG_PRINTLN(s);
                    DEBUG_PRINTLN(WiFi.macAddress().c_str()); ////
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
                if (WiFi.status() == WL_CONNECTED) {                                        // WiFi is now connected ?
                    _wifiState = WiFi_connected;

                    if (DEBUG) {
                        IPAddress IP = WiFi.localIP();
                        char s[120]; sprintf(s, "-- at %11.3fs: WiFi connected, Local IP %d.%d.%d.%d (%ld dBm)",
                            now / 1000., IP[0], IP[1], IP[2], IP[3], WiFi.RSSI());
                        DEBUG_PRINTLN(s);
                    }
                }

                else {                                                                      // WiFi is not yet connected
                    // regularly report status ('still trying...' etc.)
                    if (DEBUG) {
                        if (now - _lastWiFiWaitReportedAt > WIFI_REPORT_INTERVAL) {
                            _lastWiFiWaitReportedAt = now;                              // for printing only
                            DEBUG_PRINT(".");
                        }
                    }
                }
                _lastWiFiMaintenanceTime = now;                                        // remember time of last WiFi maintenance 
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
                    char s[100]; sprintf(s, "-- at %11.3fs: %s", now / 1000., "WiFi disconnected");
                    DEBUG_PRINTLN(s);
                }
                WiFi.disconnect();
            #if !defined ARDUINO_ARCH_ESP32
                WiFi.end();
            #endif
                _lastWiFiMaintenanceTime = now;                                        // remember time of last WiFi maintenance 
            }
        }
        break;
    }

    return (_wifiState);
}


