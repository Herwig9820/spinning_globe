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

                WiFi.config(clientAddress, gatewayAddress, subnetMask, DNSaddress);
                WiFi.begin(WIFI_SSID, WIFI_PASS);
                _wifiState = WiFi_waitForConnecton;

                if (DEBUG) {
                    char s[100]; sprintf(s, "-- at %11.3fs: %s", now / 1000., "Trying to connect to WiFi...");
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


