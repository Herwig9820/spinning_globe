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


#ifndef FG_SECRETS_h
#define FG_SECRETS_h

#define MQTT_BROKER_HIVEMQ 0                                // if 0: Mosquitto (local on Pi), 1: HiveMQ serverless (cloud)

// secrets.h
#define WIFI_SSID_HOME1  "homeNet1"
#define WIFI_PASS_HOME1  "appel-telefoon5XR-blijdschap#&-verloren-onderneming"
#define WIFI_SSID_HOME2  "homeNet2"
#define WIFI_PASS_HOME2  "appel-telefoon5XR-blijdschap#&-verloren-onderneming"

#if MQTT_BROKER_HIVEMQ 
#define MQTT_SERVER_HOME1 "e303fcc2b2f344838220e08b26e702b3.s1.eu.hivemq.cloud"       // hiveMQ
#define MQTT_SERVER_HOME2 MQTT_SERVER_HOME1 
#define MQTT_PORT_HOME1 8883
#define MQTT_PORT_HOME2 8883

#else
#define MQTT_SERVER_HOME1 "192.168.0.120"                   // Pi 5 Merelbeke: static IP (Pi 5: set with 'nmcli'sudo nmcli connection modify ...' command)
#define MQTT_SERVER_HOME2 "192.168.1.120"                   // Pi 5 Vaison: static DHCP lease for Pi MAC address (router settings)
#define MQTT_PORT_HOME1 1883
#define MQTT_PORT_HOME2 1883
#endif

#define MQTT_USER "spinning_globe"
#define MQTT_PASS "*8*twtttj^AG@R79"

constexpr const char* SECRET_TOKEN = "gl0be_L0cal_9x";      // purpose: restrict access to specific settings if node-red dashboard not on esp32 local network


struct LocationConfig {
    const char* ssid;
    const char* wifiPass;
    const char* mqttServer;
    uint16_t    mqttPort;
};

const LocationConfig knownLocations[] = {
    { WIFI_SSID_HOME1, WIFI_PASS_HOME1, MQTT_SERVER_HOME1, MQTT_PORT_HOME1 },   // MQTT server Merelbeke: static IP (telenet router: static DHCP lease not available)
    { WIFI_SSID_HOME2, WIFI_PASS_HOME2, MQTT_SERVER_HOME2, MQTT_PORT_HOME2 },   // MQTT server Vaison   : static DHCP lease (raspberry Pi MAC = 88:a2:9e:d6:a1:f6 )
};
const size_t knownLocationsCount = sizeof(knownLocations) / sizeof(knownLocations[0]);






/*
    AANPASSEN in deze secrets.h file : WiFi SSID en paswoord voor MERELBEKE
    -----------------------------------------------------------------------
    #define SERVER_SSID "Mathurot"
    #define SERVER_PASS "ikbennietgeborenop9augustus"

    AANPASSEN in deze secrets.h file : WiFi SSID en paswoord voor VAISON
    --------------------------------------------------------------------
    #define SERVER_SSID "Bbox-HTA"
    #define SERVER_PASS "ibngo9Augibngo9Aug"




    AANPASSEN in .ino file : IP adressen voor MERELBEKE
    --------------------------------------------------
    ( note: WAN IP address: 213.119.105.5 - can change over time !)
    NOTE: enable router port forwarding if server will be contacted from outside LAN

    telenet: static IP range is 192.168.0.2 to 192.168.0.99

    const IPAddress clientAddress(192, 168, 0, 95);     // STATIC server IP (LAN)

    const IPAddress gatewayAddress(192, 168, 0, 1);
    const IPAddress subnetMask(255, 255, 255, 0);
    const IPAddress DNSaddress(195, 130, 130, 5);


    AANPASSEN in .ino file : IP adressen voor VAISON
    -----------------------------------------------
    ( note: WAN IP address: 31.39.207.74 - can change over time !)

    DHCP reservation (or 'static lease') instead of static IP address: IP for specific nano esp32 MAC address is set in the router.
    The nano esp32 still goes through the normal DHCP process — it asks the router "give me an IP" —
    but the router always hands out the same one based on the MAC address. From the ESP32's perspective it's just using DHCP normally.
    Static IP breaks silently when the network assumptions don't match (different gateway, subnet, DNS, or an IP conflict).
    Also, the ESP32's network stack appears to be more sensitive to static IP mismatches than the Nano 33 IoT's NINA chip

    NOTE: the statement: "WiFi.config(clientAddress, gatewayAddress, subnetMask, DNSaddress);" MUST BE OUTCOMMENTED
    (just before the WiFi.begin(...) statement)


    Bouyges: DHCP range is ??? //// (dhcp reservation / static lease)
*/

#endif
