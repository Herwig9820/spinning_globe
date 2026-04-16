#ifndef FG_SECRETS_h
#define FG_SECRETS_h

    // WiFi: SSID and password
    #define WIFI_SSID "Mathurot"
    #define WIFI_PASS "ikbennietgeborenop9augustus"

    // MQTT
    #define MQTT_SERVER "e303fcc2b2f344838220e08b26e702b3.s1.eu.hivemq.cloud"
    #define MQTT_PORT 8883
    
    #define MQTT_USER "spinning_globe"
    #define MQTT_PASS "*8*twtttj^AG@R79"

    constexpr const char* SECRET_TOKEN = "gl0be_L0cal_9x";      // purpose: restrict access to specific settings if node-red dashboard not on esp32 local network


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

    Bij telenet is het zo dat alle ip adressen beneden de 100 niet in de dhcp zitten.
    De toestellen die je zelf een ip adres wil toewijzen moeten dus tussen tussen 192.168.0.2 en 192.168.0.99 zitten.
    
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
*/

#endif
