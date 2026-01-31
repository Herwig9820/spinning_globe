#ifndef _SECRETS_h
#define _SECRETS_h

    // WiFi: SSID and password
    #define WIFI_SSID "Mathurot"
    #define WIFI_PASS "ikbennietgeborenop9augustus"

    // MQTT
    #define MQTT_SERVER "e303fcc2b2f344838220e08b26e702b3.s1.eu.hivemq.cloud"
    #define MQTT_PORT 8883
    
    #define MQTT_USER "spinning_globe"
    #define MQTT_PASS "*8*twtttj^AG@R79"

    #define MQTT_DEVICE_ID "floating_globe_esp32_B59J4NF4865L"

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
    ( WAN IP adres: 213.119.105.5 - kan wijzigen) 
    // note: enable router port forwarding if server will be contacted from outside LAN 

    const int serverPort = 8085;
    const IPAddress serverAddress(192, 168, 0, 95);     // STATIC server IP (LAN)
    
    const IPAddress gatewayAddress(192, 168, 0, 1);
    const IPAddress subnetMask(255, 255, 255, 0);
    const IPAddress DNSaddress(195, 130, 130, 5);

    AANPASSEN in .ino file : IP adressen voor VAISON
    -----------------------------------------------
    ( WAN IP adres: 31.39.207.74:8085 - kan wijzigen )
    // note: enable router port forwarding if server will be contacted from outside LAN 
   
    const int serverPort = 8085;
    const IPAddress serverAddress(192, 168, 1, 45);     // STATIC server IP (LAN)
    
    const IPAddress gatewayAddress(192, 168, 1, 254);
    const IPAddress subnetMask(255, 255, 255, 0);
    const IPAddress DNSaddress(195, 130, 130, 5);
*/

#endif
