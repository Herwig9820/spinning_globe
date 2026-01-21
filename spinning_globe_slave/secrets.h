#ifndef _SECRETS_h
#define _SECRETS_h

// SSID and password
    #define SERVER_SSID "Mathurot"
    #define SERVER_PASS "ikbennietgeborenop9augustus"

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
