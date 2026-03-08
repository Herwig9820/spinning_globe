/*
==================================================================================================
Floating and spinning earth globe
---------------------------------
Copyright 2019, 2026 Herwig Taveirne

Program written and tested for classic (8-bit) Arduino Nano.

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

A complete description of this project can be found here:
https://www.instructables.com/Floating-and-Spinning-Earth-Globe/

===============================================================================================
Spinning globe extension: using the Wire interface to exchange messages with an Arduino nano esp32.
---------------------------------------------------------------------------------------------------
An Arduino nano esp32, acting as wire slave, will control the spinning globe (change settings, check states)
over WiFi, e.g. using MQTT.

Note that, if the program is compiled with this option enabled, hardware buttons and LCD (connector SV2)...
...will be inoperable (switches are still functioning). USB terminal is not used except for a welcome message.

===============================================================================================
*/


/*
===============================================================================
Wire master: message handling layer
===============================================================================
*/

#ifndef FG_MASTER_MESSAGES_h
#define FG_MASTER_MESSAGES_h

#include "master_context.h"
#include "master_transport.h"             // or master slave 

class MessageHandling {

    I2C_messageStats _msgStats{};   // message level stats

    GreenwichData& _greenwichData;
    StatusData& _statusData;
    SecondData& _secondData;

    SmoothedMeasurements& _smoothedMeasurements;
    PIDsettings& _pidSettings;
    LedStripSettings& _ledStripSettings;
    EventData& _globeEventSnapshot;
    VisualRing& _visualRing;

    int* _globeMetrics;

    WireMaster _wireMaster;

    using I2C_m_masterSendStats = WireMaster::I2C_MasterSendStats;
    using I2C_m_masterReceiveStats = WireMaster::I2C_MasterReceiveStats;

        
    /* ========== METHODS ========== */

public:
    MessageHandling(GreenwichData& greenwichData, StatusData& statusData, SecondData& secondData,
        SmoothedMeasurements& smoothedMeasurements, PIDsettings& pidSettings, int* globeMetrics,
        LedStripSettings& ledStripSettings, EventData& globeEventSnapshot, VisualRing& visualRing);

    ~MessageHandling();

    uint8_t transmit();
    void enqueueI2CmessageToSlave(uint8_t& msgTypeOut);
    void dequeueI2CmessageFromSlave(uint8_t& nextMsgTypeOut);
};

#endif

