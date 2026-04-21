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


#ifndef FG_SLAVE_MESSAGES_h
#define FG_SLAVE_MESSAGES_h

#include "bridge_context.h"
#include "wireSlave_transport.h"


// MASTER send & receive stats, received as message. Sole purpose: receive wire stats from master 

class WireSlaveMessages {

    struct WireStatSummary {
        uint32_t slaveErrorMsgCount{ 0 };
        uint32_t slaveValidMsgCount{ 0 };
        uint32_t masterErrorMsgCount{ 0 };
        uint32_t masterValidMsgCount{ 0 };

        void inline zeroMemebers() {
            slaveErrorMsgCount = slaveValidMsgCount = masterErrorMsgCount = masterValidMsgCount = 0;
        }
    };

    static constexpr uint32_t wireCommQuality_measPeriod = 2560; // 20 * 128 ms
    
    uint32_t pingCount{0}; 
    WireSlave wireSlave;

    SharedContext& _sharedContext;
    WireStatSummary wireStatSummary{};

    void convertGlobeStatusToMQTT(I2C_m_status* p);
    void convertGlobeGreenwichCueToMQTT(I2C_m_greenwich* p);
    void convertTelemetryToMQTT(I2C_m_telemetry* p);
    void convertTelemetryExtraToMQTT(I2C_m_telemetry_extra* p);
    void convertGlobeSettingsToMQTT(I2C_m_globeSettings* p);
    void convertPIDsettingsToMQTT(I2C_m_PIDsettings* p);
    void convertVertPosSetpointToMQTT(I2C_m_vertPosSetpoint* p);
    void convertCoilPhaseAdjustmentToMQTT(I2C_m_coilPhaseAdjust* p);

    void prepareWirecommStatsForMQTT();

    void replyAndFlagSlaveDataAvailable(uint8_t msgSequence);

public:
    WireSlaveMessages(SharedContext& sharedContext);
    bool loop();


};


#endif

