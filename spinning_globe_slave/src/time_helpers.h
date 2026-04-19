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


#ifndef FG_TIME_HELPERS_h
#define FG_TIME_HELPERS_h

#include <time.h>
#include <Arduino.h>

namespace timeHelpers {
    static bool _ntpSynced{ false };

    inline bool ntpSynced() {
        if (!_ntpSynced) { _ntpSynced = (sntp_get_sync_status() == SNTP_SYNC_STATUS_COMPLETED); } 
        return _ntpSynced;
    }

    inline bool getLocalTimeString(char* buf, size_t bufSize) {
        if (!ntpSynced()) return false;
        tm timeinfo;
        getLocalTime(&timeinfo);
        strftime(buf, bufSize, "%d/%m/%y %H:%M:%S", &timeinfo);
        return true;
    }

    inline bool getLocalTimeStruct(tm* pTimeInfo) {
        if (ntpSynced()) return false;
        getLocalTime(pTimeInfo);
        return true;
    }

    inline bool getUTC(time_t* pNow) {
        if (!ntpSynced()) return false;
        time(pNow);
        return true;
    }

}

#endif