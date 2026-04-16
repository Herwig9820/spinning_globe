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