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


#ifndef FG_JSON_HELPERS_h
#define FG_JSON_HELPERS_h

#pragma once
#include <ctype.h>              
#include <string.h>
#include <stdlib.h>

namespace JsonAssemble {
// A tiny helper to append a JSON key/value pair into a buffer.
// Always safe: never writes past bufferSize, always null-terminates.

    inline void add(char* buffer, size_t bufferSize,
        const char* key, const char* fmt, ...)
    {
        size_t len = strlen(buffer);

        // Add comma if not the first element
        if ((len > 1) && (buffer[len - 1] != '{') && (buffer[len - 1] != '[')) {
            if (len < bufferSize - 1) {
                buffer[len++] = ',';
                buffer[len] = '\0';
            }
        }

        // Append "key":
        int written = snprintf(buffer + len, bufferSize - len, "\"%s\":", key);
        if (written < 0) return;
        len += written;

        // Append formatted value
        va_list args;
        va_start(args, fmt);
        vsnprintf(buffer + len, bufferSize - len, fmt, args);
        va_end(args);
    }

    inline void startJson(char* buffer, size_t bufferSize) {
        strlcpy(buffer, "{", bufferSize);
    }

    inline void closeJson(char* buffer, size_t bufferSize) {
        strlcat(buffer, "}", bufferSize);
    }

    inline void startArray(char* buffer, size_t bufferSize) {
        strlcat(buffer, "[", bufferSize);
    }

    inline void endArray(char* buffer, size_t bufferSize) {
        strlcat(buffer, "]", bufferSize);
    }
} // namespace JsonAssemble


namespace JsonParse {

    // --------------------------------------------------------------------
    // Trim leading whitespace only (spaces, tabs, CR, LF)
    // --------------------------------------------------------------------
    inline const char* trim(const char* s) {
        while (*s == ' ' || *s == '\t' || *s == '\n' || *s == '\r')
            s++;
        return s;
    }

    // --------------------------------------------------------------------
    // Helper: find "key":
    // --------------------------------------------------------------------
    inline const char* findKey(const char* json, const char* key) {
        static char pattern[64];
        snprintf(pattern, sizeof(pattern), "\"%s\":", key);
        return strstr(json, pattern);
    }

    // --------------------------------------------------------------------
    // STRING: "key":"value"
    // --------------------------------------------------------------------
    inline bool getString(const char* json, const char* key,
        char* outBuffer, size_t outSize)
    {
        if (!json || !key || !outBuffer || outSize == 0) return false;

        const char* p = findKey(json, key);
        if (!p) return false;

        p += strlen(key) + 3; // skip "key":
        p = trim(p);

        if (*p != '\"') return false;
        p++;

        size_t i = 0;
        while (*p && *p != '\"' && i < outSize - 1) {
            outBuffer[i++] = *p++;
        }
        outBuffer[i] = '\0';

        return (*p == '\"');
    }

    // --------------------------------------------------------------------
    // UNSIGNED INT: "key":123
    // --------------------------------------------------------------------
    inline bool getUInt(const char* json, const char* key, unsigned* outValue)
    {
        if (!json || !key || !outValue) return false;

        const char* p = findKey(json, key);
        if (!p) return false;

        p += strlen(key) + 3;
        p = trim(p);

        if (!isdigit(*p)) return false;

        *outValue = (unsigned)strtoul(p, nullptr, 10);
        return true;
    }

    // --------------------------------------------------------------------
    // SIGNED INT: "key":-123
    // --------------------------------------------------------------------
    inline bool getInt(const char* json, const char* key, int* outValue)
    {
        if (!json || !key || !outValue) return false;

        const char* p = findKey(json, key);
        if (!p) return false;

        p += strlen(key) + 3;
        p = trim(p);

        if (!isdigit(*p) && *p != '-') return false;

        *outValue = (int)strtol(p, nullptr, 10);
        return true;
    }

    // --------------------------------------------------------------------
    // FLOAT: "key":12.34
    // --------------------------------------------------------------------
    inline bool getFloat(const char* json, const char* key, float* outValue)
    {
        if (!json || !key || !outValue) return false;

        const char* p = findKey(json, key);
        if (!p) return false;

        p += strlen(key) + 3;
        p = trim(p);

        if (!isdigit(*p) && *p != '-' && *p != '.') return false;

        *outValue = strtof(p, nullptr);
        return true;
    }

    // --------------------------------------------------------------------
    // BOOLEAN: "key":true / "key":false / "key":"true"
    // --------------------------------------------------------------------
    inline bool getBool(const char* json, const char* key, bool* outValue)
    {
        if (!json || !key || !outValue) return false;

        const char* p = findKey(json, key);
        if (!p) return false;

        p += strlen(key) + 3;
        p = trim(p);

        // unquoted
        if (strncmp(p, "true", 4) == 0) { *outValue = true;  return true; }
        if (strncmp(p, "false", 5) == 0) { *outValue = false; return true; }

        // quoted
        if (*p == '\"') {
            p++;
            if (strncmp(p, "true", 4) == 0) { *outValue = true;  return true; }
            if (strncmp(p, "false", 5) == 0) { *outValue = false; return true; }
        }

        return false;
    }

    // --------------------------------------------------------------------
    // ARRAY OF UNSIGNED INTS: "key":[1,2,3]
    // --------------------------------------------------------------------
    inline int getUIntArray(const char* json, const char* key,
        unsigned* outArray, int maxCount)
    {
        if (!json || !key || !outArray || maxCount <= 0) return 0;

        const char* p = findKey(json, key);
        if (!p) return 0;

        p += strlen(key) + 3;
        p = trim(p);

        if (*p != '[') return 0;
        p++;

        int count = 0;
        while (*p && *p != ']' && count < maxCount) {
            p = trim(p);
            if (!isdigit(*p)) break;

            outArray[count++] = (unsigned)strtoul(p, (char**)&p, 10);

            p = trim(p);
            if (*p == ',') p++;
        }

        return count;
    }

    // --------------------------------------------------------------------
    // ARRAY OF FLOATS: "key":[1.2,3.4,5.6]
    // --------------------------------------------------------------------
    inline int getFloatArray(const char* json, const char* key,
        float* outArray, int maxCount)
    {
        if (!json || !key || !outArray || maxCount <= 0) return 0;

        const char* p = findKey(json, key);
        if (!p) return 0;

        p += strlen(key) + 3;
        p = trim(p);

        if (*p != '[') return 0;
        p++;

        int count = 0;
        while (*p && *p != ']' && count < maxCount) {
            p = trim(p);
            if (!isdigit(*p) && *p != '-' && *p != '.') break;

            outArray[count++] = strtof(p, (char**)&p);

            p = trim(p);
            if (*p == ',') p++;
        }

        return count;
    }

    // --------------------------------------------------------------------
    // ARRAY OF STRINGS: "key":["a","b","c"]
    // Caller provides outArray[N][maxLen]
    // --------------------------------------------------------------------
    inline int getStringArray(const char* json, const char* key,
        char outArray[][32], int maxCount)
    {
        if (!json || !key || !outArray || maxCount <= 0) return 0;

        const char* p = findKey(json, key);
        if (!p) return 0;

        p += strlen(key) + 3;
        p = trim(p);

        if (*p != '[') return 0;
        p++;

        int count = 0;
        while (*p && *p != ']' && count < maxCount) {
            p = trim(p);
            if (*p != '\"') break;
            p++;

            int i = 0;
            while (*p && *p != '\"' && i < 31) {
                outArray[count][i++] = *p++;
            }
            outArray[count][i] = '\0';

            if (*p == '\"') p++;

            count++;

            p = trim(p);
            if (*p == ',') p++;
        }

        return count;
    }

} // namespace JsonParse

#endif 