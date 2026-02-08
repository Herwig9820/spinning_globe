#ifndef _JSON_HELPERS_h
#define _JSON_HELPERS_h

#pragma once
#include <ctype.h>              //// check: nodig ?
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
}


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