#ifndef _JSON_HELPERS_h
#define _JSON_HELPERS_h

#endif // !_JSON_HELPERS_h

namespace JsonUtil {
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
        strlcat(buffer, "[", bufferSize);
    }

    inline void closeJson(char* buffer, size_t bufferSize) {
        strlcat(buffer, "]", bufferSize);
    }

    inline void startArray(char* buffer, size_t bufferSize) {
        strlcat(buffer, "[", bufferSize);
    }

    inline void endArray(char* buffer, size_t bufferSize) {
        strlcat(buffer, "]", bufferSize);
    }
}
