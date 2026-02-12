#ifndef FG_DEBUG_h
#define FG_DEBUG_h


// ============================================================
// Debug configuration
// ============================================================

// Master switch: enable/disable all debug output
#define DEBUG 1

// Optional: enable ANSI colors (only works in real terminals)
// Set to 0 if using Arduino Serial Monitor
#define DEBUG_COLOR 0


// ============================================================
// Color codes (only active when DEBUG_COLOR = 1)
// ============================================================
#if DEBUG && DEBUG_COLOR
#define COLOR_RED     "\033[31m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_BLUE    "\033[34m"
#define COLOR_RESET   "\033[0m"
#else
#define COLOR_RED
#define COLOR_GREEN
#define COLOR_YELLOW
#define COLOR_BLUE
#define COLOR_RESET
#endif


// ============================================================
// Debug macros
// ============================================================
#if DEBUG

  // Basic printing
#define DEBUG_BEGIN(...)        Serial.begin(__VA_ARGS__)
#define DEBUG_PRINT(...)        Serial.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...)      Serial.println(__VA_ARGS__)

// Timestamped printing
#define DEBUG_PRINT_TS(msg) \
      do { Serial.print(millis()); Serial.print(" ms: "); Serial.print(msg); } while(0)

#define DEBUG_PRINTLN_TS(msg) \
      do { Serial.print(millis()); Serial.print(" ms: "); Serial.println(msg); } while(0)

  // Hex printing
#define DEBUG_PRINT_HEX(val)        Serial.print((val), HEX)
#define DEBUG_PRINTLN_HEX(val)      Serial.println((val), HEX)

// Fixed-width 2‑digit hex (00–FF)
#define DEBUG_PRINT_HEX8(val) \
      do { if ((uint8_t)(val) < 0x10) Serial.print('0'); Serial.print((uint8_t)(val), HEX); } while(0)

#define DEBUG_PRINTLN_HEX8(val) \
      do { DEBUG_PRINT_HEX8(val); Serial.println(); } while(0)

  // Binary printing
#define DEBUG_PRINT_BIN(val)        Serial.print((val), BIN)
#define DEBUG_PRINTLN_BIN(val)      Serial.println((val), BIN)

// Hex dump of a byte buffer (16 bytes per line)
#define DEBUG_DUMP_HEX(buf, len) \
      do { \
        for (size_t _i = 0; _i < (len); _i++) { \
          if (_i % 16 == 0) Serial.println(); \
          if (buf[_i] < 0x10) Serial.print('0'); \
          Serial.print(buf[_i], HEX); \
          Serial.print(' '); \
        } \
        Serial.println(); \
      } while(0)

#else

  // All debug disabled → compile to nothing
#define DEBUG_BEGIN(...)
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#define DEBUG_PRINT_TS(msg)
#define DEBUG_PRINTLN_TS(msg)
#define DEBUG_PRINT_HEX(val)
#define DEBUG_PRINTLN_HEX(val)
#define DEBUG_PRINT_HEX8(val)
#define DEBUG_PRINTLN_HEX8(val)
#define DEBUG_PRINT_BIN(val)
#define DEBUG_PRINTLN_BIN(val)
#define DEBUG_DUMP_HEX(buf, len)

#endif

#endif
