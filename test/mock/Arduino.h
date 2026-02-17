#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cstdarg>

inline unsigned long millis() { return 0; }

class MockSerial {
public:
    void begin(unsigned long) {}

    int printf(const char* fmt, ...) {
        va_list args;
        va_start(args, fmt);
        int ret = vprintf(fmt, args);
        va_end(args);
        return ret;
    }

    void println(const char* s = "") { ::printf("%s\n", s); }
    void print(const char* s) { ::printf("%s", s); }
};

extern MockSerial Serial;
