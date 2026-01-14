#pragma once

#include "config.h"

#ifdef HAS_GPS

#include <Arduino.h>
#include <HardwareSerial.h>

struct GPSPosition {
    double latitude;
    double longitude;
};

struct GPSMetadata {
    char ns;
    char ew;
    bool fixStatus;
    double speedKnots;
    double altitude;
    int satelliteCount;
};

struct GPSTime {
    int hours;
    int minutes;
    int seconds;
};

struct GPSDate {
    int day;
    int month;
    int year;
};

struct GPSData {
    GPSPosition position;
    GPSMetadata metadata;
    GPSDate date;
    GPSTime time;
};

class GPSModule {
public:
    GPSModule();
    ~GPSModule();

    bool init();
    bool isInitialized() const { return initialized_; }

    bool readData(GPSData* data, uint32_t timeoutMs = 5000);

    void logData(const GPSData* data);

    bool isFixValid(const GPSData* data) const { return data->metadata.fixStatus; }

private:
    HardwareSerial* serial_;
    bool initialized_;

    int tokenizeNMEA(char* sentence, char** tokens, int maxTokens);
    void parseTime(const char* timestr, GPSTime* time);
    void parseDate(const char* datestr, GPSDate* date);
    double nmeaToDecimal(const char* coord, char direction);
    bool parseGPRMC(char* sentence, GPSPosition* pos, GPSMetadata* meta, GPSTime* time, GPSDate* date);
    bool parseGPGGA(char* sentence, GPSMetadata* meta);
    bool readLine(char* buffer, size_t maxLen, uint32_t timeoutMs);
};

#endif // HAS_GPS
