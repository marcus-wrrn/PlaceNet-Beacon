#include "GPSModule.h"

#ifdef HAS_GPS

#include "logger.h"
#include <string.h>
#include <stdlib.h>

static const char* TAG = "GPS_MODULE";

GPSModule::GPSModule()
    : serial_(nullptr)
    , initialized_(false)
{}

GPSModule::~GPSModule() {
    if (initialized_ && serial_) {
        serial_->end();
        initialized_ = false;
    }
}

bool GPSModule::init() {
    if (initialized_) {
        LOGW(TAG, "GPS already initialized");
        return true;
    }

    LOGI(TAG, "Initializing GPS module...");
    LOGI(TAG, "GPS pins: RX=%d, TX=%d, BAUD=%d", GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD_RATE);

    serial_ = &Serial1;
    serial_->begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

#ifdef GPS_EN_PIN
    pinMode(GPS_EN_PIN, OUTPUT);
    digitalWrite(GPS_EN_PIN, HIGH);
    LOGI(TAG, "GPS power enabled on pin %d", GPS_EN_PIN);
    delay(100);
#endif

    initialized_ = true;
    LOGI(TAG, "GPS initialized successfully");
    return true;
}

bool GPSModule::readLine(char* buffer, size_t maxLen, uint32_t timeoutMs) {
    if (!serial_ || !initialized_) return false;

    uint32_t startTime = millis();
    size_t idx = 0;

    while (millis() - startTime < timeoutMs) {
        if (serial_->available()) {
            char c = serial_->read();

            if (c == '\n') {
                buffer[idx] = '\0';
                return idx > 0;
            } else if (c != '\r' && idx < maxLen - 1) {
                buffer[idx++] = c;
            }
        }
        yield();
    }

    buffer[idx] = '\0';
    return false;
}

int GPSModule::tokenizeNMEA(char* sentence, char** tokens, int maxTokens) {
    int count = 0;
    char* ptr = sentence;
    char* start = sentence;

    while (*ptr && count < maxTokens) {
        if (*ptr == ',') {
            *ptr = '\0';
            tokens[count++] = start;
            start = ptr + 1;
        }
        ptr++;
    }

    if (count < maxTokens) {
        tokens[count++] = start;
    }

    return count;
}

void GPSModule::parseTime(const char* timestr, GPSTime* time) {
    if (strlen(timestr) < 6) return;
    char buf[3] = {0};

    strncpy(buf, timestr, 2);
    time->hours = atoi(buf);
    strncpy(buf, timestr + 2, 2);
    time->minutes = atoi(buf);
    strncpy(buf, timestr + 4, 2);
    time->seconds = atoi(buf);
}

void GPSModule::parseDate(const char* datestr, GPSDate* date) {
    if (strlen(datestr) < 6) return;
    char buf[3] = {0};

    strncpy(buf, datestr, 2);
    date->day = atoi(buf);
    strncpy(buf, datestr + 2, 2);
    date->month = atoi(buf);
    strncpy(buf, datestr + 4, 2);
    date->year = atoi(buf);
}

double GPSModule::nmeaToDecimal(const char* coord, char direction) {
    double raw = atof(coord);
    int degrees = (int)(raw / 100);
    double minutes = raw - (degrees * 100);
    double decimal = degrees + minutes / 60.0;

    if (direction == 'S' || direction == 'W') {
        decimal *= -1;
    }

    return decimal;
}

bool GPSModule::parseGPRMC(char* sentence, GPSPosition* pos, GPSMetadata* meta, GPSTime* time, GPSDate* date) {
    if (strncmp(sentence, "$GPRMC", 6) != 0 && strncmp(sentence, "$GNRMC", 6) != 0) {
        return false;
    }

    char* tokens[13] = {0};
    int tokenCount = tokenizeNMEA(sentence, tokens, 13);

    if (tokenCount < 12) return false;

    parseTime(tokens[1], time);
    meta->fixStatus = tokens[2][0] == 'A';

    if (strlen(tokens[3]) > 0 && strlen(tokens[5]) > 0) {
        pos->latitude = nmeaToDecimal(tokens[3], tokens[4][0]);
        pos->longitude = nmeaToDecimal(tokens[5], tokens[6][0]);
        meta->ns = tokens[4][0];
        meta->ew = tokens[6][0];
    }

    if (strlen(tokens[7]) > 0) {
        meta->speedKnots = atof(tokens[7]);
    }

    if (strlen(tokens[9]) > 0) {
        parseDate(tokens[9], date);
    }

    return true;
}

bool GPSModule::parseGPGGA(char* sentence, GPSMetadata* meta) {
    if (strncmp(sentence, "$GPGGA", 6) != 0 && strncmp(sentence, "$GNGGA", 6) != 0) {
        return false;
    }

    char* tokens[15] = {0};
    int tokenCount = tokenizeNMEA(sentence, tokens, 15);

    if (tokenCount < 10) return false;

    if (strlen(tokens[7]) > 0) {
        meta->satelliteCount = atoi(tokens[7]);
    }

    if (strlen(tokens[9]) > 0) {
        meta->altitude = atof(tokens[9]);
    }

    return true;
}

bool GPSModule::readData(GPSData* data, uint32_t timeoutMs) {
    if (!initialized_ || !serial_) {
        LOGE(TAG, "GPS not initialized");
        return false;
    }

    char line[128];
    bool parsedGPRMC = false;
    bool parsedGPGGA = false;
    uint32_t startTime = millis();

    memset(data, 0, sizeof(GPSData));

    while (!(parsedGPGGA && parsedGPRMC) && (millis() - startTime < timeoutMs)) {
        if (readLine(line, sizeof(line), 1000)) {
            if (!parsedGPRMC && (strstr(line, "$GPRMC") != NULL || strstr(line, "$GNRMC") != NULL)) {
                if (parseGPRMC(line, &data->position, &data->metadata, &data->time, &data->date)) {
                    parsedGPRMC = true;
                    LOGV(TAG, "Parsed GPRMC sentence");
                }
            } else if (!parsedGPGGA && (strstr(line, "$GPGGA") != NULL || strstr(line, "$GNGGA") != NULL)) {
                if (parseGPGGA(line, &data->metadata)) {
                    parsedGPGGA = true;
                    LOGV(TAG, "Parsed GPGGA sentence");
                }
            }
        }
    }

    if (!(parsedGPGGA && parsedGPRMC)) {
        LOGW(TAG, "Timeout waiting for GPS data (GPRMC=%d, GPGGA=%d)", parsedGPRMC, parsedGPGGA);
        return false;
    }

    return true;
}

void GPSModule::logData(const GPSData* data) {
    LOGI(TAG, "========== GPS DATA ==========");
    LOGI(TAG, "Position:");
    LOGI(TAG, "   Latitude : %.6f %c", data->position.latitude, data->metadata.ns);
    LOGI(TAG, "   Longitude: %.6f %c", data->position.longitude, data->metadata.ew);

    LOGI(TAG, "Time:");
    LOGI(TAG, "   %02d:%02d:%02d", data->time.hours, data->time.minutes, data->time.seconds);

    LOGI(TAG, "Date:");
    LOGI(TAG, "   %02d/%02d/%04d", data->date.day, data->date.month, data->date.year + 2000);

    LOGI(TAG, "Metadata:");
    LOGI(TAG, "   Fix Status     : %s", data->metadata.fixStatus ? "Valid" : "Invalid");
    LOGI(TAG, "   Satellites     : %d", data->metadata.satelliteCount);
    LOGI(TAG, "   Altitude       : %.2f m", data->metadata.altitude);
    LOGI(TAG, "   Speed (knots)  : %.2f", data->metadata.speedKnots);
    LOGI(TAG, "==============================");
}

#endif // HAS_GPS
