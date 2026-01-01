#pragma once

#include <Arduino.h>

// Log level macros (ESP-IDF style)
#define LOGE(tag, format, ...) Serial.printf("E (%lu) %s: " format "\n", millis(), tag, ##__VA_ARGS__)
#define LOGW(tag, format, ...) Serial.printf("W (%lu) %s: " format "\n", millis(), tag, ##__VA_ARGS__)
#define LOGI(tag, format, ...) Serial.printf("I (%lu) %s: " format "\n", millis(), tag, ##__VA_ARGS__)
#define LOGD(tag, format, ...) Serial.printf("D (%lu) %s: " format "\n", millis(), tag, ##__VA_ARGS__)
#define LOGV(tag, format, ...) Serial.printf("V (%lu) %s: " format "\n", millis(), tag, ##__VA_ARGS__)
