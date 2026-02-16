# PlaceNet Beacon Configuration

## Overview

The PlaceNet Beacon uses a JSON configuration file stored on the SD card to manage WiFi credentials, MQTT broker settings, HTTP server endpoints, and beacon behavior.

## Configuration File Location

The configuration file must be stored at the root of the SD card:
- **Path**: `/config.json`
- **Format**: JSON (pretty-printed for readability)

## Configuration Structure

### WiFi Networks

Support for up to 3 WiFi networks with automatic fallback:

```json
"wifi": [
  {
    "ssid": "MyNetwork",
    "password": "MyPassword",
    "enabled": true
  }
]
```

- **ssid**: WiFi network name (max 32 characters)
- **password**: WiFi password (8-63 characters, or empty for open networks)
- **enabled**: Set to `true` to enable this network

### MQTT Configuration

MQTT broker settings for telemetry and command/control:

```json
"mqtt": {
  "broker": "mqtt.example.com",
  "port": 1883,
  "username": "beacon_user",
  "password": "mqtt_password",
  "clientId": "beacon_001",
  "baseTopic": "placenet/beacons",
  "enabled": true,
  "useTLS": false
}
```

- **broker**: MQTT broker hostname or IP address (max 128 characters)
- **port**: MQTT broker port (default: 1883 for non-TLS, 8883 for TLS)
- **username**: MQTT authentication username (optional)
- **password**: MQTT authentication password (optional)
- **clientId**: Unique client identifier (max 32 characters)
- **baseTopic**: Base topic prefix for all beacon messages (max 64 characters)
- **enabled**: Set to `true` to enable MQTT
- **useTLS**: Set to `true` to use TLS/SSL encryption

### HTTP Server Configuration

HTTP/HTTPS server endpoint for data upload:

```json
"httpServer": {
  "url": "api.example.com",
  "port": 443,
  "enabled": true,
  "useTLS": true
}
```

- **url**: Server hostname or IP address (max 128 characters)
- **port**: Server port (default: 80 for HTTP, 443 for HTTPS)
- **enabled**: Set to `true` to enable HTTP uploads
- **useTLS**: Set to `true` to use HTTPS

### Beacon Configuration

Beacon behavior and feature flags:

```json
"beacon": {
  "intervalMs": 30000,
  "loraEnabled": true,
  "gpsEnabled": true,
  "bleEnabled": false
}
```

- **intervalMs**: Beacon transmission interval in milliseconds (minimum 1000ms)
- **loraEnabled**: Enable LoRa radio transmission
- **gpsEnabled**: Enable GPS module
- **bleEnabled**: Enable BLE advertising

## Usage Example

### Loading Configuration

```cpp
#include "SDCardModule.h"
#include "PlaceNetConfig.h"

SDCardModule sdCard;
PlaceNetConfig config;

if (sdCard.init()) {
    if (sdCard.loadConfig(&config)) {
        config.print();  // Display loaded configuration

        // Access configuration values
        if (config.wifi[0].enabled) {
            Serial.printf("Connecting to: %s\n", config.wifi[0].ssid);
        }

        if (config.mqtt.enabled) {
            Serial.printf("MQTT Broker: %s:%d\n",
                         config.mqtt.broker, config.mqtt.port);
        }
    }
}
```

### Saving Configuration

```cpp
PlaceNetConfig config;

// Configure WiFi
strncpy(config.wifi[0].ssid, "MyNetwork", MAX_SSID_LENGTH - 1);
strncpy(config.wifi[0].password, "MyPassword", MAX_PASSWORD_LENGTH - 1);
config.wifi[0].enabled = true;

// Configure MQTT
strncpy(config.mqtt.broker, "mqtt.example.com", MAX_MQTT_BROKER_LENGTH - 1);
config.mqtt.port = 1883;
config.mqtt.enabled = true;

// Validate and save
if (config.validate()) {
    sdCard.saveConfig(&config);
}
```

## Configuration Validation

The `PlaceNetConfig::validate()` method performs the following checks:

- **WiFi SSID**: Length between 1-32 characters
- **WiFi Password**: Empty or 8-63 characters
- **MQTT Broker**: Non-empty, max 128 characters
- **MQTT Port**: Valid port number (1-65535)
- **HTTP Server URL**: Non-empty when enabled
- **HTTP Port**: Valid port number (1-65535)
- **Beacon Interval**: Warning if less than 1000ms

## Example Configuration File

See `config.example.json` in the project root for a complete example configuration file.

## Creating the Configuration File

1. Copy `config.example.json` to your SD card
2. Rename it to `config.json`
3. Edit the file with your WiFi credentials and server settings
4. Insert the SD card into your beacon
5. The beacon will load the configuration on boot

## Troubleshooting

### Configuration Not Loading

- Verify the SD card is properly inserted and formatted (FAT32)
- Check that the file is named exactly `config.json` (case-sensitive)
- Ensure the file is at the root of the SD card, not in a subdirectory
- Verify the JSON syntax is valid (use a JSON validator)

### Validation Errors

- Check Serial output for specific validation error messages
- Verify all required fields are present
- Ensure string lengths don't exceed maximum values
- Confirm port numbers are valid (1-65535)

### SD Card Initialization Failed

- Verify SD card is properly formatted (FAT32)
- Check SD card connections and pin definitions in `config.h`
- Ensure PMU has enabled SD card power rail (T-Beam S3 only)
- Try a different SD card (some cards are incompatible)

## Memory Considerations

- Configuration loading allocates 4KB for JSON parsing
- PlaceNetConfig structure size: ~1KB
- Memory is freed after loading/saving operations
- Configuration is loaded once on boot, not continuously
