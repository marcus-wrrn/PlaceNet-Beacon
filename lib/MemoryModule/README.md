# MemoryModule

Handles all persistent storage for PlaceNet-Beacon configuration and data.

## Components

### PlaceNetConfig
Configuration structure for beacon settings, network credentials, and connectivity options:
- WiFi credentials (up to 3 networks)
- MQTT broker configuration
- HTTP server settings
- Beacon parameters (interval, enabled features)

### SDCardModule
SD card interface for persistent storage (T-Beam only):
- JSON-based config file management (`/config.json`)
- Generic file operations (read, write, delete, list)
- SD card diagnostics (size, type)

## Usage

Configuration is stored on SD card and loaded at boot. Changes persist across power cycles and deep sleep.

```cpp
PlaceNetConfig config;
SDCardModule* sd = new SDCardModule();

if (sd->init() && sd->loadConfig(&config)) {
    // Use config.wifi, config.mqtt, config.beacon
}
```
