# PlaceNet

The PlaceNet Beacon is an ESP32 + LoRa enabled device designed to discover other beacons over LoRa and negotiate with local servers (also running on the same network) to pair both networks over a TailScale like VPN system. I.e dynamically merging networks together over relatively short distances.

## Directory Structure

```
boards/     # Supported dev boards
├── t-beams3-supreme.json
└── t-deck.json

lib/
├── BLEModule/
│   ├── BLECallbacks.cpp
│   ├── BLECallbacks.h
│   ├── BLEModule.cpp
│   └── BLEModule.h
├── BoardUtility/
│   ├── BoardUtility.cpp
│   └── BoardUtility.h
├── config/
│   └── config.h
├── DisplayModule/
│   ├── DisplayModule.cpp
│   └── DisplayModule.h
├── GPSModule/
│   ├── GPSModule.cpp
│   └── GPSModule.h
├── Logging/
│   └── logger.h
├── LoRaModule/
│   ├── LoRaModule.cpp
│   └── LoRaModule.h
├── MemoryModule/
│   ├── PlaceNetConfig.cpp
│   ├── PlaceNetConfig.h
│   ├── README.md
│   ├── SDCardModule.cpp
│   └── SDCardModule.h
├── NetworkModule/
│   ├── NetworkModule.cpp
│   ├── NetworkModule.h
│   └── README
└── PMUModule/
    ├── PMUModule.cpp
    └── PMUModule.h

src/
├── main.cpp
├── managers/
│   ├── gps_manager.cpp
│   ├── gps_manager.h
│   ├── http_manager.cpp
│   ├── http_manager.h
│   ├── network_manager.cpp
│   ├── network_manager.h
│   ├── pmu_manager.cpp
│   └── pmu_manager.h
└── tasks/
    ├── location_task.cpp
    ├── location_task.h
    ├── lora_task.cpp
    ├── lora_task.h
    ├── main_task.cpp
    ├── main_task.h
    ├── pmu_task.cpp
    └── pmu_task.h

test/
├── embedded/
│   ├── test_ble_provisioning/
│   │   └── test_ble_provisioning.cpp
│   └── test_placenet_config_hw/
│       └── test_placenet_config_hw.cpp
├── mock/
│   ├── Arduino.cpp
│   ├── Arduino.h
│   └── library.json
└── native/
    ├── test_config_json/
    │   └── test_config_json.cpp
    └── test_placenet_config/
        └── test_placenet_config.cpp
```

## Running the project

The project uses platformio to build primarily off of the T_BEAM_S3_SUPREME_SX1262 board environment.

## Dev Notes

Everytime you modify the directory structure make sure to update CLAUDE.md 