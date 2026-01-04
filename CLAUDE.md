# PlaceNet-Beacon Project Documentation

## Overview

PlaceNet-Beacon is an ESP32-based LoRa beacon/tracker firmware designed for **LilyGo ESP32-S3 development boards**. It provides a foundation for building location-aware, long-range wireless communication devices suitable for IoT tracking, mesh networking, or beacon applications.

**Hardware Platform**: ESP32-S3 (dual-core, 240MHz)
**Build System**: PlatformIO with Arduino framework
**Architecture**: FreeRTOS task-based with modular hardware abstraction
**Primary Use Case**: GPS-enabled LoRa beacon

## Supported Boards

### LilyGo T-Beam S3 Supreme
- **Flash**: 8MB, **PSRAM**: 8MB
- **Display**: I2C SH1106 OLED (128x64)
- **PMU**: AXP2101/AXP192 (battery management)
- **LoRa**: SX1262 or LR1121
- **GPS**: L76K/UBlox with power control
- **Product**: https://lilygo.cc/products/t-beam-supreme

### LilyGo T-Deck
- **Flash**: 16MB, **PSRAM**: 8MB
- **Display**: ST7789 SPI LCD (320x240) - **not yet implemented**
- **PMU**: None (direct battery with ADC monitoring)
- **LoRa**: SX1262
- **GPS**: Optional UART
- **Unique Features**: Physical keyboard (I2C), trackball, speaker/microphone
- **Product**: https://lilygo.cc/products/t-deck
- **Note**: T-Deck requires GPIO10 HIGH to enable peripheral power

## Project Structure

```
PlaceNet-Beacon/
├── lib/                          # Custom libraries (hardware modules)
│   ├── PMUModule/                # Power management (non-singleton)
│   ├── DisplayModule/            # Display rendering with queue
│   ├── LoRaModule/               # LoRa radio with RadioLib (SX1262/LR1121)
│   ├── BoardUtility/             # Static board utilities
│   ├── Logging/                  # Custom logging macros
│   └── config/
│       └── config.h              # Pin definitions & configuration
├── src/
│   ├── main.cpp                  # FreeRTOS initialization & main loop
│   └── tasks/                    # FreeRTOS task implementations
│       ├── pmu_task.h/.cpp       # PMU event handling task
│       ├── display_task.h/.cpp   # Display rendering task
│       └── lora_task.h/.cpp      # LoRa beacon transmission task
├── boards/
│   ├── t-beams3-supreme.json     # T-Beam S3 board definition
│   └── t-deck.json               # T-Deck board definition
├── platformio.ini                # Build configuration
└── CLAUDE.md                     # This file
```

## Architecture

### Design Patterns

- **Module Pattern**: Independent, instantiable hardware abstraction classes
- **Task-Based Concurrency**: FreeRTOS tasks with dedicated priorities and cores
- **Message Passing**: Inter-task communication via queues and task notifications
- **Event-Driven**: PMU uses ISR → task notification (not polling)

**Task Priorities**:
- PMU Task: `configMAX_PRIORITIES - 1` (highest, ~24) - Critical power events
- LoRa Task: 8 (medium-high) - Beacon transmission, stack: 8192 bytes
- Display Task: 5 (medium) - Non-critical UI updates
- Main Loop: 10 (default Arduino loop priority)

## Core Components

### 1. PMUModule (`lib/PMUModule/`)

**Purpose**: Interrupt-driven power management for AXP192/AXP2101

**Features**:
- FreeRTOS task notification support (`setTaskHandle()`)
- ISR uses `vTaskNotifyGiveFromISR()` to wake PMU task
- Power rail configuration (GPS, LoRa, Display, SD Card, Sensors)
- Battery/VBUS monitoring
- Button press event handling

**Key Methods**:
- `initialize()` - Detect and configure PMU chip
- `setTaskHandle(TaskHandle_t)` - Register task for ISR notifications
- `processEvents()` - Handle interrupt events (called by task)
- `getBatteryVoltage()`, `isCharging()`, `hasBattery()`

**Usage**:
```cpp
PMUModule* pmu = new PMUModule(Wire1, PMU_IRQ);
pmu->initialize();
xTaskCreatePinnedToCore(pmuTask, "PMU", 4096, pmu, 24, &handle, 0);
pmu->setTaskHandle(handle);
```

### 2. DisplayModule (`lib/DisplayModule/`)

**Purpose**: Queue-based display rendering (SH1106/SSD1306)

**Features**:
- Command queue for thread-safe rendering
- Supports: Clear, DrawText, DrawHLine, DrawVLine, SetFont, SendBuffer
- U8g2 graphics library integration

**Command Structure**:
```cpp
DisplayCommand cmd;
cmd.type = DISPLAY_DRAW_TEXT;
cmd.data.drawText.x = 0;
cmd.data.drawText.y = 16;
strcpy(cmd.data.drawText.text, "Hello World");
xQueueSend(display->getCommandQueue(), &cmd, 0);
```

**Usage**:
```cpp
DisplayModule* display = new DisplayModule();
xTaskCreatePinnedToCore(displayTask, "Display", 4096, display, 5, NULL, 1);
```

### 3. LoRaModule (`lib/LoRaModule/`)

**Purpose**: LoRa radio TX/RX with RadioLib (SX1262/LR1121)

**Features**:
- RadioLib-based SX1262 driver integration
- TX/RX queues (10 deep each) for packet handling
- Blocking `transmit()` and queued `send()` methods
- Duty cycle tracking with rolling time windows (1min, 10min, 1hour)
- Configurable radio parameters (frequency, bandwidth, spreading factor, power)
- Time-on-air calculation for regulatory compliance

**Duty Cycle Tracking**:
Maintains circular buffer of last 100 transmissions with timestamps and time-on-air values. Enables regulatory compliance monitoring.

**Usage**:
```cpp
LoRaModule* lora = new LoRaModule();
xTaskCreatePinnedToCore(loraTask, "LoRa", 8192, lora, 8, NULL, 1);
// Task calls lora->init() and lora->transmit() for beaconing
```

**Current Implementation**:
The LoRa task (`src/tasks/lora_task.cpp`) transmits a beacon URL every 30 seconds using blocking `transmit()`. Duty cycle is reported every 60 seconds to Serial.

### 4. BoardUtility (`lib/BoardUtility/`)

**Purpose**: Static utility functions for board diagnostics

**Functions**:
- `printChipInfo()` - ESP32 model, PSRAM, flash, MAC address
- `printWakeupReason()` - Deep sleep wakeup cause
- `scanI2C(TwoWire*)` - Detect I2C devices on bus
- `getDeviceInfo()` - Return DevInfo_t struct

**Usage**:
```cpp
BoardUtility::printChipInfo();
BoardUtility::scanI2C(&Wire1);
```

### 5. Logging (`lib/Logging/logger.h`)

**Purpose**: Unified logging interface with timestamps

**Custom Logging Macros** (Serial.printf-based):
- `LOGI(tag, format, ...)` - Info level with timestamp
- `LOGW(tag, format, ...)` - Warning level
- `LOGE(tag, format, ...)` - Error level
- `LOGD(tag, format, ...)` - Debug level
- `LOGV(tag, format, ...)` - Verbose level

### 6. Configuration (`lib/config/config.h`)

**Purpose**: Centralized hardware pin definitions, feature flags, and radio parameters

See `lib/config/config.h` for board-specific pin assignments.

## Hardware Resources & Dependencies

### T-Beam S3 Supreme Hardware

#### I2C Buses

**Wire (I2C0)** - Pins: SDA=17, SCL=18
- Display (SH1106 @ 0x3C)

**Wire1 (I2C1)** - Pins: SDA=42, SCL=41
- PMU (AXP2101/AXP192 @ 0x34)
- PMU_IRQ on GPIO40

#### SPI Buses

**SPI (HSPI)** - Radio
- Pins: MOSI=11, MISO=13, SCLK=12, CS=10
- Additional: RST=5, DIO1=1, BUSY=4
- Devices: LoRa Radio (SX1262/LR1121)

#### UART

**Serial1** - GPS Module
- Pins: RX=9, TX=8
- Additional: EN=7 (power enable), PPS=6
- Baud: 9600
- Devices: L76K or UBlox GPS

#### Critical Dependencies

**PMU MUST initialize first** - It powers:
- GPS (ALDO4: 3.3V)
- LoRa (ALDO3: 3.3V)
- Sensors (ALDO1/2: 3.3V)
- SD Card (BLDO1/2: 3.3V)
- Display I2C bus

### T-Deck Hardware

#### I2C Bus

**Wire (I2C0)** - Pins: SDA=18, SCL=8
- Keyboard controller (interrupt on GPIO46)

#### SPI Bus (Shared)

**SPI** - Pins: MOSI=41, MISO=38, SCLK=40
- LoRa Radio: CS=9, RST=17, DIO1=45, BUSY=13
- SD Card: CS=39
- Display (not implemented): CS=12, DC=11, BL=42

#### UART

**GPS Module** (Optional)
- Pins: RX=44, TX=43
- Baud: 9600

#### Power Control

**CRITICAL**: GPIO10 must be set HIGH before peripheral initialization
- Controls power to LoRa, SD card, and other peripherals
- Implemented in `src/main.cpp` via `BOARD_POWERON_PIN`

#### Battery Monitoring

- ADC Pin: GPIO4
- Direct battery voltage sensing (no PMU)

## Task Communication

### Task Notifications
- **PMU Task**: Woken by ISR via `vTaskNotifyGiveFromISR()`

### Mutexes
- None currently needed (I2C buses are separate)
- ESP-IDF logging has built-in mutex protection

## Development Guidelines

### Adding a New Module

1. Create `lib/YourModule/YourModule.h/.cpp`
2. Constructor: Initialize queues/member variables
3. `init()`: Hardware initialization
4. Provide accessor methods (e.g., `getQueue()`)
5. Create `src/tasks/your_task.h/.cpp`
6. Update `main.cpp` to spawn task

### Thread Safety

- Use queues for inter-task communication
- Use task notifications for ISR → task signaling
- Use `LOG*()` macros for consistent logging (thread-safe Serial.printf)
- Avoid shared state; prefer message passing

### Power Management

- PMU controls all peripheral power rails
- Call `pmu->disablePeripherals()` before deep sleep
- Use `esp_sleep_enable_*()` for wakeup sources

## Dependencies

**PlatformIO Libraries** (`platformio.ini`):
- `jgromes/RadioLib@^7.4.0` - LoRa/FSK radio abstraction (SX1262, LR1121, etc.) - **all boards**
- `olikraus/U8g2@^2.36.15` - Monochrome OLED/LCD graphics library - **T-Beam only**
- `lewisxhe/XPowersLib@^0.3.2` - AXP192/AXP2101 PMU driver - **T-Beam only**

**Platform**:
- `espressif32@6.9.0` - ESP32 Arduino framework
- Partition scheme: `huge_app.csv` (large application partition for PSRAM usage)

**Board Definitions**:
- `boards/t-beams3-supreme.json` - T-Beam S3 (8MB Flash, 8MB PSRAM)
- `boards/t-deck.json` - T-Deck (16MB Flash, 8MB PSRAM)

**PlatformIO Environments**:
- `T_BEAM_S3_SUPREME_SX1262` - T-Beam with SX1262 radio
- `T_BEAM_S3_SUPREME_LR1121` - T-Beam with LR1121 radio (future)
- `T_DECK_SX1262` - T-Deck with SX1262 radio

**Build Flags**:
- `ARDUINO_USB_CDC_ON_BOOT=1` - Enable USB CDC for Serial communication
- `BOARD_HAS_PSRAM` - Enable PSRAM support (8MB)
- Board variant flag (e.g., `T_BEAM_S3_SUPREME_SX1262` or `T_DECK_SX1262`)

## Future Extensions

**General**:
- GPS Task with NMEA parsing (TinyGPS++) and position beaconing
- LoRa RX implementation with interrupt-driven packet reception
- LoRa mesh networking (packet routing, hop count, RSSI-based path selection)
- SD Logging Task with async writes
- Event Groups for system state (GPS_LOCKED, LORA_READY)
- Deep sleep coordination with periodic wakeup for beaconing
- BLE configuration interface (radio parameters, beacon interval, GPS enable)

**T-Deck Specific**:
- ST7789 SPI display driver integration (TFT_eSPI or Arduino_GFX)
- Keyboard module for I2C input handling
- Trackball GPIO reading for navigation
- Battery ADC monitoring task
- Audio support (speaker/microphone via ES7210 I2S codec)

## Dev Notes

Do not abuse comments. Code should be self-documenting where possible.
