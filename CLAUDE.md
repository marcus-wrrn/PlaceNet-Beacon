# PlaceNet-Beacon Project Documentation

## Overview

PlaceNet-Beacon is an ESP32-based LoRa beacon/tracker firmware designed for the **LilyGo T-Beam S3 Supreme** development board. It provides a foundation for building location-aware, long-range wireless communication devices suitable for IoT tracking, mesh networking, or beacon applications.

**Hardware Platform**: ESP32-S3 (dual-core, 240MHz, 8MB Flash, 8MB PSRAM)
**Build System**: PlatformIO with Arduino framework
**Architecture**: FreeRTOS task-based with modular hardware abstraction
**Primary Use Case**: GPS-enabled LoRa beacon with PMU

## Project Structure

```
PlaceNet-Beacon/
├── lib/                          # Custom libraries (hardware modules)
│   ├── PMUModule/                # Power management (non-singleton)
│   ├── DisplayModule/            # Display rendering with queue
│   ├── LoRaModule/               # LoRa radio (stub implementation)
│   ├── BoardUtility/             # Static board utilities
│   └── config/
│       └── config.h              # Pin definitions & configuration
├── src/
│   ├── main.cpp                  # FreeRTOS initialization & main loop
│   └── tasks/                    # FreeRTOS task implementations
│       ├── pmu_task.h/.cpp       # PMU event handling task
│       ├── display_task.h/.cpp   # Display rendering task
│       └── lora_task.h/.cpp      # LoRa TX/RX task (stub)
├── boards/
│   └── t-beams3-supreme.json     # Custom PlatformIO board definition
├── platformio.ini                # Build configuration
└── CLAUDE.md                     # This file
```

## Architecture

### Design Patterns

- **Module Pattern**: Independent, instantiable hardware abstraction classes
- **Task-Based Concurrency**: FreeRTOS tasks with dedicated priorities and cores
- **Message Passing**: Inter-task communication via queues and task notifications
- **Event-Driven**: PMU uses ISR → task notification (not polling)

### FreeRTOS Task Architecture

```
┌─────────────┐     Task Notification      ┌──────────────┐
│  PMU ISR    │ ─────────────────────────> │   PMU Task   │ (Priority: 24, Core 0)
└─────────────┘                            └──────────────┘
                                                   │
                                                   │ Power Events
                                                   ▼
┌─────────────┐     Display Queue          ┌──────────────┐
│  Main Loop  │ ─────────────────────────> │ Display Task │ (Priority: 5, Core 1)
└─────────────┘                            └──────────────┘
       │
       │ Init, Coordination
       ▼
┌──────────────┐
│  LoRa Task   │  (Future - stub only)
└──────────────┘
```

**Task Priorities**:
- PMU Task: `configMAX_PRIORITIES - 1` (highest, ~24) - Critical power events
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
// Display task calls display->init() and processes queue
```

### 3. LoRaModule (`lib/LoRaModule/`) - STUB

**Purpose**: LoRa radio TX/RX (SX1262/LR1121) - **Not yet implemented**

**Planned Features**:
- TX/RX queues for packet handling
- RadioLib integration
- Configurable frequency, bandwidth, spreading factor
- RSSI/SNR reporting

**TODO**:
- Implement `init()` with RadioLib
- Add packet transmission logic
- Add receive interrupt handling

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

### 5. Logging

**Uses ESP-IDF native logging** (thread-safe by default):
- `ESP_LOGI(TAG, "message")` - Info level
- `ESP_LOGW(TAG, "message")` - Warning
- `ESP_LOGE(TAG, "message")` - Error
- `ESP_LOGD(TAG, "message")` - Debug
- `ESP_LOGV(TAG, "message")` - Verbose

**Tag Convention**: `static const char* TAG = "MODULE_NAME";`

### 6. Configuration (`lib/config/config.h`)

**Purpose**: Centralized hardware pin definitions and feature flags

**Key Definitions**:
- I2C buses: `I2C_SDA`, `I2C_SCL`, `I2C1_SDA`, `I2C1_SCL`
- SPI pins: `RADIO_SCLK_PIN`, `RADIO_MISO_PIN`, `RADIO_MOSI_PIN`, `RADIO_CS_PIN`
- PMU: `PMU_IRQ`, `PMU_WIRE_PORT`
- GPS: `GPS_RX_PIN`, `GPS_TX_PIN`, `GPS_EN_PIN`
- Display: `DISPLAY_ADDR`, `DISPLAY_MODEL`

## Initialization Flow

```
setup() {
  1. Serial.begin(115200)
  2. BoardUtility::printChipInfo()
  3. Wire1.begin() - PMU I2C bus
  4. PMUModule::initialize() ⚠️ CRITICAL - Powers all peripherals
  5. Wire.begin() - Display I2C bus (now powered)
  6. SPI.begin() - Radio SPI bus
  7. Create module instances (DisplayModule, etc.)
  8. xTaskCreatePinnedToCore(...) - Spawn FreeRTOS tasks
  9. Enter loop()
}

loop() {
  - Send display updates via queue
  - Read sensor data
  - Trigger LoRa TX
  - vTaskDelay() - Yield to tasks
}
```

## Hardware Resources & Dependencies

### I2C Buses

**Wire (I2C0)** - Pins: SDA=17, SCL=18
- Display (SH1106 @ 0x3C)

**Wire1 (I2C1)** - Pins: SDA=42, SCL=41
- PMU (AXP2101/AXP192 @ 0x34)
- PMU_IRQ on GPIO40

### SPI Buses

**SPI (HSPI)** - Radio
- Pins: MOSI=11, MISO=13, SCLK=12, CS=10
- Additional: RST=5, DIO1=1, BUSY=4
- Devices: LoRa Radio (SX1262/LR1121)

### UART

**Serial1** - GPS Module
- Pins: RX=9, TX=8
- Additional: EN=7 (power enable), PPS=6
- Baud: 9600
- Devices: L76K or UBlox GPS

### Critical Dependencies

**PMU MUST initialize first** - It powers:
- GPS (ALDO4: 3.3V)
- LoRa (ALDO3: 3.3V)
- Sensors (ALDO1/2: 3.3V)
- SD Card (BLDO1/2: 3.3V)
- Display I2C bus

## Task Communication

### Queues
- **Display Queue**: `DisplayCommand` structs (10 deep)
- **LoRa TX Queue**: `LoRaPacket` structs (10 deep) - Future
- **LoRa RX Queue**: `LoRaPacket` structs (10 deep) - Future

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
- Use `ESP_LOG*()` for thread-safe logging
- Avoid shared state; prefer message passing

### Power Management

- PMU controls all peripheral power rails
- Call `pmu->disablePeripherals()` before deep sleep
- Use `esp_sleep_enable_*()` for wakeup sources

## Future Extensions

- GPS Task with NMEA parsing (TinyGPS++)
- LoRa Task full implementation (RadioLib)
- SD Logging Task with async writes
- Event Groups for system state (GPS_LOCKED, LORA_READY)
- Deep sleep coordination

## Dev Notes

Do not abuse comments. Code should be self-documenting where possible.
