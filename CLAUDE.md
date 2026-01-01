# PlaceNet-Beacon Project Documentation

## Overview

PlaceNet-Beacon is an ESP32-based LoRa beacon/tracker firmware designed for the **LilyGo T-Beam S3 Supreme** development board. It provides a foundation for building location-aware, long-range wireless communication devices suitable for IoT tracking, mesh networking, or beacon applications.

**Hardware Platform**: ESP32-S3 (dual-core, 240MHz, 8MB Flash, 8MB PSRAM)
**Build System**: PlatformIO with Arduino framework
**Primary Use Case**: GPS-enabled LoRa beacon with PMU

## Project Structure

```
PlaceNet-Beacon/
├── lib/                      # Custom libraries
│   ├── BoardManager/         # HAL
│   ├── PMUManager/           
│   ├── Logger/               
│   ├── LoraManager/          # LoRa radio manager
│   └── config/
│       └── config.h          # Pin definitions & configuration
├── src/
│   └── main.cpp              # Entry point
├── boards/
│   └── t-beams3-supreme.json # Custom PlatformIO board definition
├── platformio.ini            # Build configuration
└── CLAUDE.md                 # This file
```

## Architecture

### Design Patterns

- **Singleton Pattern**: `BoardManager` ensures single instance managing all hardware
- **Facade Pattern**: `BoardManager` provides unified interface to peripherals
- **Modular Design**: Each subsystem is a self-contained library
- **Interrupt-Driven**: PMU events handled via GPIO interrupts

## Core Components

### 1. BoardManager (`lib/BoardManager/`)

**Purpose**: Central HAL managing all peripherals

### 2. PMUManager (`lib/PMUManager/`)

**Purpose**: Manage AXP192/AXP2101 Power Management IC

### 3. Logger (`lib/Logger/`)

**Purpose**: ESP-IDF style logging system with color-coded output, Uses ESP-IDF like syntax

**Log Levels**:
- `ERROR`: Critical errors
- `WARN`: Warnings
- `INFO`: Informational messages
- `DEBUG`: Debug information
- `VERBOSE`: Detailed trace


### 4. LoraManager (`lib/LoraManager/`)

### 5. Configuration (`lib/config/config.h`)

**Purpose**: Centralized hardware pin definitions and feature flags

## Dev Notes

Do not abuse comments