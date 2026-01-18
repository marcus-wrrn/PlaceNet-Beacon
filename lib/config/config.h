#pragma once

// Supported Boards:
// - T-Beam S3 Supreme (8MB Flash 8MB PSRAM)
// - T-Deck (16MB Flash 8MB PSRAM)

// Supported radio modules:
// - T_BEAM_S3_SUPREME_SX1262
// - T_BEAM_S3_SUPREME_LR1121
// - T_DECK_SX1262

#define UNUSED_PIN                   (0)

//=====================================================
// T-Beam S3 Supreme Configuration
//=====================================================
#if defined(T_BEAM_S3_SUPREME_SX1262) || defined(T_BEAM_S3_SUPREME_LR1121)

// Radio Type Selection
#if defined(T_BEAM_S3_SUPREME_SX1262)
#ifndef USING_SX1262
#define USING_SX1262
#endif
#elif defined(T_BEAM_S3_SUPREME_LR1121)
#ifndef USING_LR1121
#define USING_LR1121
#endif
#endif

#define ENABLE_BLE

// I2C Configuration
#define I2C_SDA                     (17)
#define I2C_SCL                     (18)

#define I2C1_SDA                    (42)
#define I2C1_SCL                    (41)
#define PMU_IRQ                     (40)

// GPS Configuration
#define GPS_RX_PIN                  (9)
#define GPS_TX_PIN                  (8)
#define GPS_EN_PIN                  (7)
#define GPS_PPS_PIN                 (6)
#define GPS_BAUD_RATE               (9600)

// Button Configuration
#define BUTTON_PIN                  (0)
#define BUTTON_PIN_MASK             (GPIO_SEL_0)
#define BUTTON_COUNT                (1)
#define BUTTON_ARRAY                {BUTTON_PIN}

// Radio SPI Configuration
#define RADIO_SCLK_PIN              (12)
#define RADIO_MISO_PIN              (13)
#define RADIO_MOSI_PIN              (11)
#define RADIO_CS_PIN                (10)
#define RADIO_DIO0_PIN              (-1)
#define RADIO_RST_PIN               (5)
#define RADIO_DIO1_PIN              (1)
#define RADIO_BUSY_PIN              (4)

// LR1121 Specific
#define RADIO_DIO9_PIN              (1)

// SD Card SPI Configuration
#define SPI_MOSI                    (35)
#define SPI_SCK                     (36)
#define SPI_MISO                    (37)
#define SPI_CS                      (47)

#define SDCARD_MOSI                 SPI_MOSI
#define SDCARD_MISO                 SPI_MISO
#define SDCARD_SCLK                 SPI_SCK
#define SDCARD_CS                   SPI_CS

// IMU Configuration
#define IMU_CS                      (34)
#define IMU_INT                     (33)

// RTC Configuration
#define RTC_INT                     (14)

// Feature Flags
#define HAS_SDCARD
#define HAS_GPS
#define HAS_DISPLAY
#define HAS_PMU
#define HAS_BLE
#define __HAS_SPI1__
#define HAS_SENSOR

// Display Configuration
#define PMU_WIRE_PORT               Wire1
#define DISPLAY_MODEL               U8G2_SH1106_128X64_NONAME_F_HW_I2C
#define DISPLAY_MODEL_SSD_LIB       SH1106Wire
#define DISPLAY_ADDR                0x3C

// Board Identification
#define BOARD_VARIANT_NAME          "T-Beam S3"

// Radio Type String
#if defined(USING_SX1262)
#define RADIO_TYPE_STR              "SX1262"
#define CONFIG_RADIO_FREQ           915.1
#define CONFIG_RADIO_OUTPUT_POWER   22
#define CONFIG_RADIO_BW             125.0

#elif defined(USING_LR1121)
#define RADIO_TYPE_STR              "LR1121"
#define CONFIG_RADIO_FREQ           868.0
#define CONFIG_RADIO_OUTPUT_POWER   22
#define CONFIG_RADIO_BW             125.0
#endif

#define LORA_TX_QUEUE_LEN 10
#define LORA_RX_QUEUE_LEN 10

// LoRa Operation Mode (choose one)
#define LORA_MODE_RECEIVER     // Listen for packets and display them
//#define LORA_MODE_BEACON    // Transmit beacon URL periodically

//=====================================================
// T-Deck Configuration
//=====================================================
#elif defined(T_DECK_SX1262)

#ifndef USING_SX1262
#define USING_SX1262
#endif

// Board Power Control (CRITICAL - must be HIGH to enable peripherals)
#define BOARD_POWERON_PIN           (10)

// I2C Configuration (single bus for keyboard)
#define I2C_SDA                     (18)
#define I2C_SCL                     (8)

// GPS Configuration
#define GPS_RX_PIN                  (44)
#define GPS_TX_PIN                  (43)
#define GPS_BAUD_RATE               (9600)

// Button Configuration
#define BUTTON_PIN                  (0)
#define BUTTON_PIN_MASK             (GPIO_SEL_0)
#define BUTTON_COUNT                (1)
#define BUTTON_ARRAY                {BUTTON_PIN}

// Radio SPI Configuration (shared SPI bus)
#define RADIO_SCLK_PIN              (40)
#define RADIO_MISO_PIN              (38)
#define RADIO_MOSI_PIN              (41)
#define RADIO_CS_PIN                (9)
#define RADIO_RST_PIN               (17)
#define RADIO_DIO1_PIN              (45)
#define RADIO_BUSY_PIN              (13)
#define RADIO_DIO0_PIN              (-1)

// LR1121 Specific (for future expansion)
#define RADIO_DIO9_PIN              (45)

// SD Card SPI Configuration (same SPI bus)
#define SPI_MOSI                    (41)
#define SPI_SCK                     (40)
#define SPI_MISO                    (38)
#define SPI_CS                      (39)

#define SDCARD_MOSI                 SPI_MOSI
#define SDCARD_MISO                 SPI_MISO
#define SDCARD_SCLK                 SPI_SCK
#define SDCARD_CS                   SPI_CS

// Battery Monitoring
#define BATTERY_ADC_PIN             (4)

// Display Configuration (ST7789 SPI - NOT IMPLEMENTED YET)
// #define BOARD_TFT_CS                (12)
// #define BOARD_TFT_DC                (11)
// #define BOARD_TFT_BACKLIGHT         (42)

// Keyboard (I2C)
#define KEYBOARD_INT_PIN            (46)

// Trackball
#define TRACKBALL_UP_PIN            (3)
#define TRACKBALL_DOWN_PIN          (15)
#define TRACKBALL_LEFT_PIN          (1)
#define TRACKBALL_RIGHT_PIN         (2)
#define TRACKBALL_CLICK_PIN         (0)  // Shared with boot button

// Touch Interrupt
#define TOUCH_INT_PIN               (16)

// Feature Flags
#define HAS_SDCARD
#define HAS_GPS
// #define HAS_DISPLAY              // Skip for now - different driver needed
// T-Deck has NO PMU - do not define HAS_PMU
#define HAS_BLE
#define HAS_BATTERY_ADC
#define HAS_KEYBOARD
#define HAS_TRACKBALL

// Board Identification
#define BOARD_VARIANT_NAME          "T-Deck"

// Radio Configuration
#define RADIO_TYPE_STR              "SX1262"
#define CONFIG_RADIO_FREQ           868.0
#define CONFIG_RADIO_OUTPUT_POWER   22
#define CONFIG_RADIO_BW             125.0

#define LORA_TX_QUEUE_LEN           10
#define LORA_RX_QUEUE_LEN           10

// LoRa Operation Mode (choose one)
#define LORA_MODE_RECEIVER     // Listen for packets and display them
// #define LORA_MODE_BEACON    // Transmit beacon URL periodically

#endif