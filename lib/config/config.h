#pragma once

// T-Beam S3 Supreme Configuration
// Board: LilyGo T-Beam Supreme (8MB Flash 8MB PSRAM)
// Product: https://lilygo.cc/products/t-beam-supreme

// Supported radio modules:
// - T_BEAM_S3_SUPREME_SX1262
// - T_BEAM_S3_SUPREME_LR1121

#define UNUSED_PIN                   (0)

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
#define CONFIG_RADIO_FREQ           870.0
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