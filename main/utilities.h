#ifndef __UTILITIES__
#define __UTILITIES__
#include <driver/gpio.h>
// #ifndef T_BEAM_S3_SUPREME
// #define T_BEAM_S3_SUPREME
// #endif

// #if   defined(T_BEAM_S3_SUPREME_SX1262)
// #ifndef USING_SX1262
// #define USING_SX1262
// #endif
// #elif defined(T_BEAM_S3_SUPREME_LR1121)
// #ifndef USING_LR1121
// #define USING_LR1121
// #endif
// #endif

#if defined(BOARD_T_BEAM)
#define USING_SX1262
#define I2C_SDA                     GPIO_NUM_17
#define I2C_SCL                     GPIO_NUM_18

#define I2C1_SDA                    GPIO_NUM_42
#define I2C1_SCL                    GPIO_NUM_41
#define PMU_IRQ                     GPIO_NUM_40

#define GPS_RX_PIN                  GPIO_NUM_9
#define GPS_TX_PIN                  GPIO_NUM_8
#define GPS_EN_PIN                  GPIO_NUM_7
#define GPS_PPS_PIN                 GPIO_NUM_6

#define BUTTON_PIN                  GPIO_NUM_0
#define BUTTON_PIN_MASK             0
#define BUTTON_COUNT                GPIO_NUM_1
#define BUTTON_ARRAY                {BUTTON_PIN}

#define RADIO_SCLK_PIN              GPIO_NUM_12
#define RADIO_MISO_PIN              GPIO_NUM_13
#define RADIO_MOSI_PIN         BOARD_T_BEAM     GPIO_NUM_11
#define RADIO_CS_PIN                GPIO_NUM_10
#define RADIO_DIO0_PIN              -1
#define RADIO_RST_PIN               GPIO_NUM_5
#define RADIO_DIO1_PIN              GPIO_NUM_1
#define RADIO_BUSY_PIN              GPIO_NUM_4

// TODO: Dynamically assign display model
#define DISPLAY_MODEL               U8G2_SH1106_128X64_NONAME_F_HW_I2C    

#elif defined(BOARD_T_DECK)
#define BOARD_POWERON       10

#define BOARD_I2S_WS        5
#define BOARD_I2S_BCK       7
#define BOARD_I2S_DOUT      6

#define BOARD_I2C_SDA       18
#define BOARD_I2C_SCL       8

#define BOARD_BAT_ADC       4

#define BOARD_TOUCH_INT     16
#define BOARD_KEYBOARD_INT  46

#define BOARD_SDCARD_CS     39
#define BOARD_TFT_CS        12
#define RADIO_CS_PIN        9

#define BOARD_TFT_DC        11
#define BOARD_TFT_BACKLIGHT 42

#define BOARD_SPI_MOSI      41
#define BOARD_SPI_MISO      38
#define BOARD_SPI_SCK       40

#define BOARD_TBOX_G02      2
#define BOARD_TBOX_G01      3
#define BOARD_TBOX_G04      1
#define BOARD_TBOX_G03      15

#define BOARD_ES7210_MCLK   48
#define BOARD_ES7210_LRCK   21
#define BOARD_ES7210_SCK    47
#define BOARD_ES7210_DIN    14

#define RADIO_BUSY_PIN      13
#define RADIO_RST_PIN       17
#define RADIO_DIO1_PIN      45

#define BOARD_BOOT_PIN      0

#define BOARD_BL_PIN        42


#define BOARD_GPS_TX_PIN                 43
#define BOARD_GPS_RX_PIN                 44


#ifndef RADIO_FREQ
#define RADIO_FREQ           868.0
#endif

#ifndef RADIO_BANDWIDTH
#define RADIO_BANDWIDTH      125.0
#endif

#ifndef RADIO_SF
#define RADIO_SF             10
#endif

#ifndef RADIO_CR
#define RADIO_CR             6
#endif

#ifndef RADIO_TX_POWER
#define RADIO_TX_POWER       22
#endif

#define DEFAULT_OPA          100

#endif

#endif