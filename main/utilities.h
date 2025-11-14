#ifndef __UTILITIES__
#define __UTILITIES__
#include <driver/gpio.h>

#if defined(BOARD_T_BEAM)
    #define USING_SX1262
    
    // I2C Configuration
    #define I2C_SDA                     GPIO_NUM_17
    #define I2C_SCL                     GPIO_NUM_18
    #define I2C1_SDA                    GPIO_NUM_42
    #define I2C1_SCL                    GPIO_NUM_41
    
    // Power Management
    #define PMU_IRQ                     GPIO_NUM_40
    
    // GPS Configuration
    #define GPS_RX_PIN                  GPIO_NUM_9
    #define GPS_TX_PIN                  GPIO_NUM_8
    #define GPS_EN_PIN                  GPIO_NUM_7
    #define GPS_PPS_PIN                 GPIO_NUM_6
    
    // Button Configuration
    #define BUTTON_PIN                  GPIO_NUM_0
    #define BUTTON_PIN_MASK             0
    #define BUTTON_COUNT                GPIO_NUM_1
    #define BUTTON_ARRAY                {BUTTON_PIN}
    
    // SPI/Radio Configuration
    #define RADIO_SCLK_PIN              GPIO_NUM_12
    #define RADIO_MISO_PIN              GPIO_NUM_13
    #define RADIO_MOSI_PIN              GPIO_NUM_11
    #define RADIO_CS_PIN                GPIO_NUM_10
    #define RADIO_DIO0_PIN              -1
    #define RADIO_RST_PIN               GPIO_NUM_5
    #define RADIO_DIO1_PIN              GPIO_NUM_1
    #define RADIO_BUSY_PIN              GPIO_NUM_4
    
    // Display Configuration
    #define DISPLAY_MODEL               U8G2_SH1106_128X64_NONAME_F_HW_I2C
    
#elif defined(BOARD_T_DECK)
    #define USING_SX1262
    
    // Power Configuration
    #define BOARD_POWERON               GPIO_NUM_10
    
    // I2C Configuration
    #define I2C_SDA                     GPIO_NUM_18
    #define I2C_SCL                     GPIO_NUM_8
    
    // I2S Audio Configuration
    #define HAS_I2S
    #define I2S_WS                      GPIO_NUM_5
    #define I2S_BCK                     GPIO_NUM_7
    #define I2S_DOUT                    GPIO_NUM_6
    
    // ES7210 Audio Codec
    #define ES7210_MCLK                 GPIO_NUM_48
    #define ES7210_LRCK                 GPIO_NUM_21
    #define ES7210_SCK                  GPIO_NUM_47
    #define ES7210_DIN                  GPIO_NUM_14
    
    // Battery & Power Monitoring
    #define BAT_ADC_PIN                 GPIO_NUM_4
    
    // Interrupt Pins
    #define TOUCH_INT_PIN               GPIO_NUM_16
    #define KEYBOARD_INT_PIN            GPIO_NUM_46
    
    // SPI Configuration
    #define RADIO_SCLK_PIN              GPIO_NUM_40
    #define RADIO_MISO_PIN              GPIO_NUM_38
    #define RADIO_MOSI_PIN              GPIO_NUM_41
    #define RADIO_CS_PIN                GPIO_NUM_9
    #define SDCARD_CS_PIN               GPIO_NUM_39
    
    // Radio Configuration
    #define RADIO_BUSY_PIN              GPIO_NUM_13
    #define RADIO_RST_PIN               GPIO_NUM_17
    #define RADIO_DIO1_PIN              GPIO_NUM_45
    
    // Display/TFT Configuration
    #define TFT_CS_PIN                  GPIO_NUM_12
    #define TFT_DC_PIN                  GPIO_NUM_11
    #define TFT_BACKLIGHT_PIN           GPIO_NUM_42
    
    // GPS Configuration
    #define GPS_TX_PIN                  GPIO_NUM_43
    #define GPS_RX_PIN                  GPIO_NUM_44
    
    // Trackball/Expansion Pins
    #define TBOX_G01                    GPIO_NUM_3
    #define TBOX_G02                    GPIO_NUM_2
    #define TBOX_G03                    GPIO_NUM_15
    #define TBOX_G04                    GPIO_NUM_1
    
    // Button Configuration
    #define BUTTON_PIN                  GPIO_NUM_0
    
    // Radio Default Settings
    #ifndef RADIO_FREQ
    #define RADIO_FREQ                  868.0
    #endif
    #ifndef RADIO_BANDWIDTH
    #define RADIO_BANDWIDTH             125.0
    #endif
    #ifndef RADIO_SF
    #define RADIO_SF                    10
    #endif
    #ifndef RADIO_CR
    #define RADIO_CR                    6
    #endif
    #ifndef RADIO_TX_POWER
    #define RADIO_TX_POWER              22
    #endif
    #define DEFAULT_OPA                 100
    
#else
    #error "No board type defined! Use BOARD_T_BEAM or BOARD_T_DECK"
#endif

#endif // __UTILITIES__