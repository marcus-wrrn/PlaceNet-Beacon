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
#define RADIO_MOSI_PIN              GPIO_NUM_11
#define RADIO_CS_PIN                GPIO_NUM_10
#define RADIO_DIO0_PIN              -1
#define RADIO_RST_PIN               GPIO_NUM_5
#define RADIO_DIO1_PIN              GPIO_NUM_1
#define RADIO_BUSY_PIN              GPIO_NUM_4

#define DISPLAY_MODEL               U8G2_SH1106_128X64_NONAME_F_HW_I2C    

#endif