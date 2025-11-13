#include <stdio.h>
#include "display_driver.h"
#include "u8g2.h"
#include "u8g2_esp32_hal.h"

static u8g2_t u8g2;

void disp_clear_display(void) {
    u8g2_ClearDisplay(&u8g2);
}

void disp_draw_str(uint16_t x, uint16_t y, const char* str) {
    u8g2_DrawStr(&u8g2, x, y, str);
}

void disp_draw_buffer(void) {
    u8g2_SendBuffer(&u8g2);
}

void disp_clear_buffer(void) {
    u8g2_ClearBuffer(&u8g2);
}

void disp_set_font(const uint8_t* font) {
    u8g2_SetFont(&u8g2, font);
}

void disp_init(gpio_num_t sda_pin, gpio_num_t scl_pin, uint8_t address) {
    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
    u8g2_esp32_hal.bus.i2c.sda = sda_pin;
    u8g2_esp32_hal.bus.i2c.scl = scl_pin;

    u8g2_esp32_hal_init(u8g2_esp32_hal);
    u8g2_Setup_sh1106_i2c_128x64_noname_f(
        &u8g2, U8G2_R0,
        u8g2_esp32_i2c_byte_cb,
        u8g2_esp32_gpio_and_delay_cb
    );

    u8x8_SetI2CAddress(&u8g2.u8x8, address);
    u8g2_InitDisplay(&u8g2);
    disp_clear_display();
    u8g2_SetPowerSave(&u8g2, 0);
    disp_clear_buffer();
}

