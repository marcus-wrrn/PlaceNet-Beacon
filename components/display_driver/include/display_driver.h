#ifndef __DISPLAY_DRIVER__
#define __DISPLAY_DRIVER__
#include "driver/i2c_master.h"

/** Initializes Display
 * Must be run before using display
 */
void disp_init_i2c(gpio_num_t i2c_pin, gpio_num_t scl_pin, uint8_t address);
void disp_init_spi(gpio_num_t miso_pin, gpio_num_t mosi_pin, gpio_num_t sclk_pin, gpio_num_t cs_pin, gpio_num_t dc_pin);
// Clears SH content
void disp_clear_display(void);
void disp_draw_str(uint16_t x, uint16_t y, const char* str);
void disp_send_buffer(void);
void disp_clear_buffer(void);
void disp_set_font(const uint8_t* font);
#endif