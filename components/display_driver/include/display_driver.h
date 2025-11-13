#ifndef __DISPLAY_DRIVER__
#define __DISPLAY_DRIVER__
#include "driver/i2c_master.h"

/** Initializes Display
 * Must be run before using display
 */
void disp_init(gpio_num_t i2c_pin, gpio_num_t scl_pin, uint8_t address);

// Clears SH content
void disp_clear_display(void);
void disp_draw_str(uint16_t x, uint16_t y, const char* str);
void disp_draw_buffer(void);
void disp_clear_buffer(void);
void disp_set_font(const uint8_t* font);
#endif