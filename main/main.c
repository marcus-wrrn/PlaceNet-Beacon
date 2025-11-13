#include <stdio.h>
#include "utilities.h"
#include "u8g2.h"
#include "display_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char* TAG = "MAIN";

void app_main(void) {
    ESP_LOGI(TAG, "Initializing display");
    disp_init(I2C_SDA, I2C_SCL, 0x3C << 1);
    ESP_LOGI(TAG, "Setting font");
    disp_set_font(u8g2_font_boutique_bitmap_9x9_bold_te);
    disp_draw_str(10, 20, "Hello Bobby");
    ESP_LOGI(TAG, "Sending buffer");
    disp_draw_buffer();
    vTaskDelay(pdMS_TO_TICKS(1000));
    disp_draw_str(10, 29, "How are you?");
    disp_draw_buffer();

    vTaskDelete(NULL);
}