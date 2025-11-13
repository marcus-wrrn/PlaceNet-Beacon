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
    disp_init_i2c(I2C_SDA, I2C_SCL, 0x3C << 1);
    ESP_LOGI(TAG, "Sending String");
    disp_draw_str(0, 10, "Hello World");
    ESP_LOGI(TAG, "Drawing screen");
    disp_send_buffer();
    disp_draw_str(0, 19, "Test");
    disp_send_buffer();

    vTaskDelete(NULL);
}