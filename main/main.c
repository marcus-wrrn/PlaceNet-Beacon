#include <stdio.h>
#include "utilities.h"
#include "u8g2.h"
#include "display_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "beacon_i2c_driver.h"
#include "beacon_spi_driver.h"

static const char* TAG = "MAIN";

void app_main(void) {
    ESP_LOGI(TAG, "Initializing I2C");
    beacon_i2c_driver_init(I2C_SDA, I2C_SCL, I2C_NUM_1);
    ESP_LOGI(TAG, "Initializing SPI");
    beacon_spi_driver_init(RADIO_MISO_PIN, RADIO_MOSI_PIN, RADIO_SCLK_PIN);

#ifdef BOARD_T_BEAM
    
    disp_init_i2c(I2C_SDA, I2C_SCL, 0x3C << 1);
#elif BOARD_T_DECK
    disp_init_spi(RADIO_MISO_PIN, RADIO_MOSI_PIN, RADIO_SCLK_PIN, TFT_CS_PIN, TFT_DC_PIN);
#endif

    
    ESP_LOGI(TAG, "Sending String");
    disp_draw_str(0, 10, "Hello World");
    ESP_LOGI(TAG, "Drawing screen");
    disp_send_buffer();
    disp_draw_str(0, 19, "Test");
    disp_send_buffer();

    ESP_LOGI(TAG, "Done");
    vTaskDelete(NULL);
}