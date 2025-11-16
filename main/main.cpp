#include <stdio.h>
#include <string>
#include "utilities.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "RadioLib.h"
#include "RadioLibESP_HAL.h"

extern "C" {
#include "display_driver.h"
#include "beacon_i2c_driver.h"
#include "beacon_spi_driver.h"
}

#if defined(USING_SX1262)
#ifndef CONFIG_RADIO_FREQ
#define CONFIG_RADIO_FREQ           915.0
#endif
#ifndef CONFIG_RADIO_OUTPUT_POWER
#define CONFIG_RADIO_OUTPUT_POWER   22
#endif
#ifndef CONFIG_RADIO_BW
#define CONFIG_RADIO_BW             125.0
#endif
// TODO enter other radio IC's here
#endif

static const char* TAG = "MAIN";
static volatile bool received_flag = false;
static const char rssi[16] = "0dBm";
static const char snr[16] = "0dB";
static const char payload[256] = "0";

esp_err_t setup_gpio() {


    esp_err_t err;

    // gpio_config_t io_conf_out = {};
    // io_conf_out.mode = GPIO_MODE_OUTPUT;
    // io_conf_out.pin_bit_mask = (1ULL << RADIO_DIO0_PIN);

    // err = gpio_config(&io_conf_out);
    // if (err != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to configure DIO0 as OUTPUT");
    //     return err;
    // }

    gpio_config_t io_conf_in = {};
    io_conf_in.mode = GPIO_MODE_INPUT;
    io_conf_in.pin_bit_mask = (1ULL << RADIO_DIO1_PIN);

    // Optional: enable pull-up or pull-down
    io_conf_in.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf_in.pull_down_en = GPIO_PULLDOWN_DISABLE;

    err = gpio_config(&io_conf_in);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure DIO2 as INPUT");
        return err;
    }

    return ESP_OK;
}

void set_flag(void) {
    received_flag = true;
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Initializing GPIO");
    esp_err_t err = setup_gpio();
    if (err != ESP_OK) {
        vTaskDelete(NULL);
    }

    // err = beacon_spi_driver_init(RADIO_MISO_PIN, RADIO_MOSI_PIN, RADIO_SCLK_PIN);
    // if (err != ESP_OK) {
    //     vTaskDelete(NULL);
    // }

    vTaskDelay(pdMS_TO_TICKS(1500));

    ESP_LOGI(TAG, "Initializing Radio HAL");
    // SX126x initialization
    RadioLibEspHal radio_hal = RadioLibEspHal(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN);
    ESP_LOGI(TAG, "Initializing Radio module");
    SX1262 radio_module = new Module(&radio_hal, RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);
    int state = radio_module.begin();

    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "Unable to initialize radio code: %d", state);
        vTaskDelete(NULL);
    }

    radio_module.setPacketReceivedAction(set_flag);

    if (radio_module.setFrequency(CONFIG_RADIO_FREQ) == RADIOLIB_ERR_INVALID_FREQUENCY) {
        ESP_LOGE(TAG, "Invalid Radio frequency");
        vTaskDelete(NULL);
    }

    if (radio_module.setBandwidth(CONFIG_RADIO_BW) == RADIOLIB_ERR_INVALID_BANDWIDTH) {
        ESP_LOGE(TAG, "Selected bandwidth is invalid");
        vTaskDelete(NULL);
    }

    if (radio_module.setSpreadingFactor(12) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR) {
        ESP_LOGE(TAG, "Invalid spreading factor");
        vTaskDelete(NULL);
    }

    if (radio_module.setCodingRate(6) == RADIOLIB_ERR_INVALID_CODING_RATE) {
        ESP_LOGE(TAG, "Invalid coding rate");
        vTaskDelete(NULL);
    }

    if (radio_module.setSyncWord(0xAB) != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "Unable to set sync word");
        vTaskDelete(NULL);
    }

    if (radio_module.setOutputPower(CONFIG_RADIO_OUTPUT_POWER) == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
        ESP_LOGE(TAG, "Invalid power output parameter");
        vTaskDelete(NULL);
    }


    ESP_LOGI(TAG, "Initializing I2C");
    beacon_i2c_driver_init(I2C_SDA, I2C_SCL, I2C_NUM_1);
    // ESP_LOGI(TAG, "Initializing SPI");
    // beacon_spi_driver_init(RADIO_MISO_PIN, RADIO_MOSI_PIN, RADIO_SCLK_PIN);

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

    ESP_LOGI(TAG, "Done: Entering super loop");

    while(1) {
        if (received_flag) {
            disp_clear_buffer();
            disp_draw_str(0, 10, "Received Packet!");
            disp_send_buffer();
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}