#include "beacon_spi_driver.h"
#include "esp_log.h"

static const char* TAG = "SPI_DRIVER";
#define HOST SPI1_HOST
#define CHANNEL SPI_DMA_CH_AUTO

esp_err_t beacon_spi_init(gpio_num_t miso_pin, gpio_num_t mosi_pin, gpio_num_t sclk_pin) {
    spi_bus_config_t bus_config = {
        .sclk_io_num = sclk_pin,
        .mosi_io_num = mosi_pin,
        .miso_io_num = miso_pin,
        .quadhd_io_num = GPIO_NUM_NC,
        .quadwp_io_num = GPIO_NUM_NC
    };

    esp_err_t err = spi_bus_initialize(HOST, &bus_config, CHANNEL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Unable to initialize SPI bus");
    }
    return err;
}

esp_err_t beacon_spi_add_device(spi_device_handle_t* dev_handle, spi_device_interface_config_t* dev_config) {
    esp_err_t err = spi_bus_add_device(HOST, dev_config, dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Unable to add device to spi bus");
    }
    return err;
}