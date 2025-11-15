#ifndef __BEACON_SPI_DRIVER__
#define __BEACON_SPI_DRIVER__
#include "driver/spi_master.h"
#include "driver/gpio.h"

esp_err_t beacon_spi_driver_init(gpio_num_t miso_pin, gpio_num_t mosi_pin, gpio_num_t sclk_pin);
esp_err_t beacon_spi_driver_add_device(spi_device_handle_t* dev_handle, spi_device_interface_config_t* dev_config);

#endif