#ifndef __I2C_DRIVER__
#define __I2C_DRIVER__

#include "driver/i2c_master.h"
#include "driver/gpio.h"

esp_err_t prot_driver_i2c_init(gpio_num_t sda_pin, gpio_num_t scl_pin, i2c_port_num_t i2c_num);
esp_err_t prot_driver_i2c_add_device(i2c_master_dev_handle_t* dev_handle, i2c_device_config_t* dev_cfg);

#endif