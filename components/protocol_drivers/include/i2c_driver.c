#include "i2c_driver.h"
#include "esp_log.h"

static const char* TAG = "I2C_DRIVER";
static i2c_master_bus_handle_t bus_handle;

esp_err_t prot_driver_i2c_init(gpio_num_t sda_pin, gpio_num_t scl_pin, i2c_port_num_t i2c_num) {
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = i2c_num,
        .scl_io_num = scl_pin,
        .sda_io_num = sda_pin,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };

    esp_err_t err = i2c_new_master_bus(&i2c_bus_config, &bus_handle);
    if (err != ESP_OK) {
        // ESP_LOGE(TAG, "Unable to initialize I2C bus");
        return err;
    }
    return err;
}

esp_err_t prot_driver_i2c_add_device(i2c_master_dev_handle_t* dev_handle, i2c_device_config_t* dev_cfg) {
    if (bus_handle == NULL) {
        // 
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = i2c_master_bus_add_device(&bus_handle, dev_cfg, dev_handle);
    if (err != ESP_OK) {
        // ESP_LOGE(TAG, "Unable to add device");
        return err;
    }

    return err;
}

// esp_err_t prot_driver_i2c_cleanup(void) {
    
// }