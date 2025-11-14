#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "u8g2_esp32_hal.h"
#include "protocol_drivers.h"

static const char* TAG = "u8g2_hal";
static const unsigned int I2C_TIMEOUT_MS = 1000;

static spi_device_handle_t spi_dev_handle;   // SPI handle.
static i2c_master_dev_handle_t i2c_dev_handle;

#define I2C_BUFFER_SIZE 256
static uint8_t i2c_buffer[I2C_BUFFER_SIZE];
static size_t i2c_buffer_len = 0;


static u8g2_esp32_hal_t u8g2_esp32_hal;  // HAL state data.

#define HOST    SPI2_HOST

#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)                   \
  do {                                       \
    esp_err_t rc = (x);                      \
    if (rc != ESP_OK) {                      \
      ESP_LOGE("err", "esp_err_t = %d", rc); \
      assert(0 && #x);                       \
    }                                        \
  } while (0);


/*
 * Initialze the ESP32 HAL.
 */
void u8g2_esp32_hal_init(u8g2_esp32_hal_t u8g2_esp32_hal_param) {
  u8g2_esp32_hal = u8g2_esp32_hal_param;
}  // u8g2_esp32_hal_init

/*
 * HAL callback function as prescribed by the U8G2 library.  This callback is
 * invoked to handle SPI communications.
 */
uint8_t u8g2_esp32_spi_byte_cb(u8x8_t* u8x8,
                               uint8_t msg,
                               uint8_t arg_int,
                               void* arg_ptr) {
  ESP_LOGD(TAG, "spi_byte_cb: Received a msg: %d, arg_int: %d, arg_ptr: %p",
           msg, arg_int, arg_ptr);
  switch (msg) {
    case U8X8_MSG_BYTE_SET_DC:
      if (u8g2_esp32_hal.dc != U8G2_ESP32_HAL_UNDEFINED) {
        gpio_set_level(u8g2_esp32_hal.dc, arg_int);
      }
      break;

    case U8X8_MSG_BYTE_INIT: {
      if (u8g2_esp32_hal.bus.spi.clk == U8G2_ESP32_HAL_UNDEFINED ||
          u8g2_esp32_hal.bus.spi.mosi == U8G2_ESP32_HAL_UNDEFINED ||
          u8g2_esp32_hal.bus.spi.cs == U8G2_ESP32_HAL_UNDEFINED) {
        break;
      }

      spi_bus_config_t bus_config = {0};
      bus_config.sclk_io_num = u8g2_esp32_hal.bus.spi.clk;   // CLK
      bus_config.mosi_io_num = u8g2_esp32_hal.bus.spi.mosi;  // MOSI
      bus_config.miso_io_num = GPIO_NUM_NC;                  // MISO
      bus_config.quadwp_io_num = GPIO_NUM_NC;                // Not used
      bus_config.quadhd_io_num = GPIO_NUM_NC;                // Not used
      // ESP_LOGI(TAG, "... Initializing bus.");
      ESP_ERROR_CHECK(spi_bus_initialize(HOST, &bus_config, SPI_DMA_CH_AUTO));

      spi_device_interface_config_t dev_config = {0};
      dev_config.address_bits = 0;
      dev_config.command_bits = 0;
      dev_config.dummy_bits = 0;
      dev_config.mode = 0;
      dev_config.duty_cycle_pos = 0;
      dev_config.cs_ena_posttrans = 0;
      dev_config.cs_ena_pretrans = 0;
      dev_config.clock_speed_hz = 10000;
      dev_config.spics_io_num = u8g2_esp32_hal.bus.spi.cs;
      dev_config.flags = 0;
      dev_config.queue_size = 200;
      dev_config.pre_cb = NULL;
      dev_config.post_cb = NULL;
      // ESP_LOGI(TAG, "... Adding device bus.");
      ESP_ERROR_CHECK(spi_bus_add_device(HOST, &dev_config, &spi_dev_handle));

      break;
    }

    case U8X8_MSG_BYTE_SEND: {
      spi_transaction_t trans_desc = {0};
      trans_desc.addr = 0;
      trans_desc.cmd = 0;
      trans_desc.flags = 0;
      trans_desc.length = 8 * arg_int;  // Number of bits NOT number of bytes.
      trans_desc.rxlength = 0;
      trans_desc.tx_buffer = arg_ptr;
      trans_desc.rx_buffer = NULL;
      // trans_desc.override_freq_hz = 0; // this param does not exist prior to ESP-IDF 5.5.0
      // ESP_LOGI(TAG, "... Transmitting %d bytes.", arg_int);
      ESP_ERROR_CHECK(spi_device_transmit(spi_dev_handle, &trans_desc));
      break;
    }
  }
  return 0;
}  // u8g2_esp32_spi_byte_cb


/*
 * HAL callback function as prescribed by the U8G2 library.  This callback is
 * invoked to handle I2C communications using the modern ESP-IDF v5.5 I2C driver.
 */
uint8_t u8g2_esp32_i2c_byte_cb(u8x8_t* u8x8,
                                uint8_t msg,
                                uint8_t arg_int,
                                void* arg_ptr) {
  ESP_LOGD(TAG, "i2c_cb: Received a msg: %d, arg_int: %d, arg_ptr: %p", msg,
           arg_int, arg_ptr);
  
  switch (msg) {
    case U8X8_MSG_BYTE_SET_DC: {
      if (u8g2_esp32_hal.dc != U8G2_ESP32_HAL_UNDEFINED) {
        gpio_set_level(u8g2_esp32_hal.dc, arg_int);
      }
      break;
    }
    
    case U8X8_MSG_BYTE_INIT: {
      ESP_LOGI(TAG, "Creating I2C device with address 0x3C");
      i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x3C,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
      };
      
      esp_err_t ret = prot_driver_i2c_add_device(&i2c_dev_handle, &dev_cfg);
      if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C device add failed: %s", esp_err_to_name(ret));
        ESP_ERROR_CHECK(ret);
      }
      
      ESP_LOGI(TAG, "I2C master bus initialized successfully");
      break;
    }
    
    case U8X8_MSG_BYTE_SEND: {
      uint8_t* data_ptr = (uint8_t*)arg_ptr;
      ESP_LOG_BUFFER_HEXDUMP(TAG, data_ptr, arg_int, ESP_LOG_VERBOSE);
      
      // Accumulate data in buffer
      if (i2c_buffer_len + arg_int <= I2C_BUFFER_SIZE) {
        memcpy(i2c_buffer + i2c_buffer_len, data_ptr, arg_int);
        i2c_buffer_len += arg_int;
      } else {
        ESP_LOGE(TAG, "I2C buffer overflow!");
      }
      break;
    }
    
    case U8X8_MSG_BYTE_START_TRANSFER: {
      ESP_LOGD(TAG, "Start I2C transfer to 0x3C");
      
      // Reset buffer for new transaction
      i2c_buffer_len = 0;
      break;
    }
    
    case U8X8_MSG_BYTE_END_TRANSFER: {
      ESP_LOGI(TAG, "End I2C transfer. Sending %d bytes", i2c_buffer_len);
      ESP_LOG_BUFFER_HEX(TAG, i2c_buffer, i2c_buffer_len);
      
      // Now send all accumulated data at once
      if (i2c_buffer_len > 0) {
        esp_err_t ret = i2c_master_transmit(i2c_dev_handle, i2c_buffer, 
                                            i2c_buffer_len,
                                            pdMS_TO_TICKS(I2C_TIMEOUT_MS));
        if (ret != ESP_OK) {
          ESP_LOGE(TAG, "I2C transmit failed: %s (len=%d)", esp_err_to_name(ret), i2c_buffer_len);
        } else {
          ESP_LOGI(TAG, "I2C transmit SUCCESS");
        }
        i2c_buffer_len = 0;
      }
      break;
    }
  }
  
  return 0;
}  // u8g2_esp32_i2c_byte_cb

/*
 * HAL callback function as prescribed by the U8G2 library.  This callback is
 * invoked to handle callbacks for GPIO and delay functions.
 */
uint8_t u8g2_esp32_gpio_and_delay_cb(u8x8_t* u8x8,
                                      uint8_t msg,
                                      uint8_t arg_int,
                                      void* arg_ptr) {
  ESP_LOGD(TAG,
           "gpio_and_delay_cb: Received a msg: %d, arg_int: %d, arg_ptr: %p",
           msg, arg_int, arg_ptr);
  
  switch (msg) {
    case U8X8_MSG_GPIO_AND_DELAY_INIT: {
      uint64_t bitmask = 0;
      if (u8g2_esp32_hal.dc != U8G2_ESP32_HAL_UNDEFINED) {
        bitmask = bitmask | (1ull << u8g2_esp32_hal.dc);
      }
      if (u8g2_esp32_hal.reset != U8G2_ESP32_HAL_UNDEFINED) {
        bitmask = bitmask | (1ull << u8g2_esp32_hal.reset);
      }
      if (u8g2_esp32_hal.bus.spi.cs != U8G2_ESP32_HAL_UNDEFINED) {
        bitmask = bitmask | (1ull << u8g2_esp32_hal.bus.spi.cs);
      }
      
      if (bitmask == 0) {
        break;
      }
      
      gpio_config_t gpioConfig = {
        .pin_bit_mask = bitmask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,  // Changed from ENABLE
        .intr_type = GPIO_INTR_DISABLE
      };
      gpio_config(&gpioConfig);
      break;
    }
    
    case U8X8_MSG_GPIO_RESET:
      if (u8g2_esp32_hal.reset != U8G2_ESP32_HAL_UNDEFINED) {
        gpio_set_level(u8g2_esp32_hal.reset, arg_int);
      }
      break;
    
    case U8X8_MSG_GPIO_CS:
      if (u8g2_esp32_hal.bus.spi.cs != U8G2_ESP32_HAL_UNDEFINED) {
        gpio_set_level(u8g2_esp32_hal.bus.spi.cs, arg_int);
      }
      break;
    
    // REMOVED software I2C cases - using hardware I2C
    
    case U8X8_MSG_DELAY_MILLI:
      vTaskDelay(pdMS_TO_TICKS(arg_int));
      break;
    
    case U8X8_MSG_DELAY_10MICRO:
      esp_rom_delay_us(arg_int * 10);
      break;
    
    case U8X8_MSG_DELAY_100NANO:
      // ESP32 can't reliably do 100ns delays, do 1us instead
      esp_rom_delay_us(1);
      break;
  }
  
  return 0;
}
