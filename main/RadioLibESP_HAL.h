#ifndef ___RADIOLIB_ESP_HAL_H___
#define ___RADIOLIB_ESP_HAL_H___
// Software used to make RadioLib compatible with esp-idf
#include <RadioLib.h>
#if CONFIG_IDF_TARGET_ESP32 == 0 && CONFIG_IDF_TARGET_ESP32S3 == 0
  #error This HAL only supports ESP32 targets
#endif
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if CONFIG_IDF_TARGET_ESP32
#include "esp32/rom/gpio.h"
#elif CONFIG_IDF_TARGET_ESP32S3
#include "esp32s3/rom/gpio.h"
// #include "esp_clk_tree.h"
#else

#endif

#include "soc/rtc.h"
#include "soc/dport_reg.h"
#include "soc/spi_reg.h"
#include "soc/spi_struct.h"
#include "driver/gpio.h"
#include "hal/gpio_hal.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "soc/rtc.h"


// define Arduino-style macros
#define LOW                         (0x0)
#define HIGH                        (0x1)
#define INPUT                       (0x01)
#define OUTPUT                      (0x03)
#define RISING                      (0x01)
#define FALLING                     (0x02)
#define NOP()                       asm volatile ("nop")


class RadioLibEspHal : public RadioLibHal {
  public:
    // default constructor - initializes the base HAL and any needed private members
    RadioLibEspHal(int8_t sck, int8_t miso, int8_t mosi)
      : RadioLibHal(INPUT, OUTPUT, LOW, HIGH, RISING, FALLING),
      spiSCK(sck), spiMISO(miso), spiMOSI(mosi)  {
    }

    void init() override {
      spiBegin();
    }

    void term() override {
      spiEnd();
    }

    // GPIO-related methods (pinMode, digitalWrite etc.) should check
    // RADIOLIB_NC as an alias for non-connected pins
    void pinMode(uint32_t pin, uint32_t mode) override {
      if(pin == RADIOLIB_NC) {
        return;
      }

      gpio_hal_context_t gpiohal;
      gpiohal.dev = GPIO_LL_GET_HW(GPIO_PORT_0);

      gpio_config_t conf = {
        .pin_bit_mask = (1ULL<<pin),
        .mode = (gpio_mode_t)mode,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = (gpio_int_type_t)gpiohal.dev->pin[pin].int_type,
      };
      gpio_config(&conf);
    }

    void digitalWrite(uint32_t pin, uint32_t value) override {
      if(pin == RADIOLIB_NC) {
        return;
      }

      gpio_set_level((gpio_num_t)pin, value);
    }

    uint32_t digitalRead(uint32_t pin) override {
      if(pin == RADIOLIB_NC) {
        return(0);
      }

      return(gpio_get_level((gpio_num_t)pin));
    }

    void attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void), uint32_t mode) override {
      if(interruptNum == RADIOLIB_NC) {
        return;
      }

      gpio_install_isr_service((int)ESP_INTR_FLAG_IRAM);
      gpio_set_intr_type((gpio_num_t)interruptNum, (gpio_int_type_t)(mode & 0x7));

      // this uses function typecasting, which is not defined when the functions have different signatures
      // untested and might not work
      gpio_isr_handler_add((gpio_num_t)interruptNum, (void (*)(void*))interruptCb, NULL);
    }

    void detachInterrupt(uint32_t interruptNum) override {
      if(interruptNum == RADIOLIB_NC) {
        return;
      }

      gpio_isr_handler_remove((gpio_num_t)interruptNum);
	    gpio_wakeup_disable((gpio_num_t)interruptNum);
      gpio_set_intr_type((gpio_num_t)interruptNum, GPIO_INTR_DISABLE);
    }

    void delay(unsigned long ms) override {
      vTaskDelay(ms / portTICK_PERIOD_MS);
    }

    void delayMicroseconds(unsigned long us) override {
      uint64_t m = (uint64_t)esp_timer_get_time();
      if(us) {
        uint64_t e = (m + us);
        if(m > e) { // overflow
          while((uint64_t)esp_timer_get_time() > e) {
            NOP();
          }
        }
        while((uint64_t)esp_timer_get_time() < e) {
          NOP();
        }
      }
    }

    unsigned long millis() override {
      return((unsigned long)(esp_timer_get_time() / 1000ULL));
    }

    unsigned long micros() override {
      return((unsigned long)(esp_timer_get_time()));
    }

    long pulseIn(uint32_t pin, uint32_t state, unsigned long timeout) override {
      if(pin == RADIOLIB_NC) {
        return(0);
      }

      this->pinMode(pin, INPUT);
      uint32_t start = this->micros();
      uint32_t curtick = this->micros();

      while(this->digitalRead(pin) == state) {
        if((this->micros() - curtick) > timeout) {
          return(0);
        }
      }

      return(this->micros() - start);
    }

    void spiBegin() {
      spi_bus_config_t buscfg = {};
      buscfg.mosi_io_num = this->spiMOSI;
      buscfg.miso_io_num = this->spiMISO;
      buscfg.sclk_io_num = this->spiSCK;
      buscfg.quadwp_io_num = -1;
      buscfg.quadhd_io_num = -1;

      spi_device_interface_config_t devcfg = {};
      devcfg.mode = 0;
      devcfg.clock_speed_hz = 2000000;
      devcfg.queue_size = 1;

      spi_bus_initialize(this->spi_host, &buscfg, SPI_DMA_CH_AUTO);
      spi_bus_add_device(this->spi_host, &devcfg, &this->spi_handle);
    }

    void spiBeginTransaction() {
      // Included for compatability with RadioLib
      return;
    }

    uint8_t spiTransferByte(uint8_t b) {
      spi_transaction_t t = {};
      t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
      t.length = 8;
      t.tx_data[0] = b;
      
      esp_err_t err = spi_device_transmit(this->spi_handle, &t);
      if (err != ESP_OK) {
        ESP_LOGE("ESP_HAL", "Unable to transfer byte: %d", b);
      }
      return t.rx_data[0];
    }

    void spiTransfer(uint8_t* out, size_t len, uint8_t* in) {
      for (uint8_t i = 0; i < len; i++) {
        in[i] = this->spiTransferByte(out[i]);
      }
    }

    void spiEndTransaction() {
      return;
    }

    void spiEnd() {
      spi_bus_remove_device(this->spi_handle);
      spi_bus_free(this->spi_host);
    }

  private:
    int8_t spiSCK;
    int8_t spiMISO;
    int8_t spiMOSI;
    spi_host_device_t spi_host = SPI2_HOST;
    spi_device_handle_t spi_handle;
    spi_dev_t * spi = (volatile spi_dev_t *)(DR_REG_SPI2_BASE);
};

#endif