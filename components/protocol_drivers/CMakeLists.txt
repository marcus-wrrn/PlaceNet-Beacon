idf_component_register(SRCS "beacon_i2c_driver.c" "beacon_spi_driver.c" 
                    INCLUDE_DIRS "include"
                    REQUIRES freertos driver)