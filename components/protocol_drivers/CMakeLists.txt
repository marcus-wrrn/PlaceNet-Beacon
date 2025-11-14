idf_component_register(SRCS "protocol_drivers.c" "i2c_driver.c"
                    INCLUDE_DIRS "include"
                    REQUIRES freertos driver)