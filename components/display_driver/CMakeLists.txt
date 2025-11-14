idf_component_register(SRCS "display_driver.c"
                    INCLUDE_DIRS "include"
                    REQUIRES u8g2 driver freertos u8g2_hal-esp-idf protocol_drivers)
