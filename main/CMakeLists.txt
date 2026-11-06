idf_component_register(SRCS "main.c" "knob_ui.c" "knob_fonts.c" "honda_emblem.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_gpio esp_driver_i2c esp_driver_ledc
                                esp_driver_spi esp_driver_uart esp_lcd esp_timer)
