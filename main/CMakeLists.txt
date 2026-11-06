idf_component_register(SRCS "main.c" "knob_link.c" "knob_ui.c" "knob_fonts.c" "honda_emblem.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_gpio esp_driver_i2c esp_driver_ledc
                                esp_driver_spi esp_driver_uart esp_lcd esp_timer
                                esp_wifi esp_netif esp_event nvs_flash lwip esp_app_format
                                esp_https_ota app_update esp_http_client)
