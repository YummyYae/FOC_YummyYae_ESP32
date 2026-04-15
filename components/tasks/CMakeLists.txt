idf_component_register(
    SRCS
        "src/foc_task.c"
        "src/control_task.c"
        "src/led_pov_task.c"
        "src/ap_task.c"
    INCLUDE_DIRS "include"
    REQUIRES foc sensors esp_timer algorithm esp_driver_gptimer esp_wifi nvs_flash esp_netif esp_event lwip
)
