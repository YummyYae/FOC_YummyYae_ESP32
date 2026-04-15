idf_component_register(
    SRCS
        "src/foc_control.c"
        "src/foc_output.c"
        "src/foc_driver_esp32.c"
        "src/foc_debug.c"
    INCLUDE_DIRS "include"
    REQUIRES driver nvs_flash sensors esp_driver_mcpwm
)
