idf_component_register(
    SRCS
        "src/foc_control.c"
        "src/foc_output.c"
        "src/foc_loop.c"
        "src/foc_driver_esp32.c"
    INCLUDE_DIRS "include"
    REQUIRES driver nvs_flash sensors
)
