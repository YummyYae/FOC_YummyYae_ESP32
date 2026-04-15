idf_component_register(
    SRCS
        "src/foc_task.c"
        "src/control_task.c"
    INCLUDE_DIRS "include"
    REQUIRES foc sensors esp_timer algorithm
)
