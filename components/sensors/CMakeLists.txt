idf_component_register(
    SRCS
        "src/mt6701.c"
        "src/tlc5947.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer
)
# keep this file updated so CMake regeneration picks up new sensor sources
