idf_component_register(
    SRCS "src/jitter_buffer.c"
    INCLUDE_DIRS "include"
    REQUIRES freertos esp_event
)
