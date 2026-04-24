idf_component_register(SRCS net.c
                       INCLUDE_DIRS .
                       PRIV_REQUIRES esp_wifi esp_ringbuf nvs_flash)
# target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
