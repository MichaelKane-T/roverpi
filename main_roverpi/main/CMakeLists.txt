idf_component_register(SRCS main.c
                       INCLUDE_DIRS .
                       PRIV_REQUIRES driver config esp_timer)
# PRIV_REQUIRES ...
# target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
