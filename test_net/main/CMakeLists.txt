idf_component_register(SRCS main_v2.c lcd_printf.c
                       INCLUDE_DIRS .
                       PRIV_REQUIRES driver config net lcd)
# target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
