idf_component_register(SRCS 
                          "main.c"
                          "motor_control.c"
                          "distance_sensor.c"
                          "roverpi_fsm.c"
                          "serial_uart.c"
                       PRIV_REQUIRES driver config esp_timer esp_driver_uart)
# PRIV_REQUIRES ...
# target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
