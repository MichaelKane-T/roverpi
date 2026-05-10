idf_component_register(SRCS 
                          "main.c"
                          "gyroscope.c"
                          "motor_control.c"
                          "distance_sensor.c"
                          "roverpi_fsm.c"
                          "serial_uart.c"
                          "battery_monitor.c"
                       PRIV_REQUIRES driver config esp_timer esp_driver_uart esp_adc )
# PRIV_REQUIRES ...
# target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
