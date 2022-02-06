idf_component_register(SRCS "main.c" "ESP32CLI/esp32_uart_driver.c" "ESP32CLI/serial_driver.c" "FreeRTOS-Plus-CLI/FreeRTOS_CLI.c"
                    INCLUDE_DIRS "." "./FreeRTOS-Plus-CLI" "ESP32CLI")
