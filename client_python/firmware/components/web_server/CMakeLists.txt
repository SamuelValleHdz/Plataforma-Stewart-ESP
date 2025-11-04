idf_component_register(SRCS "web_server.c"
                     INCLUDE_DIRS "."
                     REQUIRES esp_http_server wifi_ap tcp_server motor_control)
target_add_binary_data(
    ${COMPONENT_TARGET}
    "web/index.html" TEXT
)

target_add_binary_data(
    ${COMPONENT_TARGET}
    "web/style.css" TEXT
)