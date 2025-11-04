idf_component_register(SRCS "main.c"
                     INCLUDE_DIRS "."
                     REQUIRES motor_control tcp_server wifi_ap web_server nvs_flash)
#idf_component_register(SRCS "tcp_server.c" "main.c" "web_server.c" "wifi_ap.c" "motor_control.c"
#                    INCLUDE_DIRS ".")
#
#target_add_binary_data(
#    ${COMPONENT_TARGET}
#    "web/index.html" TEXT
#)
#
#target_add_binary_data(
#    ${COMPONENT_TARGET}
#    "web/style.css" TEXT
#)
