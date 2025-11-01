<<<<<<< HEAD
idf_component_register(SRCS "tcp_server.c" "main.c" "web_server.c" "wifi_ap.c" "motor_control.c"
                    INCLUDE_DIRS ".")

target_add_binary_data(
    ${COMPONENT_TARGET}
    "web/index.html" TEXT
)

target_add_binary_data(
    ${COMPONENT_TARGET}
    "web/style.css" TEXT
)
=======
idf_component_register(SRCS "motor_control.c" "main.c"
                    INCLUDE_DIRS "."
                    )

# VERSIÓN CORREGIDA
#idf_component_register(SRCS "main.cpp" "encoder.c" "blink.c"
#                       INCLUDE_DIRS ".")
>>>>>>> base/master
