idf_component_register(SRCS "motor_control.c" "main.c" "encoder.c" "blink.c"
                    INCLUDE_DIRS "."
                    )

# VERSIÓN CORREGIDA
#idf_component_register(SRCS "main.cpp" "encoder.c" "blink.c"
#                       INCLUDE_DIRS ".")