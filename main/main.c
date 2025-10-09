#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor_control.h"

// Tarea para imprimir el estado de los motores, usando la API pública
void print_angles_task(void *arg) {
    while (1) {
        // Imprime los ángulos actuales de los tres motores
        printf("Angulos -> M0: %.2f | M1: %.2f | M2: %.2f\n",
               motor_get_current_angle(MOTOR_0),
               motor_get_current_angle(MOTOR_1),
               motor_get_current_angle(MOTOR_2));
        vTaskDelay(pdMS_TO_TICKS(500)); // Actualizar cada 500ms
    }
}

void app_main(void) {
    // 1. Inicializa todo el sistema de motores con una sola llamada
    motor_control_init();
    
    // 2. Crea tarea de monitoreo que imprime ángulos periódicamente
    xTaskCreatePinnedToCore(print_angles_task, "Print Angles", 2048, NULL, 2, NULL, 1);
    
    printf("App Main: Iniciando secuencia de movimiento...\n");
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // 3. Controla los motores usando la API simple y clara
    // Mover los 3 motores simultáneamente
    motor_move_relative(MOTOR_0, 90.0);    // Mover motor 0 90 grados
    motor_move_relative(MOTOR_1, -180.0);  // Mover motor 1 media vuelta en reversa
    motor_move_relative(MOTOR_2, 90.0);    // Mover motor 2 a 90 grados
    vTaskDelay(pdMS_TO_TICKS(800));        // Esperar a que terminen
    
    // Moverlos de vuelta a su posición original
    printf("App Main: Regresando a la posición inicial...\n");
    motor_move_relative(MOTOR_0, -90.0);
    motor_move_relative(MOTOR_1, 180.0);
    motor_move_relative(MOTOR_2, -90.0);
    vTaskDelay(pdMS_TO_TICKS(800));
    
    // Detener explícitamente los motores al finalizar
    motor_stop(MOTOR_0);
    motor_stop(MOTOR_1);
    motor_stop(MOTOR_2);
    
    printf("App Main: Secuencia de prueba finalizada.\n");
}