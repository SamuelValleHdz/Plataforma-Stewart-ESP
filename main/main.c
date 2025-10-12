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
    // 1. Inicializa todo el sistema de motores
    motor_control_init();
    
    // 2. Crea una tarea para monitorear los ángulos
    xTaskCreatePinnedToCore(print_angles_task, "Print Angles", 2048, NULL, 2, NULL, 1);
    
    printf("App Main: Iniciando secuencia de movimiento...\n");
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // 3. Bucle para repetir la secuencia 5 veces
    for (int i = 0; i < 5; i++) {
        printf("========== Iniciando Ciclo %d de 5 ==========\n", i + 1);
        
        // Mueve cada motor 30° secuencialmente con una pausa de 800 ms
        printf("Moviendo motor 0...\n");
        motor_move_relative(MOTOR_0, 30.0);
        vTaskDelay(pdMS_TO_TICKS(800));
        
        printf("Moviendo motor 1...\n");
        motor_move_relative(MOTOR_1, 30.0);
        vTaskDelay(pdMS_TO_TICKS(800));
        
        printf("Moviendo motor 2...\n");
        motor_move_relative(MOTOR_2, 30.0);
        vTaskDelay(pdMS_TO_TICKS(800));
        
        // Pausa opcional antes de regresar
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // Regresar a la posición inicial de la misma forma secuencial
        printf("Regresando a la posición inicial...\n");
        
        motor_move_relative(MOTOR_2, -30.0);
        vTaskDelay(pdMS_TO_TICKS(800));
        
        motor_move_relative(MOTOR_1, -30.0);
        vTaskDelay(pdMS_TO_TICKS(800));
        
        motor_move_relative(MOTOR_0, -30.0);
        vTaskDelay(pdMS_TO_TICKS(800));

        printf("========== Ciclo %d Finalizado ==========\n\n", i + 1);
        vTaskDelay(pdMS_TO_TICKS(1000)); // Espera 1 segundo antes de empezar el siguiente ciclo
    }
    
    // Detener explícitamente los motores al finalizar todos los ciclos
    motor_stop(MOTOR_0);
    motor_stop(MOTOR_1);
    motor_stop(MOTOR_2);
    
    printf("App Main: Secuencia de prueba finalizada.\n");
}