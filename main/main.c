#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_log.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Incluimos las cabeceras de nuestros módulos
#include "wifi_ap.h"
#include "web_server.h"
#include "motor_control.h"

static const char *TAG = "app_main";

// Tarea para imprimir el estado de los motores, usando la API pública
void print_angles_task(void *arg)
{
    while (1)
    {
        // Imprime los ángulos actuales de los tres motores
        printf("Angulos -> M0: %.2f | M1: %.2f | M2: %.2f\n",
               motor_get_current_angle(MOTOR_0),
               motor_get_current_angle(MOTOR_1),
               motor_get_current_angle(MOTOR_2));
        vTaskDelay(pdMS_TO_TICKS(500)); // Actualizar cada 500ms
    }
}

float PID_KP = 2.5f;
float PID_KI = 0.1f;
float PID_KD = 0.05f;
int PID_MAX_OUTPUT = 255;
int PID_MIN_OUTPUT = -255;

/**
 * @brief Actualiza todos los parámetros del controlador PID desde una sola llamada.
 * @param kp Nueva ganancia Proporcional.
 * @param ki Nueva ganancia Integral.
 * @param kd Nueva ganancia Derivativa.
 * @param max_out Nuevo límite máximo de salida.
 * @param min_out Nuevo límite mínimo de salida.
 */
void update_all_pid_parameters(float kp, float ki, float kd, int limit_percent)
{
    // --- Lógica de conversión de porcentaje a valor absoluto ---
    // 1. Calcular el valor absoluto basado en el máximo de 255
    float absolute_value = ((float)limit_percent / 100.0f) * 255.0f;

    // 2. Asignar los valores a las variables globales (convirtiendo a entero)
    PID_KP = kp;
    PID_KI = ki;
    PID_KD = kd;
    PID_MAX_OUTPUT = (int)absolute_value;
    PID_MIN_OUTPUT = -((int)absolute_value); // El valor negativo

    // Mensaje de confirmación mostrando el valor calculado
    printf("PID actualizado -> KP:%.2f, KI:%.2f, KD:%.2f\n", PID_KP, PID_KI, PID_KD);
    printf("Límite: %d%% -> MAX: %d, MIN: %d\n", limit_percent, PID_MAX_OUTPUT, PID_MIN_OUTPUT);
}

void calibrate(int motor_n)
{
    printf("Moviendo motor 0...\n");
    motor_move_relative(MOTOR_0, 0);
    vTaskDelay(pdMS_TO_TICKS(800));

    printf("Moviendo motor 1...\n");
    motor_move_relative(MOTOR_1, 0.0);
    vTaskDelay(pdMS_TO_TICKS(800));

    printf("Moviendo motor 2...\n");
    motor_move_relative(MOTOR_2, 0.0);
    vTaskDelay(pdMS_TO_TICKS(800));
}

void demo(void)
{
    for (int i = 0; i < 5; i++)
    {
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
}

void app_main(void)
{
    printf("Valores PID iniciales -> MAX: %d, MIN: %d\n", PID_MAX_OUTPUT, PID_MIN_OUTPUT);

    // 1. Inicializa todo el sistema de motores
    motor_control_init();

    // 2. Crea una tarea para monitorear los ángulos
    xTaskCreatePinnedToCore(print_angles_task, "Print Angles", 2048, NULL, 2, NULL, 1);

    printf("App Main: Iniciando secuencia de movimiento...\n");
    vTaskDelay(pdMS_TO_TICKS(100));

    // Inicializaciones de siempre
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Inicia los módulos
    wifi_init_softap();
    start_webserver(); // El servidor ahora vive en su propio archivo

    ESP_LOGI(TAG, "Inicialización completada.");
}
