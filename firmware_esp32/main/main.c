#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_log.h"
#include <stdio.h>
#include <math.h>   // Para fabs (comparación de floats)
#include <float.h>  // Para FLT_EPSILON

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// --- Módulos del Proyecto ---
#include "wifi_ap.h"
#include "web_server.h"
#include "motor_control.h"
#include "tcp_server.h"

static const char *TAG = "app_main";

// --- Parámetros Globales de Control ---
// (Se declaran aquí y se usan en motor_control.c)
float PID_KP = 2.5f;
float PID_KI = 0.1f;
float PID_KD = 0.05f;
int PID_MAX_OUTPUT = 115;  // Límite inicial (se puede cambiar desde la web)
int PID_MIN_OUTPUT = -115; // Límite inicial

// --- Prototipos de Funciones Locales ---
void print_startup_banner(void);

// =================================================================================
// TAREA DE MONITOREO DE ÁNGULOS
// =================================================================================

/**
 * @brief Tarea de FreeRTOS que imprime los ángulos de los motores
 * únicamente si han cambiado.
 *
 * Compara el ángulo actual con el último valor impreso para evitar
 * spam en la consola.
 *
 * @param arg Parámetros de la tarea (no se usa).
 */
void print_angles_on_change_task(void *arg)
{
    // Almacena el estado anterior de los ángulos
    static float last_angle_m0 = -0.0f;
    static float last_angle_m1 = -0.0f;
    static float last_angle_m2 = -0.0f;

    while (1)
    {
        // 1. Obtener los ángulos actuales
        float current_angle_m0 = motor_get_current_angle(MOTOR_0);
        float current_angle_m1 = motor_get_current_angle(MOTOR_1);
        float current_angle_m2 = motor_get_current_angle(MOTOR_2);

        // 2. Comparar con los valores anteriores
        // Se usa FLT_EPSILON para una comparación segura de flotantes.
        if (fabs(current_angle_m0 - last_angle_m0) > FLT_EPSILON ||
            fabs(current_angle_m1 - last_angle_m1) > FLT_EPSILON ||
            fabs(current_angle_m2 - last_angle_m2) > FLT_EPSILON)
        {
            // Si algún ángulo cambió, imprimir la línea completa
            printf("Ángulos -> M0: %7.2f° | M1: %7.2f° | M2: %7.2f°\n",
                   current_angle_m0, current_angle_m1, current_angle_m2);

            // 3. Actualizar los últimos valores conocidos
            last_angle_m0 = current_angle_m0;
            last_angle_m1 = current_angle_m1;
            last_angle_m2 = current_angle_m2;
        }

        // Ejecutar la verificación periódicamente
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// =================================================================================
// FUNCIONES DE CONTROL (Implementaciones)
// =================================================================================

/**
 * @brief Calibra un motor específico moviéndolo a su posición 0.
 *
 * @param motor_n El número de motor (0, 1, o 2).
 */
void calibrate(int motor_n)
{
    if (motor_n >= MOTOR_0 && motor_n <= MOTOR_2)
    {
        printf("\n[CALIBRACIÓN] Iniciando calibración del motor %d a 0.0°...\n", motor_n);
        // Llama a la función del módulo motor_control
        motor_move_relative(motor_n, 0.0);
        vTaskDelay(pdMS_TO_TICKS(800)); // Espera a que el movimiento se complete
        printf("[CALIBRACIÓN] Motor %d calibrado.\n", motor_n);
    }
    else
    {
        ESP_LOGE(TAG, "Se intentó calibrar un motor no válido (%d).", motor_n);
    }
}

/**
 * @brief Recibe los ángulos (desde la web o TCP) y los aplica a los motores.
 *
 * Esta función también aplica un "clamp" o límite de seguridad
 * para que los ángulos no superen los 45 grados.
 *
 * @param angle_a Ángulo para MOTOR_0.
 * @param angle_b Ángulo para MOTOR_1.
 * @param angle_c Ángulo para MOTOR_2.
 */
void set_all_motors_to_angles(float angle_a, float angle_b, float angle_c)
{
    ESP_LOGI("MOTOR_CONTROL", "Moviendo a -> A:%.1f, B:%.1f, C:%.1f", angle_a, angle_b, angle_c);

    // --- Clamp de seguridad para angle_a ---
    if (angle_a < 0.0f) angle_a = 0.0f;
    else if (angle_a > 45.0f) angle_a = 45.0f;

    // --- Clamp de seguridad para angle_b ---
    if (angle_b < 0.0f) angle_b = 0.0f;
    else if (angle_b > 45.0f) angle_b = 45.0f;

    // --- Clamp de seguridad para angle_c ---
    if (angle_c < 0.0f) angle_c = 0.0f;
    else if (angle_c > 45.0f) angle_c = 45.0f;

    // Aplicar los ángulos a los motores
    // (Nota: La cinemática puede requerir ángulos negativos)
    motor_move_relative(MOTOR_0, angle_a);
    motor_move_relative(MOTOR_1, -angle_b/2);
    motor_move_relative(MOTOR_2, angle_c);
}

/**
 * @brief Ejecuta una secuencia de demostración de baile.
 */


void dance(void)
{
    const float dance_angle = 45.0f; // Ángulo de movimiento
    const int total_cycles = 5;       // Cuántas veces repite el baile

    printf("\n\n╔══════════════════════════════════╗\n");
    printf("║      INICIANDO BAILE             ║\n"); 
    printf("╚══════════════════════════════════╝\n\n");

    for (int i = 0; i < total_cycles; i++)
    {
        printf("--- Ciclo %d de %d ---\n", i + 1, total_cycles);

        /* --- PASO 1: Izquierda y Derecha (Alternado) --- */
        printf("/-(O_o)_/ Izquierda...\n"); 
        motor_move_relative(MOTOR_0, dance_angle);
        vTaskDelay(pdMS_TO_TICKS(400));

        printf("\\_(o_O)-\\ Derecha...\n");
        motor_move_relative(MOTOR_0, 0); // Asumo que 0 regresa a home o quita el offset
        vTaskDelay(pdMS_TO_TICKS(100));
        motor_move_relative(MOTOR_2, dance_angle);
        vTaskDelay(pdMS_TO_TICKS(400));
        
        motor_move_relative(MOTOR_2, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        /* --- PASO 2: Salto (Ambos Juntos) --- */
        printf("╚(°-°)╝ ¡ARRIBA!\n");
        motor_move_relative(MOTOR_0, dance_angle);
        motor_move_relative(MOTOR_2, dance_angle);
        vTaskDelay(pdMS_TO_TICKS(310)); // Pausa dramática

        printf("╔(°-°)╗ ¡ABAJO!\n");
        motor_move_relative(MOTOR_0, 0);
        motor_move_relative(MOTOR_2, 0);
        vTaskDelay(pdMS_TO_TICKS(310));

        printf("╚(°-°)╝ ¡ARRIBA!\n");
        motor_move_relative(MOTOR_0, dance_angle);
        motor_move_relative(MOTOR_2, dance_angle);
        vTaskDelay(pdMS_TO_TICKS(310)); // Pausa dramática

        printf("╔(°-°)╗ ¡ABAJO!\n");
        motor_move_relative(MOTOR_0, 0);
        motor_move_relative(MOTOR_2, 0);
        vTaskDelay(pdMS_TO_TICKS(310));
    }

    printf("\n╔══════════════════════════════════╗\n");
    printf("║      BAILE TERMINADO             ║\n");
    printf("╚══════════════════════════════════╝\n\n");
}


/**
 * @brief Ejecuta una secuencia de demostración de movimiento.
 */
void demo(void)
{
    const float demo_angle = 30.0f;
    const int total_cycles = 3;

    printf("\n\n╔══════════════════════════════════╗\n");
    printf("║      INICIANDO MODO DEMO         ║\n");
    printf("╚══════════════════════════════════╝\n\n");

    for (int i = 0; i < total_cycles; i++)
    {
        printf("===== CICLO %d de %d =====\n", i + 1, total_cycles);

        printf("[DEMO] Moviendo M0 -> %.1f°\n", demo_angle);
        motor_move_relative(MOTOR_0, demo_angle);
        vTaskDelay(pdMS_TO_TICKS(500));

        printf("[DEMO] Moviendo M1 -> %.1f°\n", demo_angle);
        motor_move_relative(MOTOR_1, -demo_angle);
        vTaskDelay(pdMS_TO_TICKS(500));

        printf("[DEMO] Moviendo M2 -> %.1f°\n", demo_angle);
        motor_move_relative(MOTOR_2, demo_angle);
        vTaskDelay(pdMS_TO_TICKS(500));

        vTaskDelay(pdMS_TO_TICKS(500));

        printf("[DEMO] Regresando a la posición inicial...\n");
        motor_move_relative(MOTOR_2, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
        motor_move_relative(MOTOR_1, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
        motor_move_relative(MOTOR_0, 0);
        vTaskDelay(pdMS_TO_TICKS(500));

        printf("===== CICLO %d FINALIZADO =====\n\n", i + 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    printf("\n╔══════════════════════════════════╗\n");
    printf("║      MODO DEMO FINALIZADO        ║\n");
    printf("╚══════════════════════════════════╝\n\n");
}

/**
 * @brief Imprime un banner visual al iniciar el dispositivo.
 */
void print_startup_banner(void)
{
    printf("\n");
    printf("**************************************************\n");
    printf("* *\n");
    printf("* PROYECTO DE CONTROL DE MOTORES MODULAR     *\n");
    printf("* *\n");
    printf("**************************************************\n");
    printf("\n");
}

// =================================================================================
// FUNCIÓN PRINCIPAL
// =================================================================================
void app_main(void)
{
    //dance();
    print_startup_banner();
    printf("Iniciando sistema...\n");

    // 1. Inicialización de NVS (necesaria para WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    printf("[1/5] Almacenamiento NVS inicializado.\n");

    // 2. Sistema de motores
    motor_control_init();
    printf("[2/5] Módulo de control de motores listo.\n");

    // 3. Inicialización de red (TCP/IP y Event Loop)
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    printf("[3/5] Pila de red TCP/IP inicializada.\n");

    // 4. Módulo WiFi AP
    wifi_init_softap(); // Imprime el SSID y la IP
    printf("[4/5] WiFi en modo Access Point iniciado.\n");

    // 5. Servidor Web
    start_webserver();
    printf("[5/5] Servidor web arrancado.\n");

    printf("\n===================================================\n");
    printf("✅  Sistema listo para operar.\n");
    printf("   Puede conectar al WiFi y acceder al servidor.\n");
    printf("===================================================\n\n");

    // --- Iniciar Tareas de fondo ---

    // Tarea para monitorear ángulos en consola
    xTaskCreatePinnedToCore(
        print_angles_on_change_task,
        "Angle Monitor",
        2048,
        NULL,
        2,  // Baja prioridad
        NULL,
        1   // Pin al núcleo 1 (el 0 es para WiFi/BT)
    );

    // Tarea para el servidor TCP de alta velocidad
    xTaskCreatePinnedToCore(
        tcp_server_task,
        "TCP Server",
        4096, // Más memoria para tareas de red
        NULL,
        5,    // Prioridad media
        NULL,
        1     // Pin al núcleo 1
    );
}