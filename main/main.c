#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_log.h"
#include <stdio.h>
#include <math.h> // Necesario para fabs (comparación de floats)
#include <float.h> // Necesario para FLT_EPSILON

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Incluimos las cabeceras de nuestros módulos
#include "wifi_ap.h"
#include "web_server.h"
#include "motor_control.h"
#include "tcp_server.h"

static const char *TAG = "app_main";

// --- MEJORA: Variables globales encapsuladas o mejor gestionadas ---
// Considera agrupar estos parámetros en un struct para pasarlos a los módulos
// en lugar de tenerlos como globales si el proyecto crece.
float PID_KP = 2.5f;
float PID_KI = 0.1f;
float PID_KD = 0.05f;
int PID_MAX_OUTPUT = 64;
int PID_MIN_OUTPUT = -64;

// --- PROTOTIPOS DE FUNCIONES LOCALES ---
void print_startup_banner(void);

// =================================================================================
// TAREA DE MONITOREO DE ÁNGULOS (MODIFICADA)
// =================================================================================
/**
 * @brief Tarea que imprime los ángulos de los motores ÚNICAMENTE si han cambiado.
 */
void print_angles_on_change_task(void *arg)
{
    // --- MEJORA: Almacenar el estado anterior de los ángulos ---
    // Usamos 'static' para que las variables conserven su valor entre llamadas.
    // Inicializamos con un valor improbable para forzar la primera impresión.
    static float last_angle_m0 = -999.0f;
    static float last_angle_m1 = -999.0f;
    static float last_angle_m2 = -999.0f;

    while (1)
    {
        // 1. Obtener los ángulos actuales
        float current_angle_m0 = motor_get_current_angle(MOTOR_0);
        float current_angle_m1 = motor_get_current_angle(MOTOR_1);
        float current_angle_m2 = motor_get_current_angle(MOTOR_2);

        // 2. --- MEJORA: Comparar con los valores anteriores ---
        // Comparamos si la diferencia absoluta es mayor a un umbral pequeño (epsilon).
        // Esto evita impresiones por ruido mínimo en la lectura del flotante.
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

        // La tarea sigue ejecutándose y verificando constantemente
        vTaskDelay(pdMS_TO_TICKS(100)); // Reducimos el delay para mayor reactividad
    }
}

// =================================================================================
// FUNCIONES DE CONTROL (MODIFICADAS PARA MEJOR FEEDBACK)
// =================================================================================

/**
 * @brief Calibra un motor específico moviéndolo a su posición 0.
 */
void calibrate(int motor_n)
{
    if (motor_n >= MOTOR_0 && motor_n <= MOTOR_2)
    {
        // --- MEJORA: Mensaje de acción claro ---
        printf("\n[CALIBRACIÓN] Iniciando calibración del motor %d a 0.0°...\n", motor_n);
        motor_move_relative(motor_n, 0.0);
        vTaskDelay(pdMS_TO_TICKS(800)); // Espera a que el movimiento se complete
        printf("[CALIBRACIÓN] Motor %d calibrado.\n", motor_n);
    }
    else
    {
        // Usamos ESP_LOGE para errores del sistema que no deberían ocurrir
        ESP_LOGE(TAG, "Se intentó calibrar un motor no válido (%d).", motor_n);
    }
}

void set_all_motors_to_angles(float angle_a, float angle_b, float angle_c) {
    // Esta función recibe los ángulos absolutos y los aplica.
    // **NOTA IMPORTANTE:** Tu función se llama `motor_move_relative`.
    // Si realmente mueve el motor una cantidad 'relativa', esta lógica es incorrecta.
    // Sin embargo, para un robot balancín, casi siempre se necesita mover a un
    // ángulo ABSOLUTO. Asumimos que `motor_move_relative` en realidad establece
    // una posición absoluta.
    
    ESP_LOGI("MOTOR_CONTROL", "Moviendo a -> A:%.1f, B:%.1f, C:%.1f", angle_a, angle_b, angle_c);
    
    // --- Para angle_a ---
    if (angle_a < 0.0f) {
        angle_a = 0.0f;
    } else if (angle_a > 45.0f) {
        angle_a = 45.0f;
    }

    // --- Para angle_b ---
    if (angle_b < 0.0f) {
        angle_b = 0.0f;
    } else if (angle_b > 45.0f) {
        angle_b = 45.0f;
    }

    // --- Para angle_c ---
    if (angle_c < 0.0f) {
        angle_c = 0.0f;
    } else if (angle_c > 45.0f) {
        angle_c = 45.0f;
    }
    motor_move_relative(MOTOR_0, angle_a/2);
    motor_move_relative(MOTOR_1, -angle_b);
    motor_move_relative(MOTOR_2, -angle_c);
}

/**
 * @brief Ejecuta una secuencia de demostración de movimiento.
 */
void demo(void)
{
    const float demo_angle = -30.0f;
    const int total_cycles = 3;

    // --- MEJORA: Encabezado claro para la secuencia ---
    printf("\n\n╔══════════════════════════════════╗\n");
    printf("║      INICIANDO MODO DEMO       ║\n");
    printf("╚══════════════════════════════════╝\n\n");

    for (int i = 0; i < total_cycles; i++)
    {
        printf("===== CICLO %d de %d =====\n", i + 1, total_cycles);

        printf("[DEMO] Moviendo M0 -> %.1f°\n", demo_angle);
        motor_move_relative(MOTOR_0, -demo_angle);
        vTaskDelay(pdMS_TO_TICKS(500));

        printf("[DEMO] Moviendo M1 -> %.1f°\n", demo_angle);
        motor_move_relative(MOTOR_1, demo_angle);
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
    printf("║      MODO DEMO FINALIZADO      ║\n");
    printf("╚══════════════════════════════════╝\n\n");
}


// =================================================================================
// FUNCIÓN PRINCIPAL (MODIFICADA)
// =================================================================================
void app_main(void)
{
    // --- MEJORA: Secuencia de inicio limpia y visual ---
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
    wifi_init_softap(); // Asumimos que esta función imprime el SSID y la IP
    printf("[4/5] WiFi en modo Access Point iniciado.\n");

    // 5. Servidor Web
    start_webserver();
    printf("[5/5] Servidor web arrancado.\n");

    // --- MEJORA: Mensaje de confirmación final ---
    printf("\n===================================================\n");
    printf("✅  Sistema listo para operar.\n");
    printf("   Puede conectar al WiFi y acceder al servidor.\n");
    printf("===================================================\n\n");

    // Crear la tarea de monitoreo DESPUÉS de que todo esté listo.
    xTaskCreatePinnedToCore(
        print_angles_on_change_task,
        "Angle Monitor",
        2048,
        NULL,
        2,
        NULL,
        1
    );
    xTaskCreatePinnedToCore(
        tcp_server_task,
        "TCP Server",
        4096, // Dale un poco más de memoria para tareas de red
        NULL,
        5,    // Prioridad
        NULL,
        1     // Fíjalo al núcleo 1
    );
}

/**
 * @brief Imprime un banner visual al iniciar el dispositivo.
 */
void print_startup_banner(void)
{
    printf("\n");
    printf("**************************************************\n");
    printf("* *\n");
    printf("* PROYECTO DE CONTROL DE MOTORES MODULAR      *\n");
    printf("* *\n");
    printf("**************************************************\n");
    printf("\n");
}