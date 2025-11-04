#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lwip/sockets.h" // Para la API de sockets
#include "motor_control.h" // Para llamar a set_all_motors_to_angles
#include "tcp_server.h"  // Nuestra propia cabecera

#define PORT 1234 // El puerto que usará el cliente de Python
static const char *TAG = "tcp_server";

/**
 * @brief Tarea de FreeRTOS que maneja el servidor TCP.
 * (Implementación de la función declarada en tcp_server.h)
 */
void tcp_server_task(void *pvParameters) {
    char addr_str[128];
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;
    struct sockaddr_in dest_addr;

    // 1. Configurar la dirección del servidor (escuchar en cualquier IP)
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);

    // 2. Crear el socket de escucha
    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Error al crear el socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    // 3. Bind (enlazar) el socket al puerto
    if (bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) != 0) {
        ESP_LOGE(TAG, "Error en bind: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    // 4. Listen (poner el socket en modo escucha)
    if (listen(listen_sock, 1) != 0) {
        ESP_LOGE(TAG, "Error en listen: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Servidor TCP escuchando en el puerto %d", PORT);

    // Bucle principal para aceptar clientes
    while (1) {
        ESP_LOGI(TAG, "Esperando conexión de cliente...");
        struct sockaddr_in source_addr;
        socklen_t addr_len = sizeof(source_addr);

        // 5. Accept (bloquea la tarea hasta que un cliente se conecta)
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Error en accept: errno %d", errno);
            break; // Salir del bucle while(1) y terminar la tarea
        }

        // Cliente conectado
        inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
        ESP_LOGI(TAG, "Cliente conectado desde: %s", addr_str);

        int len;
        // Buffer para 3 ángulos de 16 bits (6 bytes)
        uint16_t angulos_raw[3];

        // 6. Bucle de recepción de datos (mientras el cliente esté conectado)
        do {
            // Recibir exactamente el tamaño del buffer (6 bytes)
            len = recv(sock, angulos_raw, sizeof(angulos_raw), 0);
            if (len < 0) {
                ESP_LOGE(TAG, "Error en recv: errno %d", errno);
                break;
            } else if (len == 0) {
                ESP_LOGW(TAG, "Conexión cerrada por el cliente");
            } else {
                // Decodificar los ángulos
                // ntohs = Network To Host Short
                // Convierte de Big-Endian (red) a Little-Endian (ESP32)
                uint16_t angle_a_int = ntohs(angulos_raw[0]);
                uint16_t angle_b_int = ntohs(angulos_raw[1]);
                uint16_t angle_c_int = ntohs(angulos_raw[2]);
                
                // Convertir a float y llamar a la función de control
                set_all_motors_to_angles((float)angle_a_int, (float)angle_b_int, (float)angle_c_int);
            }
        } while (len > 0);

        // 7. Cerrar la conexión con el cliente actual
        shutdown(sock, 0);
        close(sock);
    }

    // 8. Cerrar el socket de escucha (si salimos del bucle while)
    close(listen_sock);
    vTaskDelete(NULL);
}