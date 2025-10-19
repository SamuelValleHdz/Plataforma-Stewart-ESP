// ==========================================================
//  tcp_server.c
// ==========================================================
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "motor_control.h" // Para llamar a la función que mueve los motores

#define PORT 1234 // El puerto que usará Python

static const char *TAG = "tcp_server";

void tcp_server_task(void *pvParameters) {
    char addr_str[128];
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;
    struct sockaddr_in dest_addr;

    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Error al crear el socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    if (bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) != 0) {
        ESP_LOGE(TAG, "Error en bind: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    if (listen(listen_sock, 1) != 0) {
        ESP_LOGE(TAG, "Error en listen: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Servidor TCP escuchando en el puerto %d", PORT);

    while (1) {
        struct sockaddr_in source_addr;
        socklen_t addr_len = sizeof(source_addr); // <<< CAMBIO AQUÍ: de 'uint' a 'socklen_t'
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Error en accept: errno %d", errno);
            break;
        }

        inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
        ESP_LOGI(TAG, "Cliente conectado desde: %s", addr_str);

        int len;
        uint16_t angulos[3];

        do {
            len = recv(sock, angulos, sizeof(angulos), 0);
            if (len < 0) {
                ESP_LOGE(TAG, "Error en recv: errno %d", errno);
                break;
            } else if (len == 0) {
                ESP_LOGW(TAG, "Conexión cerrada por el cliente");
            } else {
                uint16_t angle_a = ntohs(angulos[0]);
                uint16_t angle_b = ntohs(angulos[1]);
                uint16_t angle_c = ntohs(angulos[2]);
                
                set_all_motors_to_angles((float)angle_a, (float)angle_b, (float)angle_c);
            }
        } while (len > 0);

        shutdown(sock, 0);
        close(sock);
    }
    close(listen_sock);
    vTaskDelete(NULL);
}