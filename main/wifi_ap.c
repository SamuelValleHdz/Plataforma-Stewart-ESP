#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include <string.h>
#include "lwip/err.h"
#include "lwip/ip_addr.h"
#include "lwip/sys.h"
#include "esp_netif.h"

// --- Configuración de la Red Wi-Fi ---
#define EXAMPLE_ESP_WIFI_SSID      "Robot"
#define EXAMPLE_ESP_WIFI_PASS      "MTR09A_2022"
#define EXAMPLE_MAX_STA_CONN        2

// Etiqueta para los mensajes de log
static const char *TAG = "wifi_AP";

// Manejador de eventos Wi-Fi (para ver cuándo se conecta/desconecta un cliente)
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "Dispositivo "MACSTR" se ha conectado, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "Dispositivo "MACSTR" se ha desconectado, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

// Función principal para inicializar el Punto de Acceso
void wifi_init_softap(void)
{
    // 1. Crear la interfaz de red (netif) para el AP
    esp_netif_t *p_netif_ap = esp_netif_create_default_wifi_ap();

    // ====================================================================
    // 2. CONFIGURACIÓN DE LA DIRECCIÓN IP FIJA
    // ====================================================================
    ESP_LOGI(TAG, "Configurando IP estática para el Punto de Acceso...");
    
    // Detenemos el servidor DHCP para poder cambiar la IP
    ESP_ERROR_CHECK(esp_netif_dhcps_stop(p_netif_ap));

    // Definimos la información de la IP
    esp_netif_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip, 192, 168, 10, 1);       // <-- IP del ESP32
    IP4_ADDR(&ip_info.gw, 192, 168, 10, 1);       // <-- Gateway (la misma que la IP)
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0); // <-- Máscara de subred

    // Aplicamos la configuración de IP estática a la interfaz del AP
    ESP_ERROR_CHECK(esp_netif_set_ip_info(p_netif_ap, &ip_info));

    // Volvemos a iniciar el servidor DHCP para que asigne IPs a los clientes
    ESP_ERROR_CHECK(esp_netif_dhcps_start(p_netif_ap));
    
    ESP_LOGI(TAG, "IP estática configurada: %d.%d.%d.%d", IP2STR(&ip_info.ip));
    // ====================================================================

    // 3. Inicializar el stack de Wi-Fi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 4. Registrar nuestro manejador de eventos Wi-Fi
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    // 5. Configurar los parámetros del AP (SSID, contraseña, etc.)
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
            .password = EXAMPLE_ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    // 6. Aplicar la configuración y arrancar el Wi-Fi en modo AP
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Punto de Acceso Wi-Fi iniciado. SSID: %s | Contraseña: %s",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
}