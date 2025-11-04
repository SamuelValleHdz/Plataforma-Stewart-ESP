#ifndef WIFI_AP_H
#define WIFI_AP_H

/**
 * @brief Inicializa el sistema WiFi en modo Access Point (AP).
 *
 * Configura el SSID, contraseña y la IP estática (192.168.10.1)
 * usando las definiciones de este mismo archivo.
 */
void wifi_init_softap(void);


// --- Configuración de la Red Wi-Fi ---
#define EXAMPLE_ESP_WIFI_SSID      "Robot"
#define EXAMPLE_ESP_WIFI_PASS      "MTR09A_2022"
#define EXAMPLE_MAX_STA_CONN        3


#endif // WIFI_AP_H