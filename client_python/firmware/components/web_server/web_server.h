#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include "esp_http_server.h"

/**
 * @brief Inicia el servidor web HTTP y registra todas las URIs.
 *
 * @return El handle (manejador) del servidor httpd, o NULL si falla.
 */
httpd_handle_t start_webserver(void);

/**
 * @brief Detiene el servidor web.
 *
 * @param server El handle del servidor httpd obtenido de start_webserver.
 */
void stop_webserver(httpd_handle_t server);

#endif // WEB_SERVER_H