#include "esp_http_server.h"
#include "esp_log.h"
#include <string.h>  // Para sscanf
#include <stdlib.h>  // Para atof, atoi
#include <stdio.h>

// Incluimos las cabeceras de nuestros propios módulos
#include "web_server.h"
#include "motor_control.h" // Para calibrate(), demo(), etc.

static const char *TAG = "web_server";

// --- Declaración de los archivos web incrustados ---
// (Estos se generan durante la compilación)
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");
extern const uint8_t style_css_start[]  asm("_binary_style_css_start");
extern const uint8_t style_css_end[]    asm("_binary_style_css_end");

// ==========================================================
// ## MANEJADORES PARA SERVIR ARCHIVOS ESTÁTICOS (HTML/CSS) ##
// ==========================================================

/**
 * @brief Manejador para GET /
 * Sirve el archivo index.html incrustado.
 * @param req Petición HTTP.
 * @return ESP_OK
 */
static esp_err_t root_get_handler(httpd_req_t *req) {
    const size_t html_size = (index_html_end - index_html_start);
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char *)index_html_start, html_size);
    return ESP_OK;
}

/**
 * @brief Manejador para GET /style.css
 * Sirve el archivo style.css incrustado.
 * @param req Petición HTTP.
 * @return ESP_OK
 */
static esp_err_t style_get_handler(httpd_req_t *req) {
    const size_t css_size = (style_css_end - style_css_start);
    httpd_resp_set_type(req, "text/css");
    httpd_resp_send(req, (const char *)style_css_start, css_size);
    return ESP_OK;
}

// ==========================================================
// ## MANEJADORES PARA COMANDOS (Endpoints de la API) ##
// ==========================================================

/**
 * @brief Manejador para POST /set_pid
 * Recibe los nuevos parámetros PID (kp, ki, kd, limit)
 * y los aplica a los motores.
 * @param req Petición HTTP (cuerpo: "kp=X&ki=Y&kd=Z&limit_percent=L").
 * @return ESP_OK si éxito, ESP_FAIL si error.
 */
static esp_err_t set_pid_handler(httpd_req_t *req) {
    char buf[128];
    int ret, remaining = req->content_len;

    if (remaining >= sizeof(buf)) {
        ESP_LOGE(TAG, "Petición PID demasiado larga");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Petición demasiado larga");
        return ESP_FAIL;
    }

    ret = httpd_req_recv(req, buf, remaining);
    if (ret <= 0) { // Error o conexión cerrada
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) httpd_resp_send_408(req);
        return ESP_FAIL;
    }
    buf[ret] = '\0'; // Asegurarse de que el string termina en nulo
    ESP_LOGI(TAG, "Datos PID recibidos: %s", buf);

    // Analizar los datos recibidos (ej: "kp=2.5&ki=0.1&kd=0.05&limit_percent=64")
    float new_kp, new_ki, new_kd;
    int new_limit_percent;
    int parsed_count = sscanf(buf, "kp=%f&ki=%f&kd=%f&limit_percent=%d",
                              &new_kp, &new_ki, &new_kd, &new_limit_percent);

    if (parsed_count == 4) {
        // Llamar a la función del módulo motor_control
        update_all_pid_parameters(new_kp, new_ki, new_kd, new_limit_percent);
        httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    } else {
        ESP_LOGE(TAG, "Error al parsear los datos PID. Recibido: %s", buf);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Datos mal formados");
        return ESP_FAIL;
    }
    return ESP_OK;
}

/**
 * @brief Manejador para POST /calibrate
 * Calibra un motor específico (lo mueve a 0).
 * @param req Petición HTTP (cuerpo: "motor=N").
 * @return ESP_OK si éxito, ESP_FAIL si error.
 */
static esp_err_t calibrate_handler(httpd_req_t *req) {
    char buf[50];
    if (httpd_req_recv(req, buf, req->content_len) <= 0) {
        return ESP_FAIL;
    }
    buf[req->content_len] = '\0';

    char motor_str[5];
    if (httpd_query_key_value(buf, "motor", motor_str, sizeof(motor_str)) == ESP_OK) {
        ESP_LOGI(TAG, "Calibrando motor: %s", motor_str);
        calibrate(atoi(motor_str)); // atoi convierte el texto "1" al número 1
        httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    } else {
        ESP_LOGE(TAG, "Parámetro 'motor' no encontrado en el cuerpo de la petición.");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Parámetro 'motor' no encontrado");
    }
    return ESP_OK;
}

/**
 * @brief Manejador para POST /set_angles
 * Recibe tres ángulos (desde el PC) y los aplica a los motores.
 * @param req Petición HTTP (cuerpo: "angle_a=X&angle_b=Y&angle_c=Z").
 * @return ESP_OK si éxito, ESP_FAIL si error.
 */
static esp_err_t set_angles_handler(httpd_req_t *req) {
    char buf[100];
    if (httpd_req_recv(req, buf, req->content_len) <= 0) {
        return ESP_FAIL;
    }
    buf[req->content_len] = '\0'; // Terminar el string

    // Parsear los tres ángulos
    float angle_a, angle_b, angle_c;
    if (sscanf(buf, "angle_a=%f&angle_b=%f&angle_c=%f", &angle_a, &angle_b, &angle_c) == 3) {
        // Llamar a la función en motor_control (implementada en main.c)
        set_all_motors_to_angles(angle_a, angle_b, angle_c);
        httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    } else {
        ESP_LOGE(TAG, "Error al parsear datos de angulos: %s", buf);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Datos mal formados");
        return ESP_FAIL;
    }
    return ESP_OK;
}

/**
 * @brief Manejador para POST /demo
 * Ejecuta la secuencia de demostración de movimiento.
 * @param req Petición HTTP.
 * @return ESP_OK
 */
static esp_err_t demo_handler(httpd_req_t *req) {
    demo();
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/**
 * @brief Manejador para POST /move_relative
 * Mueve un motor a un ángulo específico (testing).
 * @param req Petición HTTP (cuerpo: "motor=N&angle=A").
 * @return ESP_OK si éxito, ESP_FAIL si error.
 */
static esp_err_t move_relative_handler(httpd_req_t *req) {
    char buf[100];
    if (httpd_req_recv(req, buf, req->content_len) <= 0) return ESP_FAIL;

    char motor_str[5], angle_str[10];
    if (httpd_query_key_value(buf, "motor", motor_str, sizeof(motor_str)) != ESP_OK ||
        httpd_query_key_value(buf, "angle", angle_str, sizeof(angle_str)) != ESP_OK ) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    motor_move_relative(atoi(motor_str), atof(angle_str));
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// ==========================================================
// ## FUNCIÓN PARA INICIAR EL SERVIDOR Y REGISTRAR RUTAS ##
// ==========================================================

/**
 * @brief Inicia el servidor web HTTP.
 * (Implementación de la función declarada en web_server.h)
 */
httpd_handle_t start_webserver(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    ESP_LOGI(TAG, "Iniciando servidor web en el puerto: '%d'", config.server_port);
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Error al iniciar el servidor");
        return NULL;
    }

    // --- Definición y registro de todas las URIs ---
    ESP_LOGI(TAG, "Registrando manejadores de URI");
    
    // URIs para archivos estáticos
    const httpd_uri_t root_uri = { .uri = "/", .method = HTTP_GET, .handler = root_get_handler };
    const httpd_uri_t style_uri = { .uri = "/style.css", .method = HTTP_GET, .handler = style_get_handler };
    httpd_register_uri_handler(server, &root_uri);
    httpd_register_uri_handler(server, &style_uri);
    
    // URIs para comandos de motor
    const httpd_uri_t calibrate_uri = { .uri = "/calibrate", .method = HTTP_POST, .handler = calibrate_handler };
    const httpd_uri_t demo_uri = { .uri = "/demo", .method = HTTP_POST, .handler = demo_handler };
    const httpd_uri_t move_relative_uri = { .uri = "/move_relative", .method = HTTP_POST, .handler = move_relative_handler };
    httpd_register_uri_handler(server, &calibrate_uri);
    httpd_register_uri_handler(server, &demo_uri);
    httpd_register_uri_handler(server, &move_relative_uri);
    
    // URI para el comando de PID
    const httpd_uri_t set_pid_uri = { .uri = "/set_pid", .method = HTTP_POST, .handler = set_pid_handler };
    httpd_register_uri_handler(server, &set_pid_uri);

    // URI para el control por PC
    const httpd_uri_t set_angles_uri = { .uri = "/set_angles", .method = HTTP_POST, .handler = set_angles_handler };
    httpd_register_uri_handler(server, &set_angles_uri);

    return server;
}

/**
 * @brief Detiene el servidor web.
 * (Implementación de la función declarada en web_server.h)
 */
void stop_webserver(httpd_handle_t server) {
    if (server) {
        httpd_stop(server);
    }
}