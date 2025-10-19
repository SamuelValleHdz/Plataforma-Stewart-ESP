#include "esp_http_server.h"
#include "esp_log.h"
#include <string.h>  // <-- NUEVO: Necesario para sscanf
#include <stdlib.h> 
#include <stdio.h>

// Incluimos las cabeceras de nuestros propios módulos
#include "web_server.h"
#include "motor_control.h" // Funciones como calibrate(), demo()

static const char *TAG = "web_server";

// --- Declaración de los archivos web incrustados ---
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");
extern const uint8_t style_css_start[]  asm("_binary_style_css_start");
extern const uint8_t style_css_end[]    asm("_binary_style_css_end");

// ==========================================================
// ## MANEJADORES PARA SERVIR ARCHIVOS ESTÁTICOS (HTML/CSS) ##
// ==========================================================

static esp_err_t root_get_handler(httpd_req_t *req) {
    const size_t html_size = (index_html_end - index_html_start);
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char *)index_html_start, html_size);
    return ESP_OK;
}

static esp_err_t style_get_handler(httpd_req_t *req) {
    const size_t css_size = (style_css_end - style_css_start);
    httpd_resp_set_type(req, "text/css");
    httpd_resp_send(req, (const char *)style_css_start, css_size);
    return ESP_OK;
}

// ==========================================================
// ## MANEJADORES PARA COMANDOS ##
// ==========================================================

// --- Manejador para los ajustes del PID (NUEVO) ---
static esp_err_t set_pid_handler(httpd_req_t *req) {
    char buf[128]; // Búfer para guardar el cuerpo de la petición
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

    // Parsear los datos recibidos del string usando sscanf
    float new_kp, new_ki, new_kd;
    int new_limit_percent;
    int parsed_count = sscanf(buf, "kp=%f&ki=%f&kd=%f&limit_percent=%d",
                              &new_kp, &new_ki, &new_kd, &new_limit_percent);

    if (parsed_count == 4) { // Si se parsearon los 4 valores correctamente
        // Llamamos a la función que actualiza los parámetros PID
        update_all_pid_parameters(new_kp, new_ki, new_kd, new_limit_percent);
        httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    } else {
        ESP_LOGE(TAG, "Error al parsear los datos PID. Recibido: %s", buf);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Datos mal formados");
        return ESP_FAIL;
    }

    return ESP_OK;
}

// --- Manejadores para comandos de motor (SIN CAMBIOS) ---

static esp_err_t calibrate_handler(httpd_req_t *req) {
    char buf[50];
    // 1. Leer el cuerpo (body) de la petición POST.
    if (httpd_req_recv(req, buf, req->content_len) <= 0) {
        return ESP_FAIL;
    }
    // Asegurarse de que el string termina en nulo
    buf[req->content_len] = '\0';

    char motor_str[5];

    // 2. Buscar el parámetro 'motor' en el buffer que contiene los datos del cuerpo.
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

// En web_server.c, junto a los otros manejadores...

// --- NUEVO: Manejador para recibir los ángulos calculados por el PC ---
static esp_err_t set_angles_handler(httpd_req_t *req) {
    char buf[100];
    // Recibir el cuerpo (body) de la petición POST
    if (httpd_req_recv(req, buf, req->content_len) <= 0) {
        return ESP_FAIL;
    }
    buf[req->content_len] = '\0'; // Terminar el string

    // Parsear los tres ángulos del string (ej: "angle_a=95.5&angle_b=88.2&angle_c=91.0")
    float angle_a, angle_b, angle_c;
    if (sscanf(buf, "angle_a=%f&angle_b=%f&angle_c=%f", &angle_a, &angle_b, &angle_c) == 3) {
        // Llamar a una función en motor_control para mover los motores
        set_all_motors_to_angles(angle_a, angle_b, angle_c);
        httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    } else {
        ESP_LOGE(TAG, "Error al parsear datos de angulos: %s", buf);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Datos mal formados");
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t demo_handler(httpd_req_t *req) {
    demo();
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

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

httpd_handle_t start_webserver(void) { // <-- Modificado para devolver el handle
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    ESP_LOGI(TAG, "Iniciando servidor web en el puerto: '%d'", config.server_port);
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Error al iniciar el servidor");
        return NULL; // <-- Devuelve NULL en caso de error
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
    
    // <-- NUEVO: URI para el comando de PID
    const httpd_uri_t set_pid_uri = { .uri = "/set_pid", .method = HTTP_POST, .handler = set_pid_handler };
    httpd_register_uri_handler(server, &set_pid_uri);

    const httpd_uri_t set_angles_uri = { .uri = "/set_angles", .method = HTTP_POST, .handler = set_angles_handler };
    httpd_register_uri_handler(server, &set_angles_uri);

    return server; // <-- Devuelve el handle del servidor
}

void stop_webserver(httpd_handle_t server) {
    if (server) {
        httpd_stop(server);
    }
}