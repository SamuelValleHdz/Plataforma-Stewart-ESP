#ifndef TCP_SERVER_H
#define TCP_SERVER_H

/**
 * @brief Tarea de FreeRTOS que maneja el servidor TCP.
 *
 * Escucha en el puerto 1234, acepta una conexión y recibe
 * paquetes de 6 bytes (3x uint16_t) con los ángulos objetivo.
 *
 * @param pvParameters Parámetros de la tarea (no se usa).
 */
void tcp_server_task(void *pvParameters);

#endif // TCP_SERVER_H