#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_log.h"

// indicar el pin del esp32 y agregarle un nombre a ese pin
#define led1 2

// uint8_t: Valor positivo de 0 a 255
uint8_t led_level = 0;
static const char *TAG = "Sam-Esp32";

esp_err_t init_led(void); 
/*
    esp_err_t: Para regresar mensajes de error y su tipo de error
    init_led(): es para inicializar una funcion en c++
    void: Para indicar que la funcion no regresa ningun valor
*/
esp_err_t blink_led(void); 

void app_main(void) {
    init_led();

    while (1) {
        ESP_LOGI(TAG,"encendido");
        vTaskDelay(500 / portTICK_PERIOD_MS);
        blink_led();
        printf("Led Valuer %u\n",led_level);
    }
}

esp_err_t init_led(void){
    gpio_reset_pin(led1); // Resetea al valor por defecto del pin
    gpio_set_direction(led1,GPIO_MODE_OUTPUT); // Seleccionar un pin y su tipio de I/O
    return ESP_OK; // la opecacion se realizo correctamente
}

esp_err_t blink_led(void){
    
    led_level = !led_level;
    gpio_set_level(led1,led_level); // determinar el valor de la salida del led
    return ESP_OK; // la opecacion se realizo correctamente
}