#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#define ENCODER_A_PIN GPIO_NUM_27
#define ENCODER_B_PIN GPIO_NUM_14
#define ENCODER_CPR 99  // Counts per revolution (without gearbox)
#define GEARBOX_RATIO 1   // Gearbox ratio

static volatile int32_t encoder_count = 0;

static void IRAM_ATTR encoder_isr_handler(void* arg) {
    int pin = (int)(intptr_t)arg;
    int a = gpio_get_level(ENCODER_A_PIN);
    int b = gpio_get_level(ENCODER_B_PIN);

    // Quadrature decoding: check which pin triggered and state of the other
    if (pin == ENCODER_A_PIN) {
        if (a == b) {
            encoder_count++;
        } else {
            encoder_count--;
        }
    } else if (pin == ENCODER_B_PIN) {
        if (a != b) {
            encoder_count++;
        } else {
            encoder_count--;
        }
    }
}

void encoder_angle_task(void *arg) {
    while (1) {
        // Cada ciclo completo son 4 flancos por CPR y relación de engranaje
        float revolutions = (float)encoder_count / (4 * ENCODER_CPR * GEARBOX_RATIO);
        float angle = revolutions * 360.0f; // Ángulo en grados
        printf("Encoder angle (degrees): %.2f\n", angle);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

esp_err_t configuracion(){
    gpio_set_direction(ENCODER_A_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(ENCODER_A_PIN, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(ENCODER_A_PIN, GPIO_INTR_ANYEDGE);

    gpio_set_direction(ENCODER_B_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(ENCODER_B_PIN, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(ENCODER_B_PIN, GPIO_INTR_ANYEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(ENCODER_A_PIN, encoder_isr_handler, (void*)(intptr_t)ENCODER_A_PIN);
    gpio_isr_handler_add(ENCODER_B_PIN, encoder_isr_handler, (void*)(intptr_t)ENCODER_B_PIN);

    return ESP_OK;
}


void app_main(void) {
    configuracion();
    xTaskCreatePinnedToCore(encoder_angle_task, "Get angle", 2048, NULL, 1, NULL, 1);

    
}

