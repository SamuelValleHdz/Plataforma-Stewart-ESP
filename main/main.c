#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>

// Definiciones de pines y constantes para el hardware del proyecto

// Pines del codificador rotativo (encoder)
#define ENCODER_A_PIN GPIO_NUM_27  // Pin para la señal de salida A del encoder
#define ENCODER_B_PIN GPIO_NUM_14  // Pin para la señal de salida B del encoder
#define ENCODER_CPR 99             // Cuenta por revolución (Cycles Per Revolution) del encoder
#define GEARBOX_RATIO 1            // Relación de la caja de engranajes

// Pines del controlador del motor (TB6612FNG)
#define MOTOR_PWM_PIN GPIO_NUM_5   // Pin para la señal de control de velocidad (PWM)
#define MOTOR_IN1_PIN GPIO_NUM_18  // Pin de control para la dirección 1 del motor
#define MOTOR_IN2_PIN GPIO_NUM_19  // Pin de control para la dirección 2 del motor

// Configuración de la modulación por ancho de pulso (PWM)
#define PWM_TIMER LEDC_TIMER_0     // Selecciona el temporizador PWM 0
#define PWM_MODE LEDC_LOW_SPEED_MODE // Modo de velocidad baja para PWM (más eficiente en consumo de energía)
#define PWM_CHANNEL LEDC_CHANNEL_0 // Canal PWM a utilizar
#define PWM_DUTY_RES LEDC_TIMER_8_BIT // Resolución del ciclo de trabajo (duty cycle) de 8 bits (0-255)
#define PWM_FREQUENCY 1000         // Frecuencia del PWM en Hz

// Variables globales volátiles para el control del sistema

// 'volatile' indica que la variable puede cambiar en cualquier momento por una interrupción
static volatile int32_t encoder_count = 0;      // Contador para los pulsos del encoder
static float target_angle = 0;                  // Ángulo objetivo al que debe moverse el motor
static bool position_control_active = false;    // Bandera para indicar si el control de posición está activo

// ---

// Funciones del Sistema

// Manejador de interrupciones del encoder

// La macro 'IRAM_ATTR' coloca esta función en la memoria IRAM para una ejecución rápida
static void IRAM_ATTR encoder_isr_handler(void* arg) {
    int pin = (int)(intptr_t)arg;  // Obtiene el pin que generó la interrupción
    int a = gpio_get_level(ENCODER_A_PIN); // Lee el estado del pin A del encoder
    int b = gpio_get_level(ENCODER_B_PIN); // Lee el estado del pin B del encoder
    
    // Lógica para determinar la dirección de rotación del encoder
    if (pin == ENCODER_A_PIN) {
        // Si el pin A es el que cambió, compara con el estado de B
        if (a == b) {
            encoder_count++; // Incrementa si el motor gira en una dirección
        } else {
            encoder_count--; // Decrementa si gira en la dirección opuesta
        }
    } else if (pin == ENCODER_B_PIN) {
        // Si el pin B es el que cambió, compara con el estado de A
        if (a != b) {
            encoder_count++; // Incrementa
        } else {
            encoder_count--; // Decrementa
        }
    }
}

// ---

// Cálculos de ángulo

// Convierte el conteo del encoder a un ángulo en grados
float get_current_angle() {
    // Calcula las revoluciones: conteo total / (4 * CPR * relación)
    // El '4' es por el método de cuadratura (4 estados por ciclo)
    float revolutions = (float)encoder_count / (4 * ENCODER_CPR * GEARBOX_RATIO);
    return revolutions * 360.0f; // Multiplica por 360 para obtener los grados
}

// ---

// Control del motor

// Establece la velocidad y dirección del motor
void set_motor_speed(int speed) {
    // El valor de 'speed' va de -255 a 255
    // (-255 = máxima velocidad en reversa, 255 = máxima velocidad hacia adelante)
    if (speed > 0) {
        // Mover hacia adelante
        gpio_set_level(MOTOR_IN1_PIN, 1); // Pin IN1 a alto
        gpio_set_level(MOTOR_IN2_PIN, 0); // Pin IN2 a bajo
        ledc_set_duty(PWM_MODE, PWM_CHANNEL, speed); // Establece el ciclo de trabajo PWM
    } else if (speed < 0) {
        // Mover en reversa
        gpio_set_level(MOTOR_IN1_PIN, 0); // Pin IN1 a bajo
        gpio_set_level(MOTOR_IN2_PIN, 1); // Pin IN2 a alto
        ledc_set_duty(PWM_MODE, PWM_CHANNEL, -speed); // Usa el valor absoluto de la velocidad
    } else {
        // Detener el motor
        gpio_set_level(MOTOR_IN1_PIN, 0); // Ambos pines a bajo para frenar
        gpio_set_level(MOTOR_IN2_PIN, 0);
        ledc_set_duty(PWM_MODE, PWM_CHANNEL, 0); // Ciclo de trabajo a cero
    }
    ledc_update_duty(PWM_MODE, PWM_CHANNEL); // Aplica el nuevo ciclo de trabajo
}

// ---

// Control de posición

// Inicia el proceso de movimiento a un ángulo específico
void move_to_position(float angle) {
    target_angle = angle;              // Almacena el ángulo objetivo
    position_control_active = true;    // Activa la bandera para que la tarea de control se ejecute
}

// Tarea del FreeRTOS para el control de posición
void position_control_task(void *arg) {
    while (1) {
        // Solo ejecuta el control si está activo
        if (position_control_active) {
            float current_angle = get_current_angle();          // Obtiene el ángulo actual
            float error = target_angle - fabs(current_angle);   // Calcula el error (distancia al objetivo)
            

            // Si el error es menor a 1 grado, el motor ha llegado a su destino
            if (fabs(error) < 1.0) {
                set_motor_speed(0);                     // Detiene el motor
                position_control_active = false;        // Desactiva el control de posición
                printf("Posicion alcanzada: %.2f grados\n", current_angle);
            } else {
                // Si el error es grande, calcula la velocidad a aplicar
                int speed;

                // Lógica de control proporcional simple: a mayor error, mayor velocidad
                if (fabs(error) > 50) {
                    speed = 100; // Velocidad alta para errores grandes
                } else if (fabs(error) > 40) {
                    speed = 80;
                } else if (fabs(error) > 30) {
                    speed = 60;
                } else if (fabs(error) > 15) {
                    speed = 30;
                } else if (fabs(error) > 5) {
                    speed = 20;
                } else {
                    speed = 5;   // Velocidad muy baja para errores pequeños (evita oscilaciones)
                }
                
                printf("\n\nData: \nObjetivo: %.2f, \nPosicion Actual: %.2f\n", target_angle, current_angle);
                // Si el error es mayor a una vuelta completa, detiene el motor
                if (fabs(error) > 360) {
                    set_motor_speed(0);
                    speed = 0; 
                    position_control_active = false;
                }
                // Determina la dirección del movimiento
                if (error > 0) {
                    set_motor_speed(speed);   // Mueve hacia adelante
                } else {
                    set_motor_speed(-speed);  // Mueve en reversa
                }
                
                printf("Error: %.2f,\nVelocidad: %d\n", error, error > 0 ? speed : -speed);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Espera 10ms antes de la siguiente iteración
    }
}

// ---

// Tarea de impresión de ángulo

// Tarea del FreeRTOS para imprimir el ángulo actual en la consola
void encoder_angle_task(void *arg) {
    while (1) {
        float angle = get_current_angle(); // Obtiene el ángulo
        printf("Angulo actual: %.2f grados\n", angle); // Imprime el valor
        vTaskDelay(pdMS_TO_TICKS(500)); // Espera 500ms antes de la siguiente lectura
    }
}

// ---

// Configuración de hardware

// Configura los pines GPIO y el PWM
esp_err_t configuracion() {
    // Configurar pines del encoder
    gpio_set_direction(ENCODER_A_PIN, GPIO_MODE_INPUT);         // Pin A como entrada
    gpio_set_pull_mode(ENCODER_A_PIN, GPIO_PULLUP_ONLY);        // Activa la resistencia de pull-up interna
    gpio_set_intr_type(ENCODER_A_PIN, GPIO_INTR_ANYEDGE);       // Genera interrupción en cualquier flanco (subida o bajada)
    
    gpio_set_direction(ENCODER_B_PIN, GPIO_MODE_INPUT);         // Pin B como entrada
    gpio_set_pull_mode(ENCODER_B_PIN, GPIO_PULLUP_ONLY);        // Activa la pull-up
    gpio_set_intr_type(ENCODER_B_PIN, GPIO_INTR_ANYEDGE);       // Interrupción en cualquier flanco
    
    gpio_install_isr_service(0);                               // Instala el servicio de interrupciones
    gpio_isr_handler_add(ENCODER_A_PIN, encoder_isr_handler, (void*)(intptr_t)ENCODER_A_PIN); // Asocia el manejador de interrupciones al pin A
    gpio_isr_handler_add(ENCODER_B_PIN, encoder_isr_handler, (void*)(intptr_t)ENCODER_B_PIN); // Asocia el manejador al pin B
    
    // Configurar pines del motor
    gpio_set_direction(MOTOR_IN1_PIN, GPIO_MODE_OUTPUT);       // Pin IN1 como salida
    gpio_set_direction(MOTOR_IN2_PIN, GPIO_MODE_OUTPUT);       // Pin IN2 como salida
    
    // Configurar el temporizador PWM
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = PWM_DUTY_RES,   // 8 bits de resolución
        .freq_hz = PWM_FREQUENCY,          // Frecuencia de 1000 Hz
        .speed_mode = PWM_MODE,            // Modo de baja velocidad
        .timer_num = PWM_TIMER,            // Temporizador 0
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);         // Aplica la configuración del temporizador
    
    // Configurar el canal PWM
    ledc_channel_config_t ledc_channel = {
        .channel    = PWM_CHANNEL,         // Canal 0
        .duty       = 0,                   // Inicia con un ciclo de trabajo de 0
        .gpio_num   = MOTOR_PWM_PIN,       // Pin PWM
        .speed_mode = PWM_MODE,            // Modo de baja velocidad
        .hpoint     = 0,
        .timer_sel  = PWM_TIMER            // Temporizador 0
    };
    ledc_channel_config(&ledc_channel);     // Aplica la configuración del canal
    
    return ESP_OK; // Devuelve el código de éxito
}

// ---

// Función principal (app_main)

// Punto de entrada principal del programa
void app_main(void) {
    configuracion(); // Llama a la función de configuración del hardware
    
    // Crear las tareas del sistema operativo (FreeRTOS)
    // Se les asigna una prioridad y un núcleo de procesador
    xTaskCreatePinnedToCore(encoder_angle_task, "Get angle", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(position_control_task, "Position control", 2048, NULL, 2, NULL, 1);
    
    // Espera 3 segundos y luego inicia el movimiento
    vTaskDelay(pdMS_TO_TICKS(3000));
    move_to_position(90.0); // Mueve el motor a 90 grados
}