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

// Configuración del controlador PID
#define PID_KP 2.5f                // Ganancia proporcional (ajustar según respuesta deseada)
#define PID_KI 0.1f                // Ganancia integral (elimina error en estado estacionario)
#define PID_KD 0.05f               // Ganancia derivativa (reduce oscilaciones)
#define PID_MAX_OUTPUT 255         // Salida máxima del PID (corresponde al PWM máximo)
#define PID_MIN_OUTPUT -255        // Salida mínima del PID
#define PID_MAX_INTEGRAL 100.0f    // Límite para evitar windup del término integral

// Variables globales volátiles para el control del sistema

// 'volatile' indica que la variable puede cambiar en cualquier momento por una interrupción
static volatile int32_t encoder_count = 0;      // Contador para los pulsos del encoder
static float target_angle = 0;                  // Ángulo objetivo al que debe moverse el motor
static bool position_control_active = false;    // Bandera para indicar si el control de posición está activo

// Variables del controlador PID
typedef struct {
    float kp, ki, kd;                    // Ganancias del PID
    float previous_error;                // Error anterior para calcular la derivada
    float integral;                      // Acumulador del término integral
    float max_output, min_output;        // Límites de salida
    float max_integral;                  // Límite del término integral
    int64_t last_time;                   // Tiempo de la última ejecución (en microsegundos)
} pid_controller_t;

static pid_controller_t pid_controller;

// ---

// Funciones del Sistema

// Inicializar el controlador PID
void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float max_out, float min_out, float max_int) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->previous_error = 0.0f;
    pid->integral = 0.0f;
    pid->max_output = max_out;
    pid->min_output = min_out;
    pid->max_integral = max_int;
    pid->last_time = esp_timer_get_time();
}

// Calcular la salida del controlador PID
float pid_compute(pid_controller_t *pid, float setpoint, float measured_value) {
    int64_t current_time = esp_timer_get_time();
    float dt = (current_time - pid->last_time) / 1000000.0f; // Convertir a segundos
    
    if (dt <= 0.0f) {
        return 0.0f; // Evitar división por cero
    }
    
    // Calcular el error
    float error = setpoint - measured_value;
    
    // Término proporcional
    float proportional = pid->kp * error;
    
    // Término integral (con límites para evitar windup)
    pid->integral += error * dt;
    if (pid->integral > pid->max_integral) {
        pid->integral = pid->max_integral;
    } else if (pid->integral < -pid->max_integral) {
        pid->integral = -pid->max_integral;
    }
    float integral = pid->ki * pid->integral;
    
    // Término derivativo
    float derivative = pid->kd * (error - pid->previous_error) / dt;
    
    // Salida total del PID
    float output = proportional + integral + derivative;
    
    // Limitar la salida
    if (output > pid->max_output) {
        output = pid->max_output;
    } else if (output < pid->min_output) {
        output = pid->min_output;
    }
    
    // Actualizar variables para la siguiente iteración
    pid->previous_error = error;
    pid->last_time = current_time;
    
    // Debug opcional - solo mostrar cada cierto tiempo para reducir spam
    static int pid_debug_counter = 0;
    pid_debug_counter++;
    if (pid_debug_counter >= 20) {  // Mostrar cada 20 ejecuciones (cada 200ms aprox)
        printf("PID Debug - Error: %.2f, P: %.2f, I: %.2f, D: %.2f, Output: %.2f\n", 
               error, proportional, integral, derivative, output);
        pid_debug_counter = 0;
    }
    
    return output;
}

// Resetear el controlador PID (útil cuando se cambia el setpoint)
void pid_reset(pid_controller_t *pid) {
    pid->previous_error = 0.0f;
    pid->integral = 0.0f;
    pid->last_time = esp_timer_get_time();
}

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
    target_angle += angle;              // Actualiza el ángulo objetivo
    position_control_active = true;    // Activa la bandera para que la tarea de control se ejecute
    pid_reset(&pid_controller);        // Resetea el PID para evitar valores residuales
    printf("Nuevo objetivo: %.2f grados\n", target_angle);
}

// Función para detener el control de posición manualmente
void stop_position_control() {
    position_control_active = false;
    set_motor_speed(0);
    printf("Control de posicion detenido manualmente\n");
}

// Tarea del FreeRTOS para el control de posición con PID
void position_control_task(void *arg) {
    while (1) {
        // Solo ejecuta el control si está activo
        if (position_control_active) {
            float current_angle = get_current_angle();          // Obtiene el ángulo actual
            float corrected_angle = current_angle; //* (-1);       // Corrige la dirección si es necesario
            float error = target_angle - corrected_angle;       // Calcula el error

            // Control PID continuo - solo reporta cuando llega cerca del objetivo
            if (fabs(error) < 1.0f && !position_control_active) {
                printf("Posicion alcanzada: %.2f grados (objetivo: %.2f)\n", corrected_angle, target_angle);
                position_control_active = true; // Mantener el control activo para correcciones
            }
            
            // Siempre ejecutar el PID mientras el control esté activo
            {
                // Usar el controlador PID para calcular la velocidad
                float pid_output = pid_compute(&pid_controller, target_angle, corrected_angle);
                
                // Convertir la salida del PID a velocidad del motor
                int motor_speed = (int)pid_output;
                
                // Si el error es mayor a una vuelta completa, detiene el motor (medida de seguridad)
                if (fabs(error) > 360) {
                    set_motor_speed(0);
                    position_control_active = false;
                    printf("Error demasiado grande (%.2f grados), deteniendo motor\n", error);
                } else {
                    set_motor_speed(motor_speed);
                }
                
                // Solo mostrar debug cada 10 iteraciones para reducir spam
                static int debug_counter = 0;
                debug_counter++;
                if (debug_counter >= 10 || fabs(error) < 2.0f) {
                    printf("\nControl PID:\nObjetivo: %.2f°, Actual: %.2f°, Error: %.2f°, Velocidad: %d\n", 
                           target_angle, corrected_angle, error, motor_speed);
                    debug_counter = 0;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Espera 10ms antes de la siguiente iteración (100Hz de control)
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
    
    // Inicializar el controlador PID
    pid_init(&pid_controller, PID_KP, PID_KI, PID_KD, PID_MAX_OUTPUT, PID_MIN_OUTPUT, PID_MAX_INTEGRAL);
    
    // Crear las tareas del sistema operativo (FreeRTOS)
    // Se les asigna una prioridad y un núcleo de procesador
    xTaskCreatePinnedToCore(encoder_angle_task, "Get angle", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(position_control_task, "Position control", 2048, NULL, 2, NULL, 1);
    
    // Espera 1 segundo y luego inicia los movimientos de prueba
    vTaskDelay(pdMS_TO_TICKS(1000));
    move_to_position(90.0); // Mueve el motor a 90 grados
    vTaskDelay(pdMS_TO_TICKS(800)); // Espera 5 segundos para que el PID se estabilice y corrija
    move_to_position(-90.0); // Mueve el motor -90 grados (relativo)
    vTaskDelay(pdMS_TO_TICKS(800)); // Espera 5 segundos
    move_to_position(90.0); // Mueve el motor 90 grados (relativo)
    vTaskDelay(pdMS_TO_TICKS(100)); // Espera final
    stop_position_control(); // Detener el control al final de la prueba
}