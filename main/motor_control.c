#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"

// Incluimos nuestra cabecera, que AHORA CONTIENE los pines
#include "motor_control.h"

// =================================================================
// ESTRUCTURAS DE DATOS PRIVADAS
// =================================================================

// Estructura del controlador PID con sus ganancias, límites y estado
typedef struct {
    float kp, ki, kd;
    float previous_error;
    float integral;
    float max_output, min_output, max_integral;
    int64_t last_time;
} pid_controller_t;

// Estructura que contiene toda la información de un motor
typedef struct {
    motor_id_t id;
    gpio_num_t pwm_pin, in1_pin, in2_pin, encoder_a_pin, encoder_b_pin;
    ledc_channel_t pwm_channel;
    volatile int32_t encoder_count; // volatile porque se modifica en ISR
    float target_angle;
    bool position_control_active;
    pid_controller_t pid;
} MotorControl_t;

// Variable global "privada" gracias a 'static'. Solo es visible dentro de este archivo.
static MotorControl_t motors[NUM_MOTORS];

// =================================================================
// FUNCIONES PRIVADAS ("HELPERS") DEL MÓDULO
// =================================================================

// La palabra clave 'static' hace que estas funciones solo puedan ser llamadas desde motor_control.c

// Inicializa los parámetros del controlador PID
static void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float max_out, float min_out, float max_int) {
    pid->kp = kp; pid->ki = ki; pid->kd = kd;
    pid->previous_error = 0.0f; pid->integral = 0.0f;
    pid->max_output = max_out; pid->min_output = min_out;
    pid->max_integral = max_int;
    pid->last_time = esp_timer_get_time();
}

// Calcula la salida del PID basándose en el setpoint y el valor actual medido
static float pid_compute(pid_controller_t *pid, float setpoint, float measured_value) {
    int64_t current_time = esp_timer_get_time();
    float dt = (current_time - pid->last_time) / 1000000.0f; // Delta tiempo en segundos
    if (dt <= 0.0f) return 0.0f;

    float error = setpoint - measured_value;
    
    // Acumulación del término integral con anti-windup
    pid->integral += error * dt;
    if (pid->integral > pid->max_integral) pid->integral = pid->max_integral;
    else if (pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;

    // Cálculo de la salida PID: P + I + D
    float output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * (error - pid->previous_error) / dt);

    // Limitar la salida a los valores máximos y mínimos
    if (output > pid->max_output) output = pid->max_output;
    else if (output < pid->min_output) output = pid->min_output;

    pid->previous_error = error;
    pid->last_time = current_time;
    return output;
}

// Reinicia el estado del PID (error previo e integral a cero)
static void pid_reset(pid_controller_t *pid) {
    pid->previous_error = 0.0f;
    pid->integral = 0.0f;
    pid->last_time = esp_timer_get_time();
}

<<<<<<< HEAD

/**
 * @brief Actualiza todos los parámetros del controlador PID.
 */
// En motor_control.c

void update_all_pid_parameters(float kp, float ki, float kd, int limit_percent)
{
    // Validar el límite para que esté entre 0 y 100
    if (limit_percent < 0) limit_percent = 0;
    if (limit_percent > 100) limit_percent = 100;

    // Calcular el nuevo límite de salida basado en la resolución del PWM (0-255 para 8 bits)
    float max_output = (limit_percent / 100.0f) * 255.0f;

    // --- ¡ESTA ES LA PARTE CLAVE! ---
    // Bucle para actualizar los parámetros DENTRO de la estructura que el controlador realmente usa.
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].pid.kp = kp;
        motors[i].pid.ki = ki;
        motors[i].pid.kd = kd;
        motors[i].pid.max_output = max_output;
        motors[i].pid.min_output = -max_output;
        
        // Es buena práctica resetear el estado del PID para una transición suave
        pid_reset(&motors[i].pid);
    }
    
    // Mensaje de confirmación
    printf("\n[PID CONTROL] Parámetros actualizados en el controlador:\n");
    printf("           ├─ Ganancias: KP=%.2f, KI=%.2f, KD=%.2f\n", kp, ki, kd);
    printf("           └─ Límite de salida: %d%% (MAX: %.0f)\n", limit_percent, max_output);
}

=======
>>>>>>> base/master
// ISR que se ejecuta en cada flanco del encoder para contar pulsos y determinar dirección
static void IRAM_ATTR encoder_isr_handler(void* arg) {
    motor_id_t motor_index = (motor_id_t)(intptr_t)arg;
    // Decodificación en cuadratura: compara A y B para determinar dirección
    if (gpio_get_level(motors[motor_index].encoder_a_pin) == gpio_get_level(motors[motor_index].encoder_b_pin)) {
        motors[motor_index].encoder_count++;
    } else {
        motors[motor_index].encoder_count--;
    }
}

// Controla la velocidad y dirección del motor mediante PWM
static void set_motor_speed(motor_id_t motor_id, int speed) {
    MotorControl_t* motor = &motors[motor_id];
    if (speed > 0) {
        // Girar en sentido positivo
        gpio_set_level(motor->in1_pin, 1);
        gpio_set_level(motor->in2_pin, 0);
        ledc_set_duty(PWM_MODE, motor->pwm_channel, speed);
    } else if (speed < 0) {
        // Girar en sentido negativo
        gpio_set_level(motor->in1_pin, 0);
        gpio_set_level(motor->in2_pin, 1);
        ledc_set_duty(PWM_MODE, motor->pwm_channel, -speed);
    } else {
        // Detener motor
        gpio_set_level(motor->in1_pin, 0);
        gpio_set_level(motor->in2_pin, 0);
        ledc_set_duty(PWM_MODE, motor->pwm_channel, 0);
    }
    ledc_update_duty(PWM_MODE, motor->pwm_channel);
}

// Tarea de FreeRTOS que controla el PID para todos los motores
static void position_control_task(void *arg) {
    while (1) {
        for (int i = 0; i < NUM_MOTORS; i++) {
            if (motors[i].position_control_active) {
                float current_angle = motor_get_current_angle(i);
                float error = motors[i].target_angle - current_angle;

                // Condición de parada: si el error es menor a 1 grado, detener
                if (fabs(error) < 1.0f) {
                    motors[i].position_control_active = false;
                    set_motor_speed(i, 0);
                    continue;
                }
                
                // Calcular salida PID y aplicarla al motor
                float pid_output = pid_compute(&motors[i].pid, motors[i].target_angle, current_angle);
                set_motor_speed(i, (int)pid_output);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Bucle de control a 100Hz
    }
}

// =================================================================
// IMPLEMENTACIÓN DE LAS FUNCIONES PÚBLICAS
// =================================================================

// Convierte los pulsos del encoder en ángulo en grados
float motor_get_current_angle(motor_id_t motor_id) {
    if (motor_id >= NUM_MOTORS) return 0.0f;
    // Factor 4 por decodificación en cuadratura (4 estados por pulso)
<<<<<<< HEAD
    float revolutions = (float)motors[motor_id].encoder_count / (2.0f * ENCODER_CPR * GEARBOX_RATIO);
=======
    float revolutions = (float)motors[motor_id].encoder_count / (4.0f * ENCODER_CPR * GEARBOX_RATIO);
>>>>>>> base/master
    return revolutions * 360.0f; // Convertir revoluciones a grados
}

// Mueve el motor un ángulo relativo desde su posición actual
void motor_move_relative(motor_id_t motor_id, float angle_change) {
    if (motor_id >= NUM_MOTORS) return;
<<<<<<< HEAD
    motors[motor_id].target_angle = angle_change;
=======
    motors[motor_id].target_angle += angle_change;
>>>>>>> base/master
    motors[motor_id].position_control_active = true;
    pid_reset(&motors[motor_id].pid);
    printf("Motor %d - Nuevo objetivo: %.2f grados (relativo: %.2f)\n", motor_id, motors[motor_id].target_angle, angle_change);
}

// Detiene inmediatamente el control de posición del motor
void motor_stop(motor_id_t motor_id) {
    if (motor_id >= NUM_MOTORS) return;
    motors[motor_id].position_control_active = false;
    set_motor_speed(motor_id, 0);
    printf("Motor %d - Control de posicion detenido\n", motor_id);
}

// Inicializa todo el sistema: motores, encoders, PWM y tarea de control
void motor_control_init(void) {
    // Definir la configuración de cada motor con sus pines específicos
    motors[MOTOR_0] = (MotorControl_t){.id = MOTOR_0, .pwm_pin = MOTOR0_PWM_PIN, .in1_pin = MOTOR0_IN1_PIN, .in2_pin = MOTOR0_IN2_PIN, .encoder_a_pin = ENCODER0_A_PIN, .encoder_b_pin = ENCODER0_B_PIN, .pwm_channel = LEDC_CHANNEL_0};
    motors[MOTOR_1] = (MotorControl_t){.id = MOTOR_1, .pwm_pin = MOTOR1_PWM_PIN, .in1_pin = MOTOR1_IN1_PIN, .in2_pin = MOTOR1_IN2_PIN, .encoder_a_pin = ENCODER1_A_PIN, .encoder_b_pin = ENCODER1_B_PIN, .pwm_channel = LEDC_CHANNEL_1};
    motors[MOTOR_2] = (MotorControl_t){.id = MOTOR_2, .pwm_pin = MOTOR2_PWM_PIN, .in1_pin = MOTOR2_IN1_PIN, .in2_pin = MOTOR2_IN2_PIN, .encoder_a_pin = ENCODER2_A_PIN, .encoder_b_pin = ENCODER2_B_PIN, .pwm_channel = LEDC_CHANNEL_2};

    // Inicializar estado y PID de cada motor
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].encoder_count = 0;
        motors[i].target_angle = 0.0f;
        motors[i].position_control_active = false;
        pid_init(&motors[i].pid, PID_KP, PID_KI, PID_KD, PID_MAX_OUTPUT, PID_MIN_OUTPUT, PID_MAX_INTEGRAL);
    }

    // Configuración del timer PWM global
    ledc_timer_config_t ledc_timer = {.duty_resolution = PWM_DUTY_RES, .freq_hz = PWM_FREQUENCY, .speed_mode = PWM_MODE, .timer_num = PWM_TIMER, .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&ledc_timer);

    // Instalar servicio de interrupciones GPIO
    gpio_install_isr_service(0);

    // Configurar hardware para cada motor
    for (int i = 0; i < NUM_MOTORS; i++) {
        // Configurar pines de dirección (IN1, IN2) como salidas
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << motors[i].in1_pin) | (1ULL << motors[i].in2_pin),
            .mode = GPIO_MODE_OUTPUT,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&io_conf);
        
        // Configurar canal PWM
        ledc_channel_config_t ledc_channel = {.channel = motors[i].pwm_channel, .duty = 0, .gpio_num = motors[i].pwm_pin, .speed_mode = PWM_MODE, .hpoint = 0, .timer_sel = PWM_TIMER};
        ledc_channel_config(&ledc_channel);

        // Configurar encoder pin A con interrupción en ambos flancos
        gpio_set_direction(motors[i].encoder_a_pin, GPIO_MODE_INPUT);
        gpio_set_pull_mode(motors[i].encoder_a_pin, GPIO_PULLUP_ONLY);
        gpio_set_intr_type(motors[i].encoder_a_pin, GPIO_INTR_ANYEDGE);

        // Configurar encoder pin B como entrada
        gpio_set_direction(motors[i].encoder_b_pin, GPIO_MODE_INPUT);
        gpio_set_pull_mode(motors[i].encoder_b_pin, GPIO_PULLUP_ONLY);
        
        // Asociar ISR al pin A del encoder
        gpio_isr_handler_add(motors[i].encoder_a_pin, encoder_isr_handler, (void*)(intptr_t)i);
    }
    
    // Crear la tarea de control que manejará todos los motores
    xTaskCreatePinnedToCore(position_control_task, "Position Control Task", 4096, NULL, 5, NULL, 1);
    
    printf("Módulo de control de motores inicializado.\n");
}