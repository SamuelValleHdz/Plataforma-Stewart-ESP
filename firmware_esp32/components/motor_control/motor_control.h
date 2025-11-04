#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include "driver/gpio.h"
#include "driver/ledc.h"

// =================================================================
// 1. CONFIGURACIÓN DE HARDWARE (PINES)
// =================================================================

// Motor 0 - Definición de pines para PWM, dirección y encoder
#define MOTOR0_PWM_PIN GPIO_NUM_23
#define MOTOR0_IN1_PIN GPIO_NUM_22
#define MOTOR0_IN2_PIN GPIO_NUM_21
#define ENCODER0_A_PIN GPIO_NUM_34
#define ENCODER0_B_PIN GPIO_NUM_35

// Motor 1 - Definición de pines para PWM, dirección y encoder
#define MOTOR1_PWM_PIN GPIO_NUM_19
#define MOTOR1_IN1_PIN GPIO_NUM_5
#define MOTOR1_IN2_PIN GPIO_NUM_18
#define ENCODER1_A_PIN GPIO_NUM_32
#define ENCODER1_B_PIN GPIO_NUM_33

// Motor 2 - Definición de pines para PWM, dirección y encoder
#define MOTOR2_PWM_PIN GPIO_NUM_4
#define MOTOR2_IN1_PIN GPIO_NUM_15
#define MOTOR2_IN2_PIN GPIO_NUM_2
#define ENCODER2_A_PIN GPIO_NUM_25
#define ENCODER2_B_PIN GPIO_NUM_26

// =================================================================
// 2. CONFIGURACIÓN DE PARÁMETROS FÍSICOS Y DE CONTROL
// =================================================================
#define ENCODER_CPR 9     // Pulsos por revolución del encoder
#define GEARBOX_RATIO 11  // Relación de la caja reductora
#define PWM_FREQUENCY 1000 // Frecuencia en Hz
#define PWM_TIMER LEDC_TIMER_0
#define PWM_MODE LEDC_LOW_SPEED_MODE
#define PWM_DUTY_RES LEDC_TIMER_8_BIT // Resolución de 8 bits (0-255)

// --- Ganancias y Límites del Controlador PID ---
// (Se definen en main.c) 
extern float PID_KP;
extern float PID_KI;
extern float PID_KD;
extern int PID_MAX_OUTPUT;
extern int PID_MIN_OUTPUT;
#define PID_MAX_INTEGRAL 100.0f // Límite para evitar "integral windup"

// =================================================================
// 3. INTERFAZ PÚBLICA (Funciones y Tipos)
// =================================================================

typedef enum {
    MOTOR_0 = 0,
    MOTOR_1 = 1,
    MOTOR_2 = 2,
    NUM_MOTORS // Número total de motores
} motor_id_t;

/**
 * @brief Inicializa los motores, encoders, PWM y la tarea de control PID.
 */
void motor_control_init(void);

/**
 * @brief Actualiza las ganancias PID y los límites para TODOS los motores.
 *
 * @param kp Nueva ganancia Proporcional.
 * @param ki Nueva ganancia Integral.
 * @param kd Nueva ganancia Derivativa.
 * @param limit_percent Nuevo límite de salida (0-100%).
 */
void update_all_pid_parameters(float kp, float ki, float kd, int limit_percent);

/**
 * @brief Obtiene el ángulo actual del motor en grados.
 *
 * @param motor_id El motor a consultar (MOTOR_0, MOTOR_1, o MOTOR_2).
 * @return El ángulo actual en grados.
 */
float motor_get_current_angle(motor_id_t motor_id);

/**
 * @brief Establece un nuevo ángulo objetivo para el motor.
 *
 * @param motor_id El motor a mover (MOTOR_0, MOTOR_1, o MOTOR_2).
 * @param angle_change El nuevo ángulo objetivo en grados.
 */
void motor_move_relative(motor_id_t motor_id, float angle_change);

/**
 * @brief Establece los ángulos objetivo para los 3 motores (control desde PC).
 *
 * @param angle_a Ángulo objetivo para MOTOR_0.
 * @param angle_b Ángulo objetivo para MOTOR_1.
 * @param angle_c Ángulo objetivo para MOTOR_2.
 */
void set_all_motors_to_angles(float angle_a, float angle_b, float angle_c);

/**
 * @brief Detiene el control de posición y apaga el motor.
 *
 * @param motor_id El motor a detener (MOTOR_0, MOTOR_1, o MOTOR_2).
 */
void motor_stop(motor_id_t motor_id);

/**
 * @brief Mueve un motor a su posición 0 (calibración).
 *
 * @param motor_n El motor a calibrar (0, 1, o 2).
 */
void calibrate(int motor_n);

/**
 * @brief Ejecuta una secuencia de demostración de movimiento.
 */
void demo(void);

#endif // MOTOR_CONTROL_H