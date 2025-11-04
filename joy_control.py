# =============================================================================
#  joy_control.py
#
#  Script principal para controlar el robot balancín usando un joystick.
#  Este script utiliza los módulos de cinemática y comunicación.
# =============================================================================

import time
import os
import argparse
import sys

# Importaciones de terceros
import pygame

# Importaciones locales (nuestros módulos)
from Kinematic_Robot import Machine, A, B, C
from comunication import ComunicadorRobot

# =============================================================================
#  ⚙️ SECCIÓN DE CONFIGURACIÓN ⚙️
# =============================================================================
# Dimensiones físicas del robot
D_VAL = 50.0
E_VAL = 50.0
F_VAL = 40.0
G_VAL = 65.25
# Parámetros de operación
ALTURA_Z_FIJA = 110.0
MAX_INCLINACION = 0.6
DEAD_ZONE_JOYSTICK = 0.15
# Datos de conexión del robot
IP_ESP32 = '192.168.10.1'
PUERTO = 1234

# =============================================================================
#  4. LÓGICA PRINCIPAL
# =============================================================================

def clean():
    """Limpia la pantalla de la terminal (compatible con 'cls' y 'clear')."""
    os.system('cls' if os.name == 'nt' else 'clear')

def init_joystick() -> pygame.joystick.Joystick:
    """
    Inicializa pygame y busca un joystick conectado.

    :return: El objeto Joystick de Pygame si se encuentra uno,
             o None si no se detecta ninguno.
    """
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        return None
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    return joystick

def control_loop(joystick: pygame.joystick.Joystick,
                 robot: Machine,
                 comunicador: ComunicadorRobot,
                 offline_mode: bool = False):
    """
    Bucle principal que lee el joystick, calcula ángulos y los envía.

    Este bucle se ejecuta continuamente, leyendo los ejes del joystick,
    aplicando una zona muerta, calculando la cinemática inversa y
    enviando los ángulos resultantes al ESP32.

    :param joystick: El objeto joystick de Pygame inicializado.
    :param robot: La instancia de la clase 'Machine' (cinemática).
    :param comunicador: La instancia de 'ComunicadorRobot' (red).
    :param offline_mode: Si es True, no intentará enviar datos por la red.
    """
    print("\nSistema listo. Mueve el joystick para controlar.")
    print("   Presiona CTRL+C para salir.")
    
    try:
        while True:
            # Actualizar eventos de Pygame
            pygame.event.pump()

            # 1. Leer ejes del joystick (eje Y se invierte por defecto)
            nx = joystick.get_axis(0)
            ny = joystick.get_axis(1) * -1

            # 2. Aplicar zona muerta
            if abs(nx) < DEAD_ZONE_JOYSTICK: nx = 0.0
            if abs(ny) < DEAD_ZONE_JOYSTICK: ny = 0.0

            # 3. Convertir lectura a valor de control de inclinación
            nx_control = (nx / 5.0) * MAX_INCLINACION
            ny_control = (ny / 5.0) * MAX_INCLINACION
            
            # 4. Calcular cinemática inversa
            angulo_a = robot.theta(A, ALTURA_Z_FIJA, nx_control, ny_control)
            angulo_b = robot.theta(B, ALTURA_Z_FIJA, nx_control, ny_control)
            angulo_c = robot.theta(C, ALTURA_Z_FIJA, nx_control, ny_control)

            # 5. Enviar ángulos al robot
            if not offline_mode:
                comunicador.enviar_angulos(angulo_a, angulo_b, angulo_c)

            # 6. Imprimir estado en la terminal
            clean()
            print("--- MODO OFFLINE ---" if offline_mode else "--- MODO ONLINE ---")
            print(f"Joystick -> X: {nx: .2f} Y: {ny: .2f}")
            #print(f"Control  -> N_X: {nx_control:.3f} N_Y: {ny_control:.3f}")
            print("------------------------------------------")
            print(f"Ángulo Servo A: {angulo_a:.2f}°")
            print(f"Ángulo Servo B: {angulo_b:.2f}°")
            print(f"Ángulo Servo C: {angulo_c:.2f}°")
            
            if not offline_mode and not comunicador.esta_conectado():
                 print("\n[ERROR] ¡Se perdió la conexión con el robot!")
            
            time.sleep(0.05) # Limitar el bucle a ~20 Hz

    except KeyboardInterrupt:
        print("\n\nPrograma terminado por el usuario.")
    finally:
        comunicador.desconectar()

def main():
    """
    Función principal: parsea argumentos, inicializa y lanza el bucle de control.
    
    Orquesta la inicialización del joystick, el robot (cinemática)
    y el comunicador (red) antes de pasar el control al bucle principal.
    """
    parser = argparse.ArgumentParser(description="Control con Joystick para robot Ball Balancer.")
    parser.add_argument('--offline', action='store_true', help="Ejecutar sin conectar al ESP32.")
    args = parser.parse_args()
    offline_mode = args.offline
    
    # --- Creación de los objetos "herramienta" ---
    robot = Machine(D_VAL, E_VAL, F_VAL, G_VAL)
    comunicador = ComunicadorRobot(IP_ESP32, PUERTO)

    clean()
    print("Iniciando programa de control (Joystick)...")
    time.sleep(1)

    clean()
    print("Buscando control (Joystick)...")
    joystick = init_joystick()
    if not joystick:
        print("\n¡ERROR! No se encontró ningún control conectado.")
        sys.exit(1)
    print(f"Control encontrado: {joystick.get_name()}")
    time.sleep(1.5)

    if not offline_mode:
        clean()
        print(f"Intentando conectar al robot en {IP_ESP32}:{PUERTO}...")
        if not comunicador.conectar():
            print(f"\n¡ERROR! No se pudo conectar al robot.")
            print("   Iniciando en MODO OFFLINE forzado.")
            offline_mode = True
    else:
        clean()
        print("Iniciando en MODO OFFLINE por petición del usuario.")

    time.sleep(2)
    clean()
    
    # Iniciar el bucle de control principal
    control_loop(joystick, robot, comunicador, offline_mode)

if __name__ == "__main__":
    main()