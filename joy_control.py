# =============================================================================
#  control_joystick.py
#
#  Script principal (MAIN) para controlar el robot usando un joystick.
#  Orquesta la cinemática y la comunicación.
# =============================================================================

# --- 1. IMPORTACIONES ---
import time
import pygame
import os
import argparse
import sys
# Importa las clases de los otros dos archivos.
from Kinematic_Robot import Machine, A, B, C 
from comunication import ComunicadorRobot  

# =============================================================================
#  ⚙️ SECCIÓN DE CONFIGURACIÓN ⚙️
# =============================================================================
# Dimensiones físicas del robot.
D_VAL = 50.0; E_VAL = 50.0; F_VAL = 40.0; G_VAL = 65.25
# Parámetros de operación.
ALTURA_Z_FIJA = 110.0
MAX_INCLINACION = 0.6
# Datos de conexión del robot.
IP_ESP32 = '192.168.10.1'
PUERTO = 1234

# =============================================================================
#  4. LÓGICA PRINCIPAL
# =============================================================================
def clean():
    """Limpia la pantalla de la terminal."""
    os.system('cls' if os.name == 'nt' else 'clear')

def init_joystick():
    """Inicializa pygame y busca un joystick conectado."""
    pygame.init(); pygame.joystick.init()
    if pygame.joystick.get_count() == 0: return None
    joystick = pygame.joystick.Joystick(0); joystick.init()
    return joystick

def control_loop(joystick, robot, comunicador, offline_mode=False):
    """Bucle principal que lee el joystick, calcula ángulos y los envía."""
    print("\n✅ Sistema listo. Mueve el joystick para controlar.")
    print("   Presiona CTRL+C para salir.")
    
    try:
        while True:
            pygame.event.pump()
            nx = joystick.get_axis(0)
            ny = joystick.get_axis(1) * -1
            dead_zone = 0.15
            if abs(nx) < dead_zone: nx = 0.0
            if abs(ny) < dead_zone: ny = 0.0
            nx_control = nx * MAX_INCLINACION
            ny_control = ny * MAX_INCLINACION
            
            # Usa el objeto 'robot' para calcular los ángulos.
            angulo_a = robot.theta(A, ALTURA_Z_FIJA, nx_control, ny_control)
            angulo_b = robot.theta(B, ALTURA_Z_FIJA, nx_control, ny_control)
            angulo_c = robot.theta(C, ALTURA_Z_FIJA, nx_control, ny_control)

            # Si no estamos en modo offline, usa el 'comunicador' para enviar los datos.
            if not offline_mode:
                comunicador.enviar_angulos(angulo_a, angulo_b, angulo_c)

            clean()
            print("--- MODO OFFLINE ---" if offline_mode else "--- MODO ONLINE ---")
            print(f"Joystick -> X: {nx: .2f} Y: {ny: .2f}")
            print("------------------------------------------")
            print(f"Ángulo Servo A: {angulo_a:.2f}°")
            print(f"Ángulo Servo B: {angulo_b:.2f}°")
            print(f"Ángulo Servo C: {angulo_c:.2f}°")
            
            if not offline_mode and not comunicador.esta_conectado():
                 print("\n[ERROR] ¡Se perdió la conexión con el robot!")
            
            time.sleep(0.05)

    except KeyboardInterrupt:
        comunicador.desconectar()
        print("\n\nPrograma terminado por el usuario.")

def main():
    """Función principal que organiza el inicio del programa."""
    parser = argparse.ArgumentParser(description="Control para robot Ball Balancer.")
    parser.add_argument('--offline', action='store_true', help="Ejecutar sin conectar al ESP32.")
    args = parser.parse_args()
    offline_mode = args.offline
    
    # --- Creación de los objetos "herramienta" ---
    # 1. Crea una instancia del robot con sus dimensiones.
    robot = Machine(D_VAL, E_VAL, F_VAL, G_VAL)
    # 2. Crea una instancia del comunicador con los datos de red.
    comunicador = ComunicadorRobot(IP_ESP32, PUERTO)

    clean(); print("🚀 Iniciando programa de control...")
    time.sleep(1)

    clean(); print("🔌 Buscando control (Joystick)...")
    joystick = init_joystick()
    if not joystick:
        print("\n❌ ¡ERROR! No se encontró ningún control conectado.")
        sys.exit(1)
    print(f"✅ Control encontrado: {joystick.get_name()}")
    time.sleep(1.5)

    if not offline_mode:
        clean()
        print(f"📡 Intentando conectar al robot en {IP_ESP32}...")
        # Usa el método del objeto comunicador.
        if not comunicador.conectar():
            print(f"\n❌ ¡ERROR! No se pudo conectar al robot.")
            print("\n   Iniciando en MODO OFFLINE forzado.")
            offline_mode = True
    else:
        clean()
        print("🎮 Iniciando en MODO OFFLINE por petición del usuario.")

    time.sleep(2)
    clean()
    
    # Pasa los objetos creados (joystick, robot, comunicador) al bucle principal.
    control_loop(joystick, robot, comunicador, offline_mode)

if __name__ == "__main__":
    main()