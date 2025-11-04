# =============================================================================
#  control_robot.py
#
#  Script principal (MAIN) para controlar el robot.
#  Orquesta la cinemática y la comunicación.
#  Modificado para una prueba de movimiento circular automático.
# =============================================================================

# --- 1. IMPORTACIONES ---
import time
import os
import argparse
import sys
import math  # <--- AÑADIDO para sin/cos
# 'pygame' se ha eliminado porque no se usa
from Kinematic_Robot import Machine, A, B, C 
from comunication import ComunicadorRobot  

# =============================================================================
#  ⚙️ SECCIÓN DE CONFIGURACIÓN ⚙️
# =============================================================================
# Dimensiones físicas del robot.
D_VAL = 50.0; E_VAL = 50.0; F_VAL = 40.0; G_VAL = 65.25
# Parámetros de operación.
ALTURA_Z_FIJA = 110.0
MAX_INCLINACION = 0.6 # Amplitud del círculo
# Datos de conexión del robot.
IP_ESP32 = '192.168.10.1'
PUERTO = 1234

# =============================================================================
#  4. LÓGICA PRINCIPAL
# =============================================================================
def clean():
    """Limpia la pantalla de la terminal."""
    os.system('cls' if os.name == 'nt' else 'clear')

def control_loop(robot, comunicador, offline_mode=False):
    """Bucle principal que genera un movimiento circular de la base."""
    print("\n✅ Sistema listo. Iniciando movimiento circular.")
    print("   Presiona CTRL+C para salir.")
    
    # Puedes ajustar 'velocidad_giro' para que el círculo sea más rápido o lento
    velocidad_giro = 1.0  # 1.0 es una velocidad normal
    
    try:
        while True:
            # 1. Calcular la inclinación (nx, ny) en un círculo
            # Usamos time.time() como un ángulo que incrementa constantemente
            t = (time.time() * velocidad_giro)/2
            nx_control = MAX_INCLINACION * math.cos(t)
            ny_control = MAX_INCLINACION * math.sin(t)
        
            # 2. Usa el objeto 'robot' para calcular los ángulos.
            angulo_a = robot.theta(A, ALTURA_Z_FIJA, nx_control, ny_control)
            angulo_b = robot.theta(B, ALTURA_Z_FIJA, nx_control, ny_control)
            angulo_c = robot.theta(C, ALTURA_Z_FIJA, nx_control, ny_control)

            # 3. Si no estamos en modo offline, envía los datos.
            if not offline_mode:
                comunicador.enviar_angulos(angulo_a, angulo_b, angulo_c)

            # 4. Imprime el estado
            clean()
            print("--- MODO OFFLINE ---" if offline_mode else "--- MODO ONLINE ---")
            print(f"Inclinación -> X: {nx_control: .2f} Y: {ny_control: .2f}")
            print("------------------------------------------")
            print(f"Ángulo Servo A: {angulo_a:.2f}°")
            print(f"Ángulo Servo B: {angulo_b:.2f}°")
            print(f"Ángulo Servo C: {angulo_c:.2f}°")
            
            if not offline_mode and not comunicador.esta_conectado():
                 print("\n[ERROR] ¡Se perdió la conexión con el robot!")
            
            time.sleep(0.05) # Tasa de actualización

    except KeyboardInterrupt:
        print("\n\nPrograma terminado. Enviando posición neutral...")
        try:
            # Poner en neutral (0 inclinación) al salir
            angulo_a = robot.theta(A, ALTURA_Z_FIJA, 0.0, 0.0)
            angulo_b = robot.theta(B, ALTURA_Z_FIJA, 0.0, 0.0)
            angulo_c = robot.theta(C, ALTURA_Z_FIJA, 0.0, 0.0)
            if not offline_mode and comunicador.esta_conectado():
                comunicador.enviar_angulos(angulo_a, angulo_b, angulo_c)
            print("Posición neutral enviada.")
        except Exception as e:
            print(f"No se pudo enviar la posición neutral: {e}")
        
        comunicador.desconectar()
        print("Programa finalizado.")

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

    # Conectar al robot
    if not offline_mode:
        print(f"Intentando conectar a {IP_ESP32}:{PUERTO}...")
        if not comunicador.conectar():
            print("❌ No se pudo conectar. Revisa la IP o el estado del robot.")
            print("Ejecuta con '--offline' para probar sin conexión.")
            sys.exit(1) # Termina el script si no puede conectar
    else:
        print("Ejecutando en modo OFFLINE. No se enviarán datos.")

    time.sleep(1)
    clean()
    
    # Pasa los objetos creados (robot, comunicador) al bucle principal.
    control_loop(robot, comunicador, offline_mode)

if __name__ == "__main__":
    main()