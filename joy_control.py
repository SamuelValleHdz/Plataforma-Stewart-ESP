# =============================================================================
#  joy_control.py
#
#  Script para controlar un robot paralelo (Ball Balancer) usando un joystick.
#  Permite un modo 'online' para enviar datos a un ESP32 y un modo 'offline'
#  para probar la cinemática sin conexión.
# =============================================================================

# --- 1. IMPORTACIONES DE LIBRERÍAS ---
import socket
import time
import pygame
import os
import struct
import math
import argparse # Para manejar argumentos de línea de comandos (--offline)
import sys      # Para salir del programa si no hay joystick

# =============================================================================
#  2. CLASE DE CINEMÁTICA INVERSA
# =============================================================================
# Constantes para hacer el código más legible al referirse a cada motor/pata.
A, B, C = 0, 1, 2

# =============================================================================
#  2. CLASE DE CINEMÁTICA INVERSA
# =============================================================================
# Constantes para referirse a cada motor/pata de forma legible.
A, B, C = 0, 1, 2

class Machine:
    """
    Calcula la cinemática inversa para el robot paralelo de 3 grados de libertad.

    Esta clase toma las dimensiones físicas del robot y, a través de su método
    'theta', determina los ángulos de los servos (A, B, C) necesarios para
    inclinar la plataforma a una orientación específica (definida por los
    vectores normales nx, ny) mientras se mantiene a una altura fija (hz).
    """
    def __init__(self, _d, _e, _f, _g):
        # Constructor que inicializa el objeto con las dimensiones del robot.
        self.d = _d # Radio de la base
        self.e = _e # Radio de la plataforma
        self.f = _f # Longitud del brazo corto (servo)
        self.g = _g # Longitud del brazo largo (varilla)

    def theta(self, leg, hz, nx, ny):
        """Calcula el ángulo para una pata específica (A, B o C)."""
        # --- Inicio de las complejas ecuaciones de cinemática inversa ---
        nmag = math.sqrt(nx**2 + ny**2 + 1);
        if nmag == 0: return 90.0 # Posición neutral para evitar división por cero.
        nx /= nmag; ny /= nmag; nz = 1 / nmag; angle = 0.0
        try:
            if leg == A:
                # Cálculos específicos para el servo A
                y = self.d + (self.e / 2) * (1 - (nx**2 + 3 * nz**2 + 3 * nz) / (nz + 1 - nx**2 + (nx**4 - 3 * nx**2 * ny**2) / ((nz + 1) * (nz + 1 - nx**2)))); z = hz + self.e * ny; mag = math.sqrt(y**2 + z**2); arg1 = y / mag; arg2 = (mag**2 + self.f**2 - self.g**2) / (2 * mag * self.f)
                if -1 <= arg1 <= 1 and -1 <= arg2 <= 1: angle = math.acos(arg1) + math.acos(arg2)
            elif leg == B:
                # Cálculos específicos para el servo B
                x = (math.sqrt(3) / 2) * (self.e * (1 - (nx**2 + math.sqrt(3) * nx * ny) / (nz + 1)) - self.d); y = x / math.sqrt(3); z = hz - (self.e / 2) * (math.sqrt(3) * nx + ny); mag = math.sqrt(x**2 + y**2 + z**2); arg1 = (math.sqrt(3) * x + y) / (-2 * mag); arg2 = (mag**2 + self.f**2 - self.g**2) / (2 * mag * self.f)
                if -1 <= arg1 <= 1 and -1 <= arg2 <= 1: angle = math.acos(arg1) + math.acos(arg2)
            elif leg == C:
                # Cálculos específicos para el servo C
                x = (math.sqrt(3) / 2) * (self.d - self.e * (1 - (nx**2 - math.sqrt(3) * nx * ny) / (nz + 1))); y = -x / math.sqrt(3); z = hz + (self.e / 2) * (math.sqrt(3) * nx - ny); mag = math.sqrt(x**2 + y**2 + z**2); arg1 = (math.sqrt(3) * x - y) / (2 * mag); arg2 = (mag**2 + self.f**2 - self.g**2) / (2 * mag * self.f)
                if -1 <= arg1 <= 1 and -1 <= arg2 <= 1: angle = math.acos(arg1) + math.acos(arg2)
        except (ValueError, ZeroDivisionError):
             # Si los cálculos fallan (posición inalcanzable), devuelve un ángulo seguro.
             return 90.0
        # Convierte el ángulo final de radianes a grados.
        return math.degrees(angle)

# =============================================================================
#  ⚙️ SECCIÓN DE CONFIGURACIÓN DEL ROBOT ⚙️
#
#  ¡AQUÍ ES DONDE DEBES MODIFICAR LOS VALORES PARA ADAPTARLOS A TU ROBOT!
# =============================================================================

# --- Dimensiones Físicas (en las unidades que prefieras, ej. milímetros) ---
d_val = 65.0   # d: Radio de la base (distancia del centro al eje del servo).
e_val = 35.0   # e: Radio de la plataforma (distancia del centro al anclaje del brazo).
f_val = 60.0   # f: Longitud del brazo corto (el que está unido al servo).
g_val = 120.0  # g: Longitud del brazo largo (la varilla que conecta a la plataforma).

# --- Parámetros de Control ---
ALTURA_Z_FIJA = 110.0  # Altura de operación (eje Z). Es la altura a la que la plataforma se mantendrá.
MAX_INCLINACION = 0.6  # Sensibilidad del joystick. Un valor más alto = más inclinación. (Rango recomendado: 0.3 a 0.8).

# --- Configuración de Red ---
IP_ESP32 = '192.168.10.1' # La IP fija que configuraste en tu ESP32.
PUERTO = 1234              # El puerto que configuraste en el servidor TCP del ESP32.

# =============================================================================
#  4. LÓGICA PRINCIPAL Y FUNCIONES AUXILIARES
# =============================================================================

# --- Variables Globales ---
cliente = None # Almacenará el objeto socket de la conexión
mi_robot = Machine(d_val, e_val, f_val, g_val) # Crea una instancia del robot con tus medidas

def clean():
    """Limpia la pantalla de la terminal."""
    os.system('cls' if os.name == 'nt' else 'clear')

def connect():
    """Intenta conectar al ESP32. Devuelve True en caso de éxito."""
    global cliente
    try:
        cliente = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        cliente.settimeout(3) # Espera máximo 3 segundos para conectar
        cliente.connect((IP_ESP32, PUERTO))
        cliente.settimeout(None) # Desactiva el timeout una vez conectado
        return True
    except socket.error:
        cliente = None
        return False

def send_angles(angle_a, angle_b, angle_c):
    """Empaqueta y envía los tres ángulos al ESP32."""
    global cliente
    if not cliente:
        return # Si no estamos conectados, no hace nada.

    try:
        # Redondea los ángulos y los empaqueta como 3 enteros cortos (unsigned short)
        # en formato Big-Endian ('>'), que es el estándar de red.
        vector_angulos = [round(angle_a), round(angle_b), round(angle_c)]
        mensaje = struct.pack('>3H', *vector_angulos)
        cliente.send(mensaje)
    except socket.error:
        print("Error de conexión al enviar. Se perdió la conexión.")
        cliente = None # Marca la conexión como perdida

def init_joystick():
    """Inicializa pygame y busca un joystick."""
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        return None # No se encontró ningún joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    return joystick

def control_loop(joystick, offline_mode=False):
    """Bucle principal que lee el joystick, calcula ángulos y los muestra/envía."""
    print("\n✅ Sistema listo. Mueve el joystick para controlar.")
    print("   Presiona CTRL+C para salir.")
    
    try:
        while True:
            pygame.event.pump() # Procesa los eventos de pygame

            # Lee los ejes del joystick (valores de -1.0 a 1.0)
            nx = joystick.get_axis(0)
            ny = joystick.get_axis(1) * -1 # El eje Y a menudo viene invertido, lo corregimos.

            # "Zona muerta" para ignorar pequeños movimientos del joystick cuando está en reposo.
            dead_zone = 0.15
            if abs(nx) < dead_zone: nx = 0.0
            if abs(ny) < dead_zone: ny = 0.0

            # Aplica el factor de sensibilidad para obtener la inclinación final
            nx_control = nx * MAX_INCLINACION
            ny_control = ny * MAX_INCLINACION
            
            # Calcula los tres ángulos usando la cinemática inversa
            angulo_a = mi_robot.theta(A, ALTURA_Z_FIJA, nx_control, ny_control)
            angulo_b = mi_robot.theta(B, ALTURA_Z_FIJA, nx_control, ny_control)
            angulo_c = mi_robot.theta(C, ALTURA_Z_FIJA, nx_control, ny_control)

            # Si no estamos en modo offline, envía los datos al robot
            if not offline_mode:
                send_angles(angulo_a, angulo_b, angulo_c)

            # --- Muestra la información en la terminal ---
            clean()
            print("--- MODO OFFLINE ---" if offline_mode else "--- MODO ONLINE ---")
            print(f"Joystick -> X: {nx: .2f} Y: {ny: .2f}")
            print("------------------------------------------")
            print(f"Ángulo Servo A: {angulo_a:.2f}°")
            print(f"Ángulo Servo B: {angulo_b:.2f}°")
            print(f"Ángulo Servo C: {angulo_c:.2f}°")
            if offline_mode:
                print("\n[INFO] Los datos NO se están enviando al robot.")
            elif not cliente:
                 print("\n[ERROR] ¡Se perdió la conexión con el robot!")
            
            time.sleep(0.05) # Pequeña pausa para no saturar la red (20 actualizaciones/seg)

    except KeyboardInterrupt:
        # Se ejecuta cuando el usuario presiona CTRL+C
        if cliente:
            cliente.close()
        print("\n\nPrograma terminado por el usuario.")

def main():
    """Función principal que orquesta el arranque del programa."""
    # Configura el parser para aceptar el argumento '--offline'
    parser = argparse.ArgumentParser(description="Control para robot Ball Balancer.")
    parser.add_argument('--offline', action='store_true', help="Ejecutar sin conectar al ESP32.")
    args = parser.parse_args()
    
    offline_mode = args.offline
    
    clean()
    print("🚀 Iniciando programa de control...")
    time.sleep(1)

    # 1. Buscar Joystick
    clean()
    print("🔌 Buscando control (Joystick)...")
    joystick = init_joystick()
    if not joystick:
        print("\n❌ ¡ERROR! No se encontró ningún control conectado.")
        print("   Por favor, conecta un joystick y vuelve a intentarlo.")
        sys.exit(1) # Cierra el programa si no hay joystick
    print(f"✅ Control encontrado: {joystick.get_name()}")
    time.sleep(1.5)

    # 2. Conectar al ESP32 (a menos que estemos en modo offline)
    if not offline_mode:
        clean()
        print(f"📡 Intentando conectar al robot en {IP_ESP32}...")
        if connect():
            print(f"✅ Conexión establecida con el robot.")
        else:
            print(f"\n❌ ¡ERROR! No se pudo conectar al robot.")
            print(f"   - Verifica que el robot esté encendido.")
            print(f"   - Asegúrate de estar conectado a su red WiFi.")
            print("\n   Iniciando en MODO OFFLINE forzado.")
            offline_mode = True # Si falla la conexión, entra en modo offline
    else:
        clean()
        print("🎮 Iniciando en MODO OFFLINE por petición del usuario.")

    time.sleep(2)
    clean()
    
    # 3. Iniciar el bucle principal de control
    control_loop(joystick, offline_mode)

# =============================================================================
#  PUNTO DE ENTRADA DEL SCRIPT
# =============================================================================
if __name__ == "__main__":
    main()