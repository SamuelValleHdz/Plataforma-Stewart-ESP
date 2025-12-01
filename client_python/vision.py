# =============================================================================
#  vision.py
#
#  Script principal para controlar el robot balancín usando visión artificial.
#  Este script utiliza OpenCV para detectar una canica y los módulos de
#  cinemática y comunicación para controlar el robot.
# =============================================================================

import cv2
import numpy as np
import time
import os
import argparse
import sys

# Importaciones locales (nuestros módulos)
from Kinematic_Robot import Machine, A, B, C
from comunication import ComunicadorRobot

# =============================================================================
#  ⚙️ SECCIÓN DE CONFIGURACIÓN ⚙️
# =============================================================================

# --- Configuración de Visión ---
URL_CAMARA = 2#'http://192.168.1.65:8080/video'  # 0 para la webcam por defecto
# Rango de color HSV para detectar la canica (ej: azul)
# Canica
#canica_bajo = np.array([80, 46, 225]) 
#canica_alto = np.array([179, 255, 255])

#Lapiz
canica_bajo = np.array([25, 35, 255])
canica_alto = np.array([65, 255, 255])
# Radio del círculo visual de la plataforma (solo visual)
radio_plataforma_calibrado = 150
# Zona muerta central en píxeles
PIXEL_DEADZONE = 10
# Define cuántos píxeles de error se consideran el 100% de inclinación.
# Un valor más PEQUEÑO hará al robot MÁS AGRESIVO.
# Un valor más GRANDE lo hará MÁS SUAVE.
MAX_PIXEL_ERROR_CONTROL = 180.0

# --- Configuración del Robot ---
D_VAL = 50.0; E_VAL = 50.0; F_VAL = 40.0; G_VAL = 65.25
ALTURA_Z_FIJA = 110.0
MAX_INCLINACION = 0.6
IP_ESP32 = '192.168.10.1'
PUERTO = 1234

# =============================================================================
#  4. LÓGICA PRINCIPAL
# =============================================================================

def clean():
    """Limpia la pantalla de la terminal (compatible con 'cls' y 'clear')."""
    os.system('cls' if os.name == 'nt' else 'clear')

def control_loop_vision(cap: cv2.VideoCapture,
                        robot: Machine,
                        comunicador: ComunicadorRobot,
                        offline_mode: bool):
    """
    Bucle principal de visión: captura frames, detecta la canica y controla el robot.

    :param cap: El objeto VideoCapture de OpenCV inicializado.
    :param robot: La instancia de la clase 'Machine' (cinemática).
    :param comunicador: La instancia de 'ComunicadorRobot' (red).
    :param offline_mode: Si es True, no intentará enviar datos por la red.
    """
    print("\nSistema de visión y control listo.")
    print("   Presiona 'q' en la ventana de OpenCV para salir.")
    
    # Obtener dimensiones del frame
    ancho_frame = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    alto_frame = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    centro_plataforma_calibrado = (ancho_frame // 2, alto_frame // 2)
    
    print(f"Centro detectado: {centro_plataforma_calibrado}")
    print(f"Max Error para Control: {MAX_PIXEL_ERROR_CONTROL}px")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: No se pudo leer el frame.")
                break

            # Invertir la cámara horizontalmente (modo espejo)
            frame = cv2.flip(frame, 1) #0

            # --- 1. Detección de la canica ---
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mascara_canica = cv2.inRange(hsv_frame, canica_bajo, canica_alto)
            contornos, _ = cv2.findContours(mascara_canica, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            error_x, error_y = 0.0, 0.0
            nx_control, ny_control = 0.0, 0.0

            if len(contornos) > 0:
                # Encontrar el contorno más grande (la canica)
                canica_contorno = max(contornos, key=cv2.contourArea)
                M = cv2.moments(canica_contorno)
                
                if M["m00"] > 0:
                    # Calcular el centro de la canica
                    centro_canica = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                    
                    # --- 2. Cálculo de Error ---
                    error_x = centro_canica[0] - centro_plataforma_calibrado[0]
                    error_y = centro_canica[1] - centro_plataforma_calibrado[1]
                    
                    # Aplicar zona muerta
                    if abs(error_x) < PIXEL_DEADZONE: error_x = 0.0
                    if abs(error_y) < PIXEL_DEADZONE: error_y = 0.0

                    # --- 3. Mapeo de Error a Control ---
                    # (Esta es la lógica de control Proporcional P-Only)
                    nx_control = (error_x / MAX_PIXEL_ERROR_CONTROL) * MAX_INCLINACION
                    ny_control = (error_y / MAX_PIXEL_ERROR_CONTROL) * MAX_INCLINACION
                    
                    # --- Dibujos visuales ---
                    ((x, y), radio) = cv2.minEnclosingCircle(canica_contorno)
                    cv2.circle(frame, (int(x), int(y)), int(radio), (0, 255, 0), 3)
                    cv2.circle(frame, centro_canica, 7, (0, 255, 0), -1)
                    cv2.line(frame, centro_plataforma_calibrado, centro_canica, (255, 0, 0), 2)
                    cv2.flip(frame, 1)

            else:
                # Si no se detecta la canica, el control es 0
                nx_control = 0.0
                ny_control = 0.0
                cv2.putText(frame, "CANICA NO DETECTADA", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            # --- 4. CÁLCULO CINEMÁTICO Y ENVÍO ---
            angulo_a = robot.theta(A, ALTURA_Z_FIJA, ny_control, nx_control)
            angulo_b = robot.theta(B, ALTURA_Z_FIJA, ny_control, nx_control)
            angulo_c = robot.theta(C, ALTURA_Z_FIJA, ny_control, nx_control)

            if not offline_mode:
                comunicador.enviar_angulos(angulo_a, angulo_b, angulo_c)

            # --- 5. Mostrar Información en Pantalla ---
            texto_error = f"Error X: {error_x:.0f} Y: {error_y:.0f}"
            texto_control = f"Control N_X: {nx_control:.3f} N_Y: {ny_control:.3f}"
            texto_angulos = f"Angulos A: {angulo_a:.1f} B: {angulo_b:.1f} C: {angulo_c:.1f}"
            
            cv2.putText(frame, texto_error, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(frame, texto_control, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(frame, texto_angulos, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            
            # Dibujar elementos estáticos de la GUI
            cv2.circle(frame, centro_plataforma_calibrado, radio_plataforma_calibrado, (0, 0, 255), 3)
            cv2.circle(frame, centro_plataforma_calibrado, PIXEL_DEADZONE, (0, 255, 255), 2)
            if offline_mode:
                cv2.putText(frame, "MODO OFFLINE", (ancho_frame - 250, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            # Redimensionar para mostrar (opcional)
            frame_mostrado = cv2.resize(frame, (960, 540))
            cv2.imshow("Robot Balancin - Control por Vision", frame_mostrado)

            # Salir con la tecla 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    except KeyboardInterrupt:
        print("\n\nPrograma terminado por el usuario.")
    
    finally:
        # Liberar recursos
        cap.release()
        cv2.destroyAllWindows()
        comunicador.desconectar()
        print("Recursos liberados (cámara y socket).")

def main():
    """
    Función principal: parsea args, inicializa cámara y robot, y lanza el bucle de visión.
    
    Orquesta la inicialización de la cámara, el robot (cinemática)
    y el comunicador (red) antes de pasar el control al bule principal.
    """
    parser = argparse.ArgumentParser(description="Control por Visión para robot Ball Balancer.")
    parser.add_argument('--offline', action='store_true', help="Ejecutar sin conectar al ESP32.")
    args = parser.parse_args()
    offline_mode = args.offline
    
    # --- Creación de los objetos "herramienta" ---
    robot = Machine(D_VAL, E_VAL, F_VAL, G_VAL)
    comunicador = ComunicadorRobot(IP_ESP32, PUERTO)

    clean(); print("Iniciando programa de control por VISIÓN...")
    time.sleep(1)

    clean(); print(f"Conectando a la cámara ({URL_CAMARA})...")
    cap = cv2.VideoCapture(URL_CAMARA)
    if not cap.isOpened():
        print("\n¡ERROR! No se pudo conectar a la cámara.")
        sys.exit(1)
    print("Cámara conectada.")
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
    control_loop_vision(cap, robot, comunicador, offline_mode)

if __name__ == "__main__":
    main()
