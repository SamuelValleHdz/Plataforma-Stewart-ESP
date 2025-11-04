# =============================================================================
#  vision_control_main.py (Versión con Threading para cámara estable)
# =============================================================================

# --- 1. IMPORTACIONES ---
import cv2
import numpy as np
import time
from threading import Thread # <<< CAMBIO: Importamos Thread
# Importa las clases de tus otros dos archivos.
from Kinematic_Robot import Machine, A, B, C 
from comunication import ComunicadorRobot

# =============================================================================
### CAMBIO: Clase para leer la cámara en un hilo separado ###
# =============================================================================
class VideoStream:
    """Clase para manejar la captura de video en un hilo dedicado."""
    def __init__(self, src=0):
        self.stream = cv2.VideoCapture(src)
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False

    def start(self):
        # Inicia el hilo para leer frames del stream
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        # Bucle que se ejecuta en el hilo, leyendo frames constantemente
        while True:
            if self.stopped:
                return
            (self.grabbed, self.frame) = self.stream.read()

    def read(self):
        # Devuelve el frame más reciente
        return self.frame

    def stop(self):
        # Indica que el hilo debe detenerse
        self.stopped = True

# =============================================================================
#  ⚙️ SECCIÓN DE CONFIGURACIÓN ⚙️ (Sin cambios)
# =============================================================================
centro_plataforma_calibrado = (884, 509)
radio_plataforma_calibrado = 300
canica_bajo = np.array([95, 50, 50])
canica_alto = np.array([130, 255, 255])

D_VAL = 65.0; E_VAL = 35.0; F_VAL = 60.0; G_VAL = 120.0
ALTURA_Z_FIJA = 110.0
PIXEL_TO_TILT_FACTOR = 0.005

IP_ESP32 = '192.168.10.1'
PUERTO = 1234
URL_CAMARA = 0#'http://192.168.10.3:8080/video'

# =============================================================================
#  🚀 LÓGICA PRINCIPAL 🚀
# =============================================================================
def main():
    print("🚀 Iniciando sistema de control por visión...")
    
    # --- INICIALIZACIÓN DE OBJETOS ---
    robot = Machine(D_VAL, E_VAL, F_VAL, G_VAL)
    comunicador = ComunicadorRobot(IP_ESP32, PUERTO)
    
    ### CAMBIO: Iniciar la cámara con la nueva clase ###
    print("🎥 Iniciando stream de la cámara...")
    vs = VideoStream(src=URL_CAMARA).start()
    time.sleep(2.0) # Damos 2 segundos para que la cámara se estabilice

    print(f"📡 Intentando conectar al robot en {IP_ESP32}...")
    if not comunicador.conectar():
        print("❌ ¡ERROR! No se pudo conectar al robot.")
        vs.stop() # Detenemos el hilo de la cámara antes de salir
        return
        
    print("✅ ¡Sistema listo! Detección y control activados.")
    
    try:
        while True:
            ### CAMBIO: Leer el frame desde nuestro hilo, nunca se congela ###
            frame = vs.read()
            if frame is None:
                print("El frame de la cámara está vacío, esperando...")
                time.sleep(0.5)
                continue

            # --- Detección y Lógica (sin cambios) ---
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mascara_canica = cv2.inRange(hsv_frame, canica_bajo, canica_alto)
            contornos, _ = cv2.findContours(mascara_canica, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Dibujos estáticos (los hacemos siempre para que se vean aunque no haya canica)
            cv2.circle(frame, centro_plataforma_calibrado, radio_plataforma_calibrado, (0, 0, 255), 3)
            cv2.circle(frame, centro_plataforma_calibrado, 10, (0, 0, 255), -1)

            if len(contornos) > 0:
                canica_contorno = max(contornos, key=cv2.contourArea)
                ((x, y), radio) = cv2.minEnclosingCircle(canica_contorno)
                M = cv2.moments(canica_contorno)
                if M["m00"] > 0:
                    centro_canica = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                    error_x = centro_canica[0] - centro_plataforma_calibrado[0]
                    error_y = centro_canica[1] - centro_plataforma_calibrado[1]
                    nx = error_x * PIXEL_TO_TILT_FACTOR
                    ny = error_y * PIXEL_TO_TILT_FACTOR * -1 
                    
                    angulo_a = robot.theta(A, ALTURA_Z_FIJA, nx, ny)
                    angulo_b = robot.theta(B, ALTURA_Z_FIJA, nx, ny)
                    angulo_c = robot.theta(C, ALTURA_Z_FIJA, nx, ny)

                    comunicador.enviar_angulos(angulo_a, angulo_b, angulo_c)

                    # Dibujos dinámicos
                    cv2.circle(frame, (int(x), int(y)), int(radio), (0, 255, 0), 3)
                    cv2.circle(frame, centro_canica, 7, (0, 255, 0), -1)
                    cv2.line(frame, centro_plataforma_calibrado, centro_canica, (255, 0, 0), 2)
                    texto_error = f"Error X: {error_x}, Y: {error_y}"
                    cv2.putText(frame, texto_error, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

            # --- Mostrar el frame final ---
            frame_mostrado = cv2.resize(frame, (1280, 720))
            cv2.imshow("Control por Vision", frame_mostrado)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    except KeyboardInterrupt:
        print("\nPrograma detenido por el usuario.")
    finally:
        print("Liberando recursos...")
        comunicador.desconectar()
        vs.stop() ### CAMBIO: Detenemos nuestro hilo de cámara
        cv2.destroyAllWindows()
        print("Recursos liberados. Adiós.")

if __name__ == "__main__":
    main()