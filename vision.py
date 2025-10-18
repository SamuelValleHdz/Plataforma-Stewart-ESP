import cv2
import numpy as np

# --- 1. CONFIGURACIÓN ---
### PEGA AQUÍ LOS VALORES OBTENIDOS DEL SCRIPT DE CALIBRACIÓN ###
centro_plataforma_calibrado = (884, 509) # <- Ejemplo, reemplaza esto
radio_plataforma_calibrado = 300       # <- Ejemplo, reemplaza esto

# Rango para la CANICA AZUL (ajusta si es necesario)
canica_bajo = np.array([95, 50, 50])
canica_alto = np.array([130, 255, 255])

# --- 2. CONEXIÓN A LA CÁMARA ---
url = 'http://192.168.1.81:8080/video' # ¡Usa la IP de tu celular!
cap = cv2.VideoCapture(url)

if not cap.isOpened():
    print("Error: No se pudo conectar a la cámara.")
    exit()

# --- 3. BUCLE PRINCIPAL ---
while True:
    ret, frame = cap.read()
    if not ret:
        break

    # --- Dibuja los elementos estáticos de la plataforma ---
    # Círculo rojo del perímetro
    cv2.circle(frame, centro_plataforma_calibrado, radio_plataforma_calibrado, (0, 0, 255), 3)
    # Punto rojo del centro
    cv2.circle(frame, centro_plataforma_calibrado, 10, (0, 0, 255), -1)
    
    # --- Detección de la canica ---
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mascara_canica = cv2.inRange(hsv_frame, canica_bajo, canica_alto)
    contornos, _ = cv2.findContours(mascara_canica, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if len(contornos) > 0:
        canica_contorno = max(contornos, key=cv2.contourArea)
        ((x, y), radio) = cv2.minEnclosingCircle(canica_contorno)
        
        M = cv2.moments(canica_contorno)
        if M["m00"] > 0:
            centro_canica = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            
            # --- CÁLCULO DEL ERROR ---
            error_x = centro_canica[0] - centro_plataforma_calibrado[0]
            error_y = centro_canica[1] - centro_plataforma_calibrado[1]
            
            # ---------------------------------------------------
            ### ✅ LUGAR PARA ENVIAR LOS DATOS A TU ROBOT ###
            #
            # Aquí es donde llamas a tu función o script.
            # Por ejemplo, si tu función se llama 'enviar_a_robot':
            #
            # enviar_a_robot(error_x, error_y)
            #
            # O si usas otro método, lo pones aquí.
            # print(f"Enviando Error -> X: {error_x}, Y: {error_y}") # Descomenta para simular
            #
            # ---------------------------------------------------

            # --- Dibuja los elementos de la canica y el error ---
            cv2.circle(frame, (int(x), int(y)), int(radio), (0, 255, 0), 3)
            cv2.circle(frame, centro_canica, 7, (0, 255, 0), -1)
            cv2.line(frame, centro_plataforma_calibrado, centro_canica, (255, 0, 0), 2)
            
            texto_error = f"Error X: {error_x}, Y: {error_y}"
            cv2.putText(frame, texto_error, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    # --- 4. MOSTRAR EL FRAME FINAL ---
    frame_mostrado = cv2.resize(frame, (1280, 720))
    cv2.imshow("Robot Balancin - Control", frame_mostrado)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()