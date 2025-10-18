import cv2
import numpy as np

# --- RANGO DE COLOR PARA LA PLATAFORMA ROJA ---
rojo_bajo = np.array([170, 70, 50])
rojo_alto = np.array([180, 255, 255])

# --- CONEXIÓN A LA CÁMARA ---
url = 'http://192.168.1.81:8080/video' # ¡Usa la IP de tu celular!
cap = cv2.VideoCapture(url)

if not cap.isOpened():
    print("Error: No se pudo conectar a la cámara.")
    exit()

print("Apuntando a la plataforma... Presiona 'c' para calibrar.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    cv2.imshow("Calibracion - Presiona 'c'", frame)

    if cv2.waitKey(1) & 0xFF == ord('c'):
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mascara_roja = cv2.inRange(hsv_frame, rojo_bajo, rojo_alto)
        contornos, _ = cv2.findContours(mascara_roja, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contornos) > 0:
            plataforma_contorno = max(contornos, key=cv2.contourArea)
            ((x, y), radio) = cv2.minEnclosingCircle(plataforma_contorno)
            
            print("\n✅ ¡Calibración Exitosa!")
            print("\n--- COPIA ESTAS LÍNEAS EN TU SCRIPT PRINCIPAL ---")
            print(f"centro_plataforma_calibrado = ({int(x)}, {int(y)})")
            print(f"radio_plataforma_calibrado = {int(radio)}")
            break
        else:
            print("⚠️ No se detectó la plataforma roja. Intenta de nuevo.")

cap.release()
cv2.destroyAllWindows()