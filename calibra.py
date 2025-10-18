import cv2
import numpy as np

# Una función vacía que necesitamos para los trackbars
def nada(x):
    pass

# --- CARGAR LA IMAGEN ---
# Asegúrate de que el nombre coincida con tu foto
img = cv2.imread('canica2.jpg')

# Si la imagen es muy grande para tu pantalla, puedes redimensionarla
img = cv2.resize(img, (640, 480))

# Convertir la imagen a espacio de color HSV
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# --- CREAR LA VENTANA DE CONTROLES ---
cv2.namedWindow('Controles')

# Crear los trackbars (deslizadores) para los valores MÍNIMOS de H, S, V
# H (Hue/Tono) va de 0 a 179 en OpenCV
cv2.createTrackbar('H_MIN', 'Controles', 0, 179, nada)
cv2.createTrackbar('S_MIN', 'Controles', 0, 255, nada)
cv2.createTrackbar('V_MIN', 'Controles', 0, 255, nada)

# Crear los trackbars para los valores MÁXIMOS de H, S, V
cv2.createTrackbar('H_MAX', 'Controles', 179, 179, nada)
cv2.createTrackbar('S_MAX', 'Controles', 255, 255, nada)
cv2.createTrackbar('V_MAX', 'Controles', 255, 255, nada)

while True:
    # --- LEER LOS VALORES ACTUALES DE LOS TRACKBARS ---
    h_min = cv2.getTrackbarPos('H_MIN', 'Controles')
    s_min = cv2.getTrackbarPos('S_MIN', 'Controles')
    v_min = cv2.getTrackbarPos('V_MIN', 'Controles')
    
    h_max = cv2.getTrackbarPos('H_MAX', 'Controles')
    s_max = cv2.getTrackbarPos('S_MAX', 'Controles')
    v_max = cv2.getTrackbarPos('V_MAX', 'Controles')

    # Crear los arrays de numpy con los valores leídos
    color_bajo = np.array([h_min, s_min, v_min])
    color_alto = np.array([h_max, s_max, v_max])

    # --- APLICAR LA MÁSCARA ---
    mascara = cv2.inRange(hsv, color_bajo, color_alto)

    # --- MOSTRAR LAS IMÁGENES ---
    cv2.imshow('Imagen Original', img)
    cv2.imshow('Mascara Resultante', mascara)

    # Romper el bucle si se presiona la tecla 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()

# Imprimir los valores finales para que los copies
print("\n¡Calibración terminada! Copia estos valores en tu script principal:")
print(f"azul_bajo = np.array([{h_min}, {s_min}, {v_min}])")
print(f"azul_alto = np.array([{h_max}, {s_max}, {v_max}])")