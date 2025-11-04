# Proyecto Robot Balancín de 3 Ejes (ESP32 + Python)

Este repositorio contiene el proyecto completo para un robot balancín (similar a una plataforma Stewart) de 3 grados de libertad. Incluye tanto el firmware para el ESP32 como el software de control en Python para la PC.

El sistema funciona de la siguiente manera:

1. **Firmware (ESP32):** Actúa como el "cuerpo". Se encarga del control de bajo nivel (PID, PWM, encoders) y crea una red WiFi.

2. **Cliente (Python):** Actúa como el "cerebro". Se ejecuta en una PC, calcula la cinemática inversa y envía los ángulos objetivo al ESP32 a través de TCP.
---
## Características Principales

- **Firmware Modular:** Código de ESP32 separado en componentes para `motor_control`, `web_server`, `tcp_server` y `wifi_ap`.
    
- **Doble Interfaz de Control:**  
    - **Servidor Web (HTTP):** Para configurar las ganancias PID, calibrar y probar los motores.

    - **Servidor de Control (TCP):** Para recibir _streaming_ de ángulos de alta frecuencia (puerto `1234`).
    
- **Cliente de PC Avanzado:** Software en Python con dos modos de control:

    - **Modo 1: Control por Joystick** (`joy_control.py`).
   
	- **Modo 2: Control por Visión Artificial** (`vision.py`) usando OpenCV.

- **Código de Cliente Modular:** La lógica de cinemática (`Kinematic_Robot.py`) y comunicación (`comunication.py`) están separadas para ser reutilizadas.
    

## Estructura del Proyecto

Este repositorio (monorepo) está organizado con las dos partes principales del proyecto:

```
/
├── firmware_esp32/     <-- Todo el proyecto de ESP-IDF
│   ├── main/
│   ├── components/
│   ├── CMakeLists.txt
│   └── sdkconfig
│
├── client_python/      <-- Todos los scripts de Python
│   ├── joy_control.py      (Modo Joystick)
│   ├── vision.py           (Modo Visión)
│   ├── Kinematic_Robot.py  (El cerebro matemático)
│   ├── comunication.py     (El "cable" de red)
│   └── requirements.txt    (Dependencias)
│
├── .gitignore
└── README.md             (¡Estás aquí!)
```

---

## Hardware y Pinout

- **Microcontrolador:** ESP32  
- **Drivers:** Motor Dual TB6612FNG
- **Motores:** 3x JGA25-370 Motores DC con Encoder
- **Fuente de alimentación:** Batería 9v

### Pinout (Definido en `firmware_esp32/components/motor_control/motor_control.h`)

|**Motor**|**Pin PWM**|**Pin IN1**|**Pin IN2**|**Encoder A**|**Encoder B**|
|---|---|---|---|---|---|
|**M0**|GPIO 23|GPIO 22|GPIO 21|GPIO 34|GPIO 35|
|**M1**|GPIO 19|GPIO 5|GPIO 18|GPIO 32|GPIO 33|
|**M2**|GPIO 4|GPIO 15|GPIO 2|GPIO 25|GPIO 26|

---

## Guía de Puesta en Marcha

Sigue estos 3 pasos para hacer funcionar el sistema.

### Paso 1: Cargar el Firmware (ESP32)

Primero, necesitas programar el ESP32.

1. Navega a la carpeta del firmware: `cd firmware_esp32`

2. Asegúrate de tener instalado **ESP-IDF** (versión `v5.4.2`).

3. Conecta tu ESP32.

4. Ejecuta `idf.py build flash monitor`.


Una vez que el ESP32 se reinicie, creará un punto de acceso WiFi:

- **SSID:** `Robot`

- **Contraseña:** `MTR09A_2022`

- **IP del ESP32:** `192.168.10.1`

### Paso 2: Configurar el Cliente (Python)

Ahora, en tu PC, instala las dependencias de Python.

1. Conéctate a la red WiFi `Robot` creada por el ESP32.

2. Navega a la carpeta del cliente: `cd client_python`

3. Instala las librerías necesarias (pygame, opencv y numpy):

   ```
   pip install -r requirements.txt
    ```

### Paso 3: ¡Ejecutar el Control!

¡Ya estás listo! Tienes dos modos para controlar el robot.

#### Modo A: Control por Joystick

1. Asegúrate de tener un Joystick conectado a tu PC.

2. Ejecuta el script:
	Bash
	```
    python joy_control.py
    ```


#### Modo B: Control por Visión

1. Asegúrate de tener una cámara web conectada.

2. Ejecuta el script:
    Bash
    ```python vision.py```

> Modo Offline: Ambos scripts se pueden ejecutar sin conectar al robot usando el flag --offline. Esto es útil para probar la cámara o el joystick.
> 
> python vision.py --offline

---
## Detalles de la Comunicación

### Interfaz Web (Configuración)

Mientras estés conectado a la red `Robot`, abre un navegador en **[http://192.168.10.1](http://192.168.10.1)**.

Desde aquí puedes:

- Ajustar las ganancias **PID** (Kp, Ki, Kd).

- **Calibrar** motores individualmente.

- Ejecutar una secuencia de **Demo**.

### Protocolo TCP (Control en Tiempo Real)

Para el control de alta velocidad, los scripts de Python usan un socket TCP.

- **Puerto:** `1234`

- **Protocolo:** El cliente envía un _payload_ de **6 bytes** en cada paquete.

- **Formato:** 3x `uint16_t` (Ángulo A, Ángulo B, Ángulo C) empaquetados en formato **Big-Endian** (Network Byte Order).

- **Implementación:** La lógica de este protocolo está en `client_python/comunication.py` (función `enviar_angulos`) y en `firmware_esp32/components/tcp_server/tcp_server.c`.

---