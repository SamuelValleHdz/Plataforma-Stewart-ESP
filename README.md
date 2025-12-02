# 🤖 Proyecto Robot Balancín de 3 Ejes (Plataforma Stewart)

![ESP-IDF](https://img.shields.io/badge/Firmware-ESP--IDF-red)
![Python](https://img.shields.io/badge/Client-Python_3.12-blue)
![OpenCV](https://img.shields.io/badge/Vision-OpenCV-green)
![Status](https://img.shields.io/badge/Status-Functional-brightgreen)

Este repositorio contiene el sistema completo de control para un robot paralelo de 3 grados de libertad (3-DOF). El proyecto combina un firmware de alto rendimiento en **ESP32** (FreeRTOS) con un cliente de procesamiento de visión y cinemática en **Python**.

---

## 📋 Tabla de Contenidos
1. [Demos en Vivo](#-demos-en-vivo)
2. [Arquitectura del Sistema](#-arquitectura-del-sistema)
3. [Hardware y Conexiones](#-hardware-y-conexiones)
4. [Instalación y Puesta en Marcha](#-instalación-y-puesta-en-marcha)
5. [Modos de Operación](#-modos-de-operación)
6. [Notas Técnicas](#-notas-técnicas-importantes)

---

## 🎥 Demos en Vivo

### 1. Sistema de Visión Artificial
El robot detecta la posición de la canica mediante una cámara web y ajusta la inclinación de la plataforma en tiempo real usando un controlador PID.
<video src="assets/vision.mp4" controls="controls" muted="muted" style="max-width: 730px;">
</video>

### 2. Modo "Dance" (Coreografía) 💃
Demostración de sincronización. Al activar este modo, el navegador reproduce audio (incrustado en Base64) mientras el robot ejecuta una secuencia de movimientos pre-programada.
<video src="assets/baile.mp4" controls="controls" muted="muted" style="max-width: 730px;">
</video>

### 3. Estabilidad General
Prueba de respuesta física y corrección de perturbaciones.
<video src="assets/robot.mp4" controls="controls" muted="muted" style="max-width: 730px;">
</video>

---

## 🏗️ Arquitectura del Sistema

El proyecto es un **Monorepo** dividido en dos grandes componentes:

```
PROYECTO_FINAL/
├── 📁 firmware_esp32/          <-- "El Cuerpo" (C / ESP-IDF)
│   ├── main/main.c             # Lógica de control y bucle principal
│   ├── components/
│   │   ├── motor_control/      # Gestión de PWM, Encoders y PID
│   │   ├── tcp_server/         # Socket TCP (Puerto 1234) para streaming
│   │   ├── web_server/         # Servidor HTTP y WebSockets
│   │   └── wifi_ap/            # Punto de Acceso WiFi
│   └── partitions.csv          # Tabla de particiones personalizada (2MB App)
│
└── 📁 client_python/           <-- "El Cerebro" (Python)
    ├── vision.py               # Detección de objetos (OpenCV)
    ├── joy_control.py          # Control manual vía Joystick
    ├── Kinematic_Robot.py      # Matemáticas de la Plataforma Stewart
    └── comunication.py         # Cliente TCP
```

## 🛠️ Hardware y Conexiones

  * **Microcontrolador:** ESP32 (DevKit V1)
  * **Drivers:** TB6612FNG (Dual Motor Driver)
  * **Actuadores:** 3x Motores DC con Encoder (JGA25-370)
  * **Alimentación:** Batería 9V externa.

### Pinout (ESP32)

| Motor | Pin PWM | Pin IN1 | Pin IN2 | Encoder A | Encoder B |
| :---: | :---: | :---: | :---: | :---: | :---: |
| **M0** | GPIO 23 | GPIO 22 | GPIO 21 | GPIO 34 | GPIO 35 |
| **M1** | GPIO 19 | GPIO 5 | GPIO 18 | GPIO 32 | GPIO 33 |
| **M2** | GPIO 4 | GPIO 15 | GPIO 2 | GPIO 25 | GPIO 26 |

> **⚠️ Nota de Hardware:** Debido a un cambio en la tapa superior, la matriz de movimiento se ha reasignado por software. M0 controla el eje C (invertido). **El control manual del Motor 0 está deshabilitado en la Web por seguridad.**

-----

## ⚙️ Instalación y Puesta en Marcha

### A. Firmware (ESP32)

1.  **Configuración de Memoria:** Este proyecto requiere una partición grande para la aplicación.

      * En `idf.py menuconfig` -\> `Serial Flasher Config`, establece **Flash Size** a **4 MB**.
      * La tabla de particiones usa `partitions.csv` (Factory App: 2MB).

2.  **Compilar y Subir:**

    ```bash
    cd firmware_esp32
    idf.py build flash monitor
    ```

3.  **WiFi:** Conéctate a la red `Robot` (Pass: `MTR09A_2022`).

### B. Cliente (Python)

1.  Instalar dependencias:
    ```bash
    cd client_python
    pip install -r requirements.txt
    ```

-----

## 🎮 Modos de Operación

### 1\. Interfaz Web (http://192.168.10.1)

  * **Ajuste PID:** Modifica Kp, Ki, Kd en tiempo real.
  * **Modo Baile:** Inicia la secuencia coreográfica con música.
  * **Calibración:** Resetea la posición cero de los motores.

### 2\. Control por Visión (Python)

Ejecuta `python vision.py`. El sistema detectará una canica naranja y moverá la plataforma para mantenerla centrada.

  * **Ajuste:** Modificar los valores HSV en `vision.py` si la iluminación cambia.

### 3\. Control Manual (Joystick)

Ejecuta `python joy_control.py`. Usa un mando de Xbox/PlayStation conectado a la PC para inclinar la plataforma.

-----

## 📝 Notas Técnicas Importantes

1.  **Audio en Web:** El audio del modo "Dance" no se almacena en el ESP32 como archivo de audio, sino que está incrustado en el HTML como una cadena **Base64**. Esto permite reproducirlo en el cliente (celular/PC) sin hardware de audio adicional en el robot.
2.  **Gestión de Recursos:** El ESP32 apaga automáticamente el Servidor Web cuando detecta una conexión TCP entrante (Python) para priorizar el control en tiempo real y evitar latencia.
3.  **Seguridad:** Se implementó un límite de seguridad (Clamp) de 60° en la cinemática inversa para evitar colisiones mecánicas.

-----