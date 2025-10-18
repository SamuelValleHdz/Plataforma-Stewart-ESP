import socket
import time
from random import randrange as ran
import time
import pygame
import os
import struct

# Vector de ejemplo
vel = 0
Flag = True
cliente = 0
respuesta = 0 #"0"
# Crear socket TCP

def connect():
    global cliente
    ip_esp32 = '192.168.10.1' 
    puerto = 1234
    cliente = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    cliente.connect((ip_esp32, puerto))

def clean():
    os.system('clear')

def send(direc_x,direc_y):
    global respuesta
    vector =[direc_x,direc_y]
    mensaje = struct.pack('>2f', *vector)
    cliente.send(mensaje)
            
    # Esperar respuesta opcional
    respuesta = cliente.recv(4)
    respuesta = struct.unpack('>i', respuesta)[0]

def stick():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No hay controles conectados.")
        exit()

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    # Estado anterior para detectar cambios
    axis_prev = [0.0] * joystick.get_numaxes()
    button_prev = [0] * joystick.get_numbuttons()

    #Variables
    mul = 4
    direc_y = [0,'-']
    direc_x = [0,'-']
    ans = "no"
    
    #time.sleep(2)
    print("Escuchando eventos...")
    try:
        while True:
            pygame.event.pump()  # Necesario para actualizar los valores

            # Detectar cambio en botones
            for i in range(joystick.get_numbuttons()):
                val = joystick.get_button(i)
                if val != button_prev[i]:
                    button_prev[i] = val
                    if val: 
                        if i == 1 and mul < 6: mul += .5
                        elif i == 0 and mul > -1: mul -= .5                        

            # Detectar movimiento significativo en ejes (evita ruido)
            for i in range(joystick.get_numaxes()):
                val = joystick.get_axis(i)

                if abs(val - axis_prev[i]) > 0.1:  # Umbral de sensibilidad
                    axis_prev[i] = val
                    if i == 0:
                        if      val >=  0.15 : direc_x = [val,'>']  # <-- Derecha   - direc_x = 1 
                        elif    val <= -0.15 : direc_x = [val,'<']  # <-- Izquierda
                    if i == 1:
                        if      val >=  0.15 : direc_y = [val,'v']  # <-- Abajo
                        elif    val <= -0.15 : direc_y = [val,'^']  # <-- Arriba
                    
                    if val <= 0.08 and val >= -0.08: 
                        direc_y = [0,'-']
                        direc_x = [0,'-']
            #if direc_x[0] > 0 and int(respuesta) >-198: 
            #    ans = "si"
            #    direc_y = [0,'-']
            #    direc_x = [0,'-']
            #elif direc_x[0] < 0 and int(respuesta) < -198:
            #    ans = "si"
            #    direc_y = [0,'-']
            #    direc_x = [0,'-']
            #else : ans = "no"
            clean()
            print(f"Usando: {joystick.get_name()}")
            print   ( f"Multiplicador de Velocidad  {mul}"
                    + f"\nDireccion(x) {direc_x[0]:2f}\t{direc_x[1]}"
                    + f"\nDireccion(y) {direc_y[0]:2f}\t{direc_y[1]}"
                    + f"\nConteo {respuesta} \nse paso? {ans}")
            print(f"multiplicador {direc_x[0]/mul}")
            time.sleep(0.1)  # 1 segundo entre envíos
            send(direc_x[0]/mul,direc_y[0]/mul)

    except KeyboardInterrupt:
        cliente.close()
        print("Conexión cerrada")
connect()
stick()
