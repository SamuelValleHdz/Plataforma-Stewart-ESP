# =============================================================================
#  comunicacion_robot.py
#
#  Módulo para manejar la conexión y el envío de datos a un robot
#  a través de un socket TCP.
# =============================================================================

import socket
import struct

class ComunicadorRobot:
    """Gestiona la conexión TCP con el robot."""
    def __init__(self, ip, puerto):
        """Inicializa el comunicador con la IP y el puerto del robot."""
        self.ip = ip
        self.puerto = puerto
        self.cliente_socket = None

    def conectar(self):
        """Intenta establecer la conexión con el robot."""
        try:
            self.cliente_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.cliente_socket.settimeout(3)
            self.cliente_socket.connect((self.ip, self.puerto))
            self.cliente_socket.settimeout(None)
            print(f"✅ Conexión establecida con el robot en {self.ip}.")
            return True
        except socket.error as e:
            print(f"❌ Error al conectar: {e}")
            self.cliente_socket = None
            return False

    def enviar_angulos(self, angulo_a, angulo_b, angulo_c):
        """Empaqueta y envía los tres ángulos al robot."""
        if not self.esta_conectado():
            return
        try:
            vector_angulos = [round(angulo_a), round(angulo_b), round(angulo_c)]
            mensaje = struct.pack('>3H', *vector_angulos)
            self.cliente_socket.send(mensaje)
        except socket.error:
            print("Error de conexión al enviar. Se perdió la conexión.")
            self.desconectar()

    def desconectar(self):
        """Cierra la conexión del socket si está abierta."""
        if self.cliente_socket:
            self.cliente_socket.close()
            self.cliente_socket = None
            print("🔌 Conexión con el robot cerrada.")

    def esta_conectado(self):
        """Devuelve True si el socket está activo, False en caso contrario."""
        return self.cliente_socket is not None