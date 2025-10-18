# =============================================================================
#  comunicacion_robot.py
#
#  Módulo para manejar la conexión y el envío de datos a un robot
#  a través de un socket TCP.
# =============================================================================

import socket
import struct

class ComunicadorRobot:
    """
    Gestiona la conexión TCP con el robot.
    """
    def __init__(self, ip, puerto):
        """
        Inicializa el comunicador con la IP y el puerto del robot.
        """
        self.ip = ip
        self.puerto = puerto
        self.cliente_socket = None # Aquí guardaremos la conexión

    def conectar(self):
        """
        Intenta establecer la conexión con el robot.
        Devuelve True si tiene éxito, False si falla.
        """
        try:
            # Crea un nuevo socket para la conexión
            self.cliente_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # Fija un tiempo de espera de 3 segundos para la conexión inicial
            self.cliente_socket.settimeout(3)
            # Intenta conectar
            self.cliente_socket.connect((self.ip, self.puerto))
            # Desactiva el timeout una vez conectado para las operaciones normales
            self.cliente_socket.settimeout(None)
            print(f"✅ Conexión establecida con el robot en {self.ip}.")
            return True
        except socket.error as e:
            print(f"❌ Error al conectar: {e}")
            self.cliente_socket = None
            return False

    def enviar_angulos(self, angulo_a, angulo_b, angulo_c):
        """
        Empaqueta y envía los tres ángulos al robot.
        """
        # Si no estamos conectados, no hace nada.
        if not self.esta_conectado():
            return

        try:
            # Redondea y empaqueta los ángulos como 3 enteros cortos sin signo (unsigned short)
            # en formato Big-Endian ('>'), el estándar de red.
            vector_angulos = [round(angulo_a), round(angulo_b), round(angulo_c)]
            mensaje = struct.pack('>3H', *vector_angulos)
            self.cliente_socket.send(mensaje)
        except socket.error:
            # Si hay un error al enviar, probablemente se perdió la conexión.
            print("Error de conexión al enviar. Se perdió la conexión.")
            self.desconectar()

    def desconectar(self):
        """
        Cierra la conexión del socket si está abierta.
        """
        if self.cliente_socket:
            self.cliente_socket.close()
            self.cliente_socket = None
            print("🔌 Conexión con el robot cerrada.")

    def esta_conectado(self):
        """
        Devuelve True si el socket está activo, False en caso contrario.
        """
        return self.cliente_socket is not None 
