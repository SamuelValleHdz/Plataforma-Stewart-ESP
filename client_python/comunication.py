# =============================================================================
#  comunication.py
#
#  Módulo de comunicación TCP para el robot balancín.
#
#  Contiene la clase 'ComunicadorRobot' que encapsula toda la lógica
#  de sockets (conexión, envío y desconexión) para hablar con el ESP32.
# =============================================================================

import socket
import struct
import logging

# Configurar un logger para este módulo
log = logging.getLogger(__name__)

class ComunicadorRobot:
    """
    Gestiona la conexión y el envío de datos a un robot a través de un socket TCP.

    Esta clase abstrae la lógica del socket para que el script principal
    solo necesite llamar a conectar(), enviar_angulos() y desconectar().
    """

    def __init__(self, ip: str, puerto: int):
        """
        Inicializa el comunicador.

        :param ip: La dirección IP del robot (ESP32).
        :param puerto: El puerto TCP en el que escucha el robot (ej: 1234).
        """
        self.ip = ip
        self.puerto = puerto
        self.cliente_socket: socket.socket | None = None

    def conectar(self) -> bool:
        """
        Intenta establecer la conexión con el robot.

        Establece un timeout corto para la conexión inicial y
        luego lo elimina para las operaciones de envío.

        :return: True si la conexión fue exitosa, False en caso contrario.
        """
        try:
            self.cliente_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # Timeout de 3 segundos solo para el intento de conexión
            self.cliente_socket.settimeout(3)
            self.cliente_socket.connect((self.ip, self.puerto))
            # Quitar el timeout para que el envío sea bloqueante (normal)
            self.cliente_socket.settimeout(None)
            print(f"Conexión establecida con el robot en {self.ip}.")
            return True
        except socket.error as e:
            print(f"Error al conectar: {e}")
            self.cliente_socket = None
            return False

    def enviar_angulos(self, angulo_a: float, angulo_b: float, angulo_c: float):
        """
        Empaqueta y envía los tres ángulos al robot.

        Los ángulos se redondean a enteros y se empaquetan como 3
        'unsigned shorts' (uint16_t) en formato Big-Endian ('>').

        :param angulo_a: Ángulo del motor A (se redondeará).
        :param angulo_b: Ángulo del motor B (se redondeará).
        :param angulo_c: Ángulo del motor C (se redondeará).
        """
        if not self.esta_conectado() or self.cliente_socket is None:
            log.warning("Intento de envío sin conexión.")
            return

        try:
            # Redondear y empaquetar los ángulos
            vector_angulos = [round(angulo_a), round(angulo_b), round(angulo_c)]
            # '>3H' = 3x Unsigned Shorts (H), Big-Endian (>)
            mensaje = struct.pack('>3H', *vector_angulos)
            self.cliente_socket.send(mensaje)
        except socket.error as e:
            print(f"Error de conexión al enviar: {e}. Desconectando.")
            self.desconectar()

    def desconectar(self):
        """Cierra la conexión del socket si está abierta."""
        if self.cliente_socket:
            self.cliente_socket.close()
            self.cliente_socket = None
            print("Conexión con el robot cerrada.")

    def esta_conectado(self) -> bool:
        """
        Verifica si el socket está actualmente conectado.

        :return: True si el socket existe, False en caso contrario.
        """
        return self.cliente_socket is not None