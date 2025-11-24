# =============================================================================
#  Kinematic_Robot.py
#
#  Módulo que contiene la lógica de la cinemática inversa para
#  el robot balancín de 3-DOF.
# =============================================================================

import math

# Constantes para referirse a cada motor/pata de forma legible.
A, B, C = 0, 1, 2

class Machine:
    """
    Calcula la cinemática inversa para el robot paralelo de 3 grados de libertad.

    Esta clase toma las dimensiones físicas del robot y, a través de su método
    'theta', determina los ángulos de los servos (A, B, C) necesarios para
    inclinar la plataforma a una orientación específica (definida por los
    vectores normales nx, ny) mientras se mantiene a una altura fija (hz).
    """

    def __init__(self, _d: float, _e: float, _f: float, _g: float):
        """
        Inicializa el objeto con las dimensiones físicas del robot.

        :param _d: Radio de la base.
        :param _e: Radio de la plataforma.
        :param _f: Longitud del brazo corto (servo).
        :param _g: Longitud del brazo largo (varilla).
        """
        self.d = _d
        self.e = _e
        self.f = _f
        self.g = _g

    def theta(self, leg: int, hz: float, nx: float, ny: float) -> float:
        """
        Calcula el ángulo para una pata específica (A, B o C).

        Implementa las ecuaciones de cinemática inversa para la
        plataforma Stewart.

        :param leg: La pata a calcular (usar constantes A, B, o C).
        :param hz: La altura Z objetivo de la plataforma.
        :param nx: La componente normal X de la inclinación de la plataforma.
        :param ny: La componente normal Y de la inclinación de la plataforma.
        :return: El ángulo calculado en grados (ej: 90.0).
                 Devuelve 90.0 si la posición es inalcanzable.
        """
        # --- Ecuaciones de cinemática inversa ---
        nmag = math.sqrt(nx**2 + ny**2 + 1)
        if nmag == 0:
            return 50.0 # Posición neutral para evitar división por cero.
        
        nx /= nmag
        ny /= nmag
        nz = 1 / nmag
        angle = 0.0

        try:
            if leg == A:
                y = self.d + (self.e / 2) * (1 - (nx**2 + 3 * nz**2 + 3 * nz) / (nz + 1 - nx**2 + (nx**4 - 3 * nx**2 * ny**2) / ((nz + 1) * (nz + 1 - nx**2))))
                z = hz + self.e * ny
                mag = math.sqrt(y**2 + z**2)
                arg1 = y / mag
                arg2 = (mag**2 + self.f**2 - self.g**2) / (2 * mag * self.f)
            elif leg == B:
                x = (math.sqrt(3) / 2) * (self.e * (1 - (nx**2 + math.sqrt(3) * nx * ny) / (nz + 1)) - self.d)
                y = x / math.sqrt(3)
                z = hz - (self.e / 2) * (math.sqrt(3) * nx + ny)
                mag = math.sqrt(x**2 + y**2 + z**2)
                arg1 = (math.sqrt(3) * x + y) / (-2 * mag)
                arg2 = (mag**2 + self.f**2 - self.g**2) / (2 * mag * self.f)
            elif leg == C:
                x = (math.sqrt(3) / 2) * (self.d - self.e * (1 - (nx**2 - math.sqrt(3) * nx * ny) / (nz + 1)))
                y = -x / math.sqrt(3)
                z = hz + (self.e / 2) * (math.sqrt(3) * nx - ny)
                mag = math.sqrt(x**2 + y**2 + z**2)
                arg1 = (math.sqrt(3) * x - y) / (2 * mag)
                arg2 = (mag**2 + self.f**2 - self.g**2) / (2 * mag * self.f)
            else:
                return 20.0 # Pata no válida

            # Comprobar si los argumentos de acos están en el rango [-1, 1]
            if -1 <= arg1 <= 1 and -1 <= arg2 <= 1:
                angle = math.acos(arg1) + math.acos(arg2)
            else:
                # Posición inalcanzable (fuera de rango)
                return 0.0

        except (ValueError, ZeroDivisionError):
             # Si los cálculos fallan (ej. división por cero), devuelve un ángulo seguro.
             return 30.0
        
        # Convierte el ángulo final de radianes a grados
        angle_degrees = math.degrees(angle)
        
        # Devuelve el valor más pequeño entre el ángulo calculado y 60
        return min(angle_degrees, 60.0)