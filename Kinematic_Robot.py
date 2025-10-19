# =============================================================================
#  cinematica_robot.py
#
#  Módulo que contiene la lógica de la cinemática inversa para el robot.
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
    def __init__(self, _d, _e, _f, _g):
        """Inicializa el objeto con las dimensiones del robot."""
        self.d = _d # Radio de la base
        self.e = _e # Radio de la plataforma
        self.f = _f # Longitud del brazo corto (servo)
        self.g = _g # Longitud del brazo largo (varilla)

    def theta(self, leg, hz, nx, ny):
        """Calcula el ángulo para una pata específica (A, B o C)."""
        # --- Ecuaciones de cinemática inversa ---
        nmag = math.sqrt(nx**2 + ny**2 + 1);
        if nmag == 0: return 90.0 # Posición neutral para evitar división por cero.
        nx /= nmag; ny /= nmag; nz = 1 / nmag; angle = 0.0
        try:
            if leg == A:
                y = self.d + (self.e / 2) * (1 - (nx**2 + 3 * nz**2 + 3 * nz) / (nz + 1 - nx**2 + (nx**4 - 3 * nx**2 * ny**2) / ((nz + 1) * (nz + 1 - nx**2)))); z = hz + self.e * ny; mag = math.sqrt(y**2 + z**2); arg1 = y / mag; arg2 = (mag**2 + self.f**2 - self.g**2) / (2 * mag * self.f)
                if -1 <= arg1 <= 1 and -1 <= arg2 <= 1: angle = math.acos(arg1) + math.acos(arg2)
            elif leg == B:
                x = (math.sqrt(3) / 2) * (self.e * (1 - (nx**2 + math.sqrt(3) * nx * ny) / (nz + 1)) - self.d); y = x / math.sqrt(3); z = hz - (self.e / 2) * (math.sqrt(3) * nx + ny); mag = math.sqrt(x**2 + y**2 + z**2); arg1 = (math.sqrt(3) * x + y) / (-2 * mag); arg2 = (mag**2 + self.f**2 - self.g**2) / (2 * mag * self.f)
                if -1 <= arg1 <= 1 and -1 <= arg2 <= 1: angle = math.acos(arg1) + math.acos(arg2)
            elif leg == C:
                x = (math.sqrt(3) / 2) * (self.d - self.e * (1 - (nx**2 - math.sqrt(3) * nx * ny) / (nz + 1))); y = -x / math.sqrt(3); z = hz + (self.e / 2) * (math.sqrt(3) * nx - ny); mag = math.sqrt(x**2 + y**2 + z**2); arg1 = (math.sqrt(3) * x - y) / (2 * mag); arg2 = (mag**2 + self.f**2 - self.g**2) / (2 * mag * self.f)
                if -1 <= arg1 <= 1 and -1 <= arg2 <= 1: angle = math.acos(arg1) + math.acos(arg2)
        except (ValueError, ZeroDivisionError):
             # Si los cálculos fallan (posición inalcanzable), devuelve un ángulo seguro.
             return 90.0
        # Convierte el ángulo final de radianes a grados.
        return math.degrees(angle)