import math

def calculate_angle_between_points(p1, p2):
    """
    Calcula el ángulo en grados entre dos puntos respecto al eje x positivo.
    p1 y p2 son tuplas que representan las coordenadas (x, y).
    """
    delta_x = p2[0] - p1[0]
    delta_y = p2[1] - p1[1]
    angle = math.degrees(math.atan2(delta_y, delta_x))
    return angle

def calculate_turn_angle(current_angle, target_angle):
    """
    Calcula el ángulo que necesita doblar desde current_angle hacia target_angle.
    Los ángulos deben estar en el rango [-180, 180].
    """
    turn_angle = target_angle - current_angle
    while turn_angle > 180:
        turn_angle -= 360
    while turn_angle < -180:
        turn_angle += 360
    return turn_angle

# Ejemplo de uso
# Puntos del robot
point1 = (1, 1)  # Primer punto
point2 = (4, 5)  # Segundo punto

# Punto objetivo
target_point = (7, 9)

# Calculamos el ángulo de dirección actual del robot
current_angle = calculate_angle_between_points(point1, point2)
print(f"Ángulo actual del robot: {current_angle:.2f} grados")

# Calculamos el ángulo hacia el punto objetivo
target_angle = calculate_angle_between_points(point2, target_point)
print(f"Ángulo hacia el objetivo: {target_angle:.2f} grados")

# Calculamos el ángulo de giro necesario
turn_angle = calculate_turn_angle(current_angle, target_angle)
print(f"Ángulo de giro necesario: {turn_angle:.2f} grados")
