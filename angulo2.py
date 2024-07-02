import matplotlib.pyplot as plt
import math

def calculate_angle_between_points(p1, p2):
    delta_x = p2[0] - p1[0]
    delta_y = p2[1] - p1[1]
    angle = math.degrees(math.atan2(delta_y, delta_x))
    return angle

def calculate_turn_angle(current_angle, target_angle):
    turn_angle = target_angle - current_angle
    while turn_angle > 180:
        turn_angle -= 360
    while turn_angle < -180:
        turn_angle += 360
    return turn_angle

# Puntos del robot
point1 = (2, 1)  # Primer punto
point2 = (4, 6)  # Segundo punto

# Punto objetivo
target_point = (7, 9)

# Calculamos el ángulo de dirección actual del robot
current_angle = calculate_angle_between_points(point1, point2)
# Calculamos el ángulo hacia el punto objetivo desde P1
target_angle = calculate_angle_between_points(point1, target_point)
# Calculamos el ángulo de giro necesario
turn_angle = calculate_turn_angle(current_angle, target_angle)

# Crear la figura y los ejes
fig, ax = plt.subplots()

# Plotear los puntos
ax.plot([point1[0], point2[0], target_point[0]], [point1[1], point2[1], target_point[1]], 'ro')
ax.text(point1[0], point1[1], 'P1', fontsize=12, ha='right')
ax.text(point2[0], point2[1], 'P2', fontsize=12, ha='right')
ax.text(target_point[0], target_point[1], 'Target', fontsize=12, ha='right')

# Dibujar las líneas entre los puntos
ax.plot([point1[0], point2[0]], [point1[1], point2[1]], 'r-')
ax.plot([point1[0], target_point[0]], [point1[1], target_point[1]], 'b-')

# Dibujar los vectores de los ángulos
ax.quiver(point1[0], point1[1], point2[0] - point1[0], point2[1] - point1[1], angles='xy', scale_units='xy', scale=1, color='r', label=f'Current Angle: {current_angle:.2f}°')
ax.quiver(point1[0], point1[1], target_point[0] - point1[0], target_point[1] - point1[1], angles='xy', scale_units='xy', scale=1, color='b', label=f'Target Angle: {target_angle:.2f}°')

# Configurar la gráfica
ax.set_xlim(0, 10)
ax.set_ylim(0, 10)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title(f'Angulo de giro necesario: {turn_angle:.2f}°')
ax.legend()

# Mostrar la gráfica
plt.grid(True)
plt.axhline(0, color='black',linewidth=0.5)
plt.axvline(0, color='black',linewidth=0.5)
plt.show()
