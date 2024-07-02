from simple_pid import PID
import time
import matplotlib.pyplot as plt

class DosRuedasAutoController:
    def __init__(self, Kp_angle, Ki_angle, Kd_angle, Kp_distance, Ki_distance, Kd_distance):
        # Crear controladores pid
        self.pid_angle = PID(Kp_angle, Ki_angle, Kd_angle, setpoint=0)
        self.pid_distance = PID(Kp_distance, Ki_distance, Kd_distance, setpoint=0)
    
    def update(self, target_angle, current_angle, target_distance, current_distance, dt):
        # Actualizar setpoints
        self.pid_angle.setpoint = target_angle
        self.pid_distance.setpoint = target_distance

        # Calcular output pid
        angle_control = self.pid_angle(current_angle)
        distance_control = self.pid_distance(current_distance)
        
        # Determinar voltaje para las ruedas
        voltage_left = distance_control - angle_control
        voltage_right = distance_control + angle_control
        
        # Limitar voltaje +-7
        voltage_left = max(min(voltage_left, 7), -7)
        voltage_right = max(min(voltage_right, 7), -7)
        
        return voltage_left, voltage_right

def simular_movimiento(target_angle, target_distance, current_angle, current_distance, Kp_angle, Ki_angle, Kd_angle, Kp_distance, Ki_distance, Kd_distance, total_time=40, dt=0.1):
    # Crear el controlador
    car_controller = DosRuedasAutoController(Kp_angle, Ki_angle, Kd_angle, Kp_distance, Ki_distance, Kd_distance)

    #Simulación de parametros
    time_elapsed = 0

    #Listas para el plot
    time_list = []
    distance_list = []
    angle_list = []
    voltage_left_list = []
    voltage_right_list = []

    while time_elapsed < total_time:
        voltage_left, voltage_right = car_controller.update(target_angle, current_angle, target_distance, current_distance, dt)
        
        print(f"Time: {time_elapsed:.2f} s, Left Voltage: {voltage_left:.2f} V, Right Voltage: {voltage_right:.2f} V, Current Distance: {current_distance:.2f} m, Current Angle: {current_angle:.2f} degrees")

        current_angle += (voltage_right - voltage_left) * dt
        current_distance += (voltage_left + voltage_right) * dt / 2

        # Guardar data para el plot
        time_list.append(time_elapsed)
        distance_list.append(current_distance)
        angle_list.append(current_angle)
        voltage_left_list.append(voltage_left)
        voltage_right_list.append(voltage_right)
        
        time.sleep(dt)
        time_elapsed += dt

    plt.figure(figsize=(10, 8))
    plt.subplot(3, 1, 1)
    plt.plot(time_list, distance_list, label='Distance')
    plt.axhline(y=target_distance, color='r', linestyle='--', label=f'Target Distance {target_distance}m')
    plt.xlabel('Time (s)')
    plt.ylabel('Distance (m)')
    plt.legend()
    plt.grid(True)
    
    plt.subplot(3, 1, 2)
    plt.plot(time_list, angle_list, label='Angle', color='orange')
    plt.axhline(y=target_angle, color='b', linestyle='--', label=f'Target Angle {target_angle} degrees')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (degrees)')
    plt.legend()
    plt.grid(True)
    
    plt.subplot(3, 1, 3)
    plt.plot(time_list, voltage_left_list, label='Left Wheel Voltage', color='green')
    plt.plot(time_list, voltage_right_list, label='Right Wheel Voltage', color='purple')
    plt.xlabel('Time (s)')
    plt.ylabel('Voltage (V)')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()

# Ejemplo de uso
target_angle = 90  # angulo en grados
target_distance = 5  # avance en metros
current_angle = 0  # angulo actual en grados
current_distance = 0  # distancia actual en metro

Kp_angle, Ki_angle, Kd_angle = 1.0, 0.1, 0.05
Kp_distance, Ki_distance, Kd_distance = 1.0, 0.1, 0.05

simular_movimiento(target_angle, target_distance, current_angle, current_distance, Kp_angle, Ki_angle, Kd_angle, Kp_distance, Ki_distance, Kd_distance)
