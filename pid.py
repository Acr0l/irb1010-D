from simple_pid import PID

class MotorController:
    def __init__(self, kp, ki, kd, setpoint):
        self.pidR = PID(kp, ki, kd, setpoint=setpoint)
        self.pidL = PID(kp, ki, kd, setpoint=setpoint)

    def update_setpoints(self, setpointR, setpointL):
        self.pidR.setpoint = setpointR
        self.pidL.setpoint = setpointL

    def compute_control(self, vel0, vel1):
        controlR = self.pidR(vel0)
        controlL = self.pidL(vel1)
        return controlR, controlL

if __name__ == "__main__":
    kp, ki, kd = 1.0, 0.1, 0.05
    setpoint_inicial = 100

    controlador = MotorController(kp, ki, kd, setpoint_inicial)

    nuevo_setpointR = 120
    nuevo_setpointL = 110
    controlador.update_setpoints(nuevo_setpointR, nuevo_setpointL)

    velocidad_actualR = 105
    velocidad_actualL = 95

    controlR, controlL = controlador.compute_control(velocidad_actualR, velocidad_actualL)

    print(f"ControlR: {controlR}, ControlL: {controlL}")